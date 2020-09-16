/**********************************************************************************************
 * Arduino PID Library
 * All time units are in sample period
 **********************************************************************************************/

#include <PID.h>

//Constructor

PID::PID(double* iInput, double* iOutput, double iSetPoint,
         double iKp, double iKi, double iKd)
{
    output = iOutput;
    input = iInput;
    setPoint = iSetPoint;
    Kp=iKp;
    Ki=iKi;
    Kd=iKd;
    inAuto = false;
    lastTime = millis();
    lastSample = *input;
    pTerm = Kp*(setPoint - sampleTotal); // positive indicates output should go up
}


/* Compute() **********************************************************************
    This function should be called every time "void loop()" executes.
    The function will decide for itself whether a new output needs to be computed.
    Returns true when the output is computed, false when output hasn't changed.
 **********************************************************************************/
bool PID::Compute()
{
    sampleTotal+=*input;
    ++sampleCount;
    // check the time
    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);
    if(timeChange>=sampleTime) // it it time for an update?
    {
        sampleTotal/=sampleCount; // average the samples in the current sample period
        double pTerm = setPoint - sampleTotal; // positive indicates output should go up
        iTerm += (Ki * pTerm); // use different to add to iTerm before applying Kp
        pTerm *= Kp; // apply Kp
        double dTerm = Kd*(lastSample - sampleTotal); // positive indicates output should go up
        if (inAuto)
        {
            // Compute PID Output
            *output = pTerm + iTerm + dTerm;
            // check limits
            if(*output > outMax)
            {
                // limit output
                *output = outMax;
                iTerm = outMax - pTerm - dTerm; // Calculate iTerm to produce maximum
            }
            else if(*output < outMin)
            {
                // limit output
                *output = outMin;
                iTerm = outMin - pTerm - dTerm; // Calculate iTerm to produce minimum
            }
        }
        else // if (inAuto)
            // if no in automatic control then adjust iTerm to what would produce current output
            // this keeps output from jumping if switched back to auto
            iTerm = *output - pTerm - dTerm;

        // save last sample period
        lastTime = now;
        lastSample = sampleTotal;
        // initialize new sample period
        sampleTotal=0.0;
        sampleCount=0;
        return true; // The output was recalculated, so return true
    }
    else //     if(timeChange>=SampleTime)
        return false; // Output wasn't recalculated, so return false
}

/* SetOuput()****************************************************************
 *	Sets Output and sets ITerm to give desired Output
 ******************************************************************************/
void PID::SetOutput(double val)
{
    *output = val;
    iTerm = val - pTerm - dTerm; // Calculate iTerm to produce val
}

