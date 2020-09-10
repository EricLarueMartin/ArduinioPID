/**********************************************************************************************
 * Arduino PID Library
 * to make calculation easier the time units on Ki and Kd are in units of the sample time
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID.h>

//Constructor

PID::PID(double* iInput, double* iOutput, double* iSetpoint,
        double iKp, double iKi, double iKd)
{

    Output = iOutput;
    Input = iInput;
    Setpoint = iSetpoint;
    Kp=iKp;
    Ki=iKi;
    Kd=iKd;
	inAuto = false;
    outMin = 0;
    outMax = 1;
    SampleTime = 1000;							//default Controller Sample Time is 0.1 seconds
    lastInput = *Input;
    lastTime = millis();
}


/* Compute() **********************************************************************
 *     This function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = *Setpoint - *Input;
      ITerm+= (Ki * error);
      double dTerm = (*Input - lastInput);

//      if(ITerm > outMax) ITerm= outMax;
//      else if(ITerm < outMin) ITerm= outMin;
      /*Compute PID Output*/
      *Output = Kp * error + ITerm - Kd * dTerm;

	  if(*Output > outMax) {
            *Output = outMax;
            if (Ki<1e-9) ITerm = 0;
            else ITerm = *Output + Kd * dTerm - Kp * error;
	  }
      else if(*Output < outMin) {
            *Output = outMin;
            if (Ki<1e-9) ITerm = 0;
            else ITerm = *Output + Kd * dTerm - Kp * error;
      }

      /*Remember some variables for next time*/
      lastInput = *Input;
      lastTime = now;
	  return true;
   }
   else return false;
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(bool Mode)
{
    inAuto = Mode;
    if(inAuto)
        PID::Initialize();
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a seamless transition
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
    ITerm = *Output + Kp * (*Setpoint - *Input); // set history such that calculated power is current power
    lastInput = *Input;
    lastTime = millis();
}

/* SetOuput()****************************************************************
 *	Sets Output and sets ITerm to give desired Output
 ******************************************************************************/
void PID::SetOutput(double val)
{
    *Output = val;
    ITerm = *Output + Kp * (*Setpoint - *Input); // set history such that calculated power is current power
}


/* Status Functions*************************************************************
 ******************************************************************************/

bool PID::GetMode()
{
    return (inAuto);
}

