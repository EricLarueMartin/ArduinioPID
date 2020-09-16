#ifndef PID_h
#define PID_h

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

class PID
{


public:

    //commonly used functions **************************************************************************
    PID(double*, double*, double,        // * constructor.  links the PID to the Input, Output, and
        double, double, double);     //   SetPoint.  Initial tuning parameters are also set here

    double Kp;                  // * (P)roportional Tuning Parameter
    double Ki;                  // * (I)ntegral Tuning Parameter. Units of inverse SampleTime
    double Kd;                  // * (D)erivative Tuning Parameter. Units of SampleTime

    double *input;              // * Pointers to the input, output, and setPoint variables
    double *output;
    double setPoint;

    unsigned long sampleTime = 10;   // Minimum time between samples for history, all samples shorter than this will be averaged
    unsigned sampleCount = 0; // Number of samples taken this period
    double sampleTotal = 0.0; // Total of samples taken this period
    double outMin = 0.0;
    double outMax = 1.0;      // default limits are 0 to 1
    bool inAuto = false;                // Compute *output? default is true
    bool Compute();                       // Performs the PID calculation.  Should be called every time loop() cycles.

    void SetOutput(double); // sets ITerm to give desired output, simply setting the value of output won't be effective when in auto mode

    unsigned long lastTime;
    double pTerm,lastSample;
    double iTerm = 0.0;
    double dTerm = 0.0;


};

#endif
