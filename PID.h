#ifndef PID_h
#define PID_h
#define LIBRARY_VERSION	1.0.0

class PID
{


  public:

  //commonly used functions **************************************************************************
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and
        double, double, double);     //   Setpoint.  Initial tuning parameters are also set here

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles.

	void SetMode(bool);          // Use when switching from manual to automatic
	void Initialize();               // Sets up

	double Kp;                  // * (P)roportional Tuning Parameter
    double Ki;                  // * (I)ntegral Tuning Parameter. Units of inverse SampleTime
    double Kd;                  // * (D)erivative Tuning Parameter. Units of SampleTime

    double *Input;              // * Pointers to the Input, Output, and Setpoint variables
    double *Output;
    double *Setpoint;

	unsigned long SampleTime;   // Sample time in milliseconds, default is 1000
	double outMin, outMax;      // default limits are 0 to 1
	bool GetMode();        // returns the private inAuto
	void SetOutput(double); // sets ITerm to give desired output

	private:
	bool inAuto;                // Compute *output? default is true
	unsigned long lastTime;
	double ITerm, lastInput;

};
#endif

