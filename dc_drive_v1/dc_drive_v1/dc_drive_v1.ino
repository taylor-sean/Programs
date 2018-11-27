/* ###############################################
 I/O Assignments
############################################### */
int _chSpeedSet = A0,  // Speed setpoint
_chKp = A1,  // Proportional coefficient reading for PID controller
_chKi = A2,  // Integral coefficient reading for PID controller
_chKd = A3,  // Derivative coefficient reading for PID controller
_chMotorCmdCCW = 3,   // PWM output to motor for counter-clockwise turn
_chMotorCmdCW = 2,   // PWM output to motor for clockwise turn
_chSpeedRead = 24,  // Speed reading
_chDirection = 25;  // Direction selector reading

/* ###############################################
 Other Constants
############################################### */
#define _minRPM        0  // Minimum RPM to initiate direction changing
#define _maxRPM     6000  // Maximum RPM limit
#define _DiscSlots    20  // Qty of slots on Index Disc

/* ###############################################
 Global Variables
############################################### */
boolean Direction, prevDirection;
float RPM = 0.0, RPMset = 0.0, OutputRPM = 0.0,
Kp = 0.0, Ki = 0.0, Kd = 0.0,
Kpmax = 2.0, Kimax = 1.0, Kdmax = 1.0,
E = 0.0, Eprev = 0.0, dT = 1.0;

/* ###############################################
 readFrequency(_DI_FrequencyCounter_Pin, _ReadingSpeed)
 Frequency Reading Function
 Input Parameters:
 (int)   _DI_FrequencyCounter_Pin : Digital pin to be read
 (float) _ReadingSpeed____________: Custom reading speed between 0...10 (Note.1)

 Note.1: _ReadingSpeed is a value to specify how long shall the changes be counted.
		 It cannot be 0(zero), negative values or a value greater than 10.
		 When _ReadingSpeed changed, 1 second shall be divided by this value to calculate
		 required counting duration. For example;
		  - _ReadingSpeed = 0.1 -> input shall be counted during 10 seconds (=1/0.1)
		  - _ReadingSpeed = 0.5 -> input shall be counted during 2 seconds (=1/0.5)
		  - _ReadingSpeed = 2.0 -> input shall be counted during 0.5 seconds (=1/2)
		  - _ReadingSpeed = 4.0 -> input shall be counted during 0.25 seconds (=1/4)
		 Importantly note that, increasing of _ReadingSpeed is a disadvantage especially
		 on lower frequencies (generally below 100 Hz) since counting error increases
		 up to 20%~40% by decreasing frequency.
############################################### */

int readFrequency(int _DI_FrequencyCounter_Pin, float _ReadingSpeed)
{
	pinMode(_DI_FrequencyCounter_Pin, INPUT);
	byte _DigitalRead, _DigitalRead_Previous = 0;
	unsigned long _Time = 0, _Time_Init;
	float _Frequency = 0;
	if ((_ReadingSpeed <= 0) || (_ReadingSpeed > 10)) return (-1);
	else
	{
		_Time_Init = micros();
		do
		{
			_DigitalRead = digitalRead(_DI_FrequencyCounter_Pin);
			if ((_DigitalRead_Previous == 1) && (_DigitalRead == 0)) _Frequency++;
			_DigitalRead_Previous = _DigitalRead;
			_Time = micros();
		} while (_Time < (_Time_Init + (1000000 / _ReadingSpeed)));
	}
	return (_ReadingSpeed * _Frequency);
}
/* ########### End of readFrequency() ########### */
/* ############################################## */

/* ###############################################
 controllerPID(RangeMin, RangeMax, _E, _Eprev, _dT, _Kp, _Ki, _Kd)
 PID Controller Function
 Input Parameters:
 (float) RangeMin: Minimum limit for output
 (float) RangeMax: Maximum limit for output
 (float) _E_____: Current error signal
 (float) _Eprev : Previous error signal
 (float) _dT____: Time difference as seconds
 (float) _Kp____: Proportional coefficient
 (float) _Ki____: Integral coefficient
 (float) _Kp____: Derivative coefficient
 Adjustment procedure:
   1. Set Kp=0, Ki=0, Kd=0.
   2. Start to increase Kp until the system oscillates at fixed period (Pc) and note
	  critical gain Kc = Kp.
   3. Adjust final coefficients as follows.
	  for P-control only  : Kp = 0.50*Kc
	  for PI-control only : Kp = 0.45*Kc, Ki = 1.2/Pc
	  for PID-control     : Kp = 0.60*Kc, Ki = 2.0/Pc, Kd=Pc/8
   4. Fine tuning could be done by slightly changing each coefficient.
############################################### */

float controllerPID(float _E, float _Eprev, float _dT, float _Kp, float _Ki, float _Kd)
{
	float P, I, D;
	/*
	 Base Formula: U = _Kp * ( _E + 0.5*(1/_Ki)*(_E+_Eprev)*_dT + _Kd*(_E-_Eprev)/_dT );
	 */
	P = _Kp * _E;                             /* Proportional Component */
	I = _Kp * 0.5 * _Ki * (_E + _Eprev) * _dT;  /* Integral Component */
	D = _Kp * _Kd * (_E - _Eprev) / _dT;        /* Derivative Component */
	return (P + I + D);
}
/* ########### End of controllerPID() ########### */
/* ############################################## */

/* ###############################################
 Setup
############################################### */

void setup()
{
	//analogReadResolution(12);
	pinMode(_chDirection, INPUT);    // Direction selector reading
	pinMode(_chMotorCmdCCW, OUTPUT); // PWM output to motor for counter-clockwise turn
	pinMode(_chMotorCmdCW, OUTPUT);  // PWM output to motor for clockwise turn
	// Initial killing the PWM outputs to motor
	analogWrite(_chMotorCmdCCW, 0); analogWrite(_chMotorCmdCW, 0);
	// Initial reading for direction selection
	Direction = digitalRead(_chDirection);   // HIGH=CCW, LOW=CW
	prevDirection = Direction;
}


/* ###############################################
 Loop
############################################### */

void loop()
{
	// Initialization Time: Necessary for PID controller.
	int InitTime = micros();

	// Reading Inputs
	   /* Controller Coefficients */
	Kp = Kpmax * (float)analogRead(_chKp) / 4095;
	Ki = Kimax * (float)analogRead(_chKi) / 4095;
	Kd = Kdmax * (float)analogRead(_chKd) / 4095;
	/* Direction Selector */
	Direction = digitalRead(_chDirection);  /* HIGH=CCW, LOW=CW */
 /* Actual RPM and RPM Setpoint
	Note that maximum selectable RPM is 5000. */
	RPM = 60 * (float)readFrequency(_chSpeedRead, 4) / _DiscSlots;
	RPMset = 5000 * (float)analogRead(_chSpeedSet) / 4095;

	// Calculations and Actions
	   /* Error Signal, PID Controller Output and Final Output (PWM) to Motor */
	E = RPMset - RPM;
	float cPID = controllerPID(E, Eprev, dT, Kp, Ki, Kd);
	if (RPMset == 0) OutputRPM = 0;
	else OutputRPM = OutputRPM + cPID;
	if (OutputRPM < _minRPM) OutputRPM = _minRPM;
	if (OutputRPM > _maxRPM) OutputRPM = _maxRPM;

	/* Changing Direction when inverted */
	if (Direction != prevDirection)
	{
		/* Killing both of the PWM outputs to motor */
		analogWrite(_chMotorCmdCCW, 0); analogWrite(_chMotorCmdCW, 0);
		/* Wait until motor speed decreases */
		do
		{
			RPM = 60 * (float)readFrequency(_chSpeedRead, 4) / _DiscSlots;
		} while (RPM > _minRPM);
	}

	// Writing Outputs
	if (Direction == HIGH) analogWrite(_chMotorCmdCCW, (int)(255 * OutputRPM / _maxRPM));
	else analogWrite(_chMotorCmdCW, (int)(255 * OutputRPM / _maxRPM));

	// Storing Values generated on previous cycle
	Eprev = E; prevDirection = Direction;

	// Calculating control application cycle time and passed Seconds
	dT = float(micros() - InitTime) / 1000000.0;
}