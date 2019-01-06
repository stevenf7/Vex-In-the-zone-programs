#pragma config(Sensor, in1,    ClawArmPot,     sensorPotentiometer)
#pragma config(Sensor, in2,    ArmPot,         sensorPotentiometer)
#pragma config(Sensor, in4,    MobilePot,      sensorPotentiometer)
#pragma config(Sensor, in5,    ClawPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl2,  Switch2,        sensorTouch)
#pragma config(Sensor, dgtl3,  BumpTouch2,     sensorTouch)
#pragma config(Sensor, dgtl5,  BumpTouch1,     sensorTouch)
#pragma config(Sensor, dgtl9,  EncRight,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder)
#pragma config(Motor,  port1,           RollerIntake,  tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port3,           Right1,        tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port4,           ClawArmL,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           Right2,        tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port9,           Left2,         tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port10,          MobileGoalL,   tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

float KP2 = -0.012;
int ClawliftOptimal;

void PID2 (int optimal2)
{
	int error2 = (optimal2 - SensorValue [ClawArmPot]);
	int output2 = error2 * KP2;
	motor [ClawArmR] = output2;

}

void PID(){
float errorOT = 0
float lastTime = 0

float maxErrorOt = someconst
float lastError = 0

float ki = 0
float kd = 0
float kp = 0
float pid(setpoint, actual) {
  dt = time - lastTime
  lastTime = time

  float error = setpoint - actual

  errorOT += error * dt
  if(abs(errorOT) > maxErrorOt) {
    errorOT = maxErrorOt * errorOT / abs(errorOT)
  }

  float errDiff = (lastError - error) / dt

  return kP * error + kI * errorOT + kD * errDiff
}
}



task main()
{
while (true){
//Joystick control main driver
float ch3_drive;
float ch1_drive;
float a = vexRT[Ch3] / 127.0;
float b = vexRT[Ch1] / 127.0;
ch3_drive = (0.463 * pow(a,3) - 0.069* pow(a,2) - 0.614 * a) * 127;
ch1_drive = (0.463 * pow(b,3) - 0.069* pow(b,2) - 0.614 * b) * 127;


		motor[Left1]  = ((ch3_drive) + (ch1_drive));
		motor[Left2]  = ((ch3_drive) + (ch1_drive));
		motor[Right1] = ((ch3_drive) - (ch1_drive));
		motor[Right2] = ((ch3_drive) - (ch1_drive));

		//Equation: 0.463x^3-0.069x^2+0.614x


//----------------------------------------------------------------------------------------------------------------------------------
// Claw Arm Control Main Driver
if(vexRT[Btn6U] == 1)
		{
			if ((vexRT[Btn6U] == 1) && (SensorValue[ClawArmPot] < 2800)&& (SensorValue[Switch2]==0) ){
				motor[ClawArmR] = 100;  		// open
				motor[ClawArmL] = 100;
			}
			else if (SensorValue[Switch2]==1){
				motor[ClawArmR]=0;
				motor[ClawArmL]=0;
			}

			else {
				ClawliftOptimal = SensorValue[ClawArmPot] -20;
			}
		}
		else if(vexRT[Btn6D] == 1)  	//Else, if button 6D is pressed...
		{
			if ((vexRT[Btn6D] == 1) && (SensorValue[ClawArmPot] > 900) && (SensorValue[BumpTouch1] ==0)){
				motor[ClawArmR] = -100; 		//...close the gripper.
				motor[ClawArmL] = -100;
				}
			else {
				ClawliftOptimal = SensorValue[ClawArmPot] + 20;
			}
		}


		else {
			if((SensorValue[ClawArmPot] < 900)||(SensorValue[ClawArmPot]>2800)||(SensorValue[Switch2]==1))
			{motor[ClawArmR]=0;
				motor[ClawArmL]=0;
			}


			else {

				PID2(ClawliftOptimal);
			}
}
//---------------------------------------------------------------------------------------------------------------------------------------
//Joystick lift system channel 2
//equation: 0.641x^3 + 0.38x
float c = vexRT[Ch2Xmtr2] / 127;
float Ch2_ClawArmLift = (0.641* pow(c,3)+ 0.38 * c)*127;

if ((SensorValue[ClawArmPot] < 2800)&& (SensorValue[Switch2]==0) ){
motor [ClawArmL]= Ch2_ClawArmLift;
motor [ClawArmR]= Ch2_ClawArmLift;
}
else
PID2(ClawliftOptimal);
//--------------------------------------------------------------------------------------------------------------------------------------
//Reverse Double 4 Bar Lift Main Joystick Control
if(vexRT[Btn8U] == 1)
		{
			while ((vexRT[Btn8U] == 1) && (SensorValue[ArmPot]>1700 )){
				motor[TL] = 100;
				motor[TR] = 100;
			}
		}

else if (vexRT[Btn8D] == 1){
			while ((vexRT[Btn8D] == 1)  && (SensorValue[ArmPot]< 3400)){
				motor[TL] = -100;
				motor[TR] = -100;
			}
		}
else
	{
			motor[TL] = 0;
			motor[TR] = 0;
	}
//-------------------------------------------------------------------------------------------------------------------------------------
//Partner Joystick

}
}
