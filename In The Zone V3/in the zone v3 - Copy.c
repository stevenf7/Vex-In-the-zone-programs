#pragma config(Sensor, in1,    ClawArmPot,     sensorPotentiometer)
#pragma config(Sensor, in2,    ArmPot,         sensorPotentiometer)
#pragma config(Sensor, in4,    MobilePot,      sensorPotentiometer)
#pragma config(Sensor, dgtl2,  Switch2,        sensorTouch)
#pragma config(Sensor, dgtl3,  BumpTouch2,     sensorTouch)
#pragma config(Sensor, dgtl4,  RollTouch,      sensorTouch)
#pragma config(Sensor, dgtl5,  BumpTouch1,     sensorTouch)
#pragma config(Sensor, dgtl9,  EncRight,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder)
#pragma config(Motor,  port1,           MobileGoalR,   tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port3,           Right1,        tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port4,           Roller,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           Right2,        tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port9,           Left2,         tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port10,          MobileGoalL,   tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
float KP2 = -0.012;
int ClawliftOptimal;

void PID2 (int optimal2)
{
	int error2 = (optimal2 - SensorValue [ClawArmPot]);
	int output2 = error2 * KP2;
	motor [ClawArmR] = output2;

}
*/
//L for claw arm lift
//R for roller
int ClawliftOptimal;

void PID_ClawLift(int optimal,int actual)
{

float kp = 1.3;
float ki = 0.1;
float kd = 0.2;
// Tuning: set the kp value first, go from 0 to the nearest value which it oscillates, then adjust kd value to prevent it from oscillating
//then increase the kp value again and repeat the procedure untill kd can no longer preventing kd from ocilating
// then find the last kp and kd value.
// for ki, find the value that smooth that out the most

float currentL = 0;
float errorTl;
float lastErrorL;
float proportionL;
float integralL;
float derivativeL;

actual = SensorValue[ClawArmPot];

while(true){
float errorL = optimal - SensorValue[ClawArmPot];
if (errorL < 1000 && errorL !=0)
{
	errorTl = errorL;
}
else{
errorTl = 0;
}

if (errorTl> 50 / ki){
errorTl = 50 / ki;
}

if (errorL ==0){
	derivativeL = 0;
}

proportionL = errorL * kp;
//proportionR = errorR * kp;
integralL = errorTl  * ki;
//integralR = errorTr  * ki;
derivativeL = (errorL - lastErrorL) * kd;
//derivativeR = (errorR - lastErrorR) * kd;

lastErrorL = errorL;
//lastErrorR = errorR;

currentL = proportionL + integralL + derivativeL;


motor[ClawArmR] = currentL;
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
	// Claw Arm Control Main Driver
if(vexRT[Btn6U] == 1)
		{
			if ((vexRT[Btn6U] == 1) && (SensorValue[ClawArmPot] < 2800)&& (SensorValue[Switch2]==0) ){
				motor[ClawArmR]= 100;
			}
			else if (SensorValue[Switch2]==1){
				motor[ClawArmR]=0;
			}

			else {
				ClawliftOptimal = SensorValue[ClawArmPot];
			}
		}
		else if(vexRT[Btn6D] == 1)  	//Else, if button 6D is pressed...
		{
			if ((vexRT[Btn6D] == 1) && (SensorValue[ClawArmPot] > 900) && (SensorValue[BumpTouch1] ==0)){
				motor[ClawArmR] = -100; 		//...close the gripper.

				}
			else {
				ClawliftOptimal = SensorValue[ClawArmPot];
			}
		}


		else {
			if((SensorValue[ClawArmPot] < 900)||(SensorValue[ClawArmPot]>2800)||(SensorValue[Switch2]==1))
			{motor[ClawArmR]=0;

			}


			else {

				PID_ClawLift(ClawliftOptimal,SensorValue[ClawArmPot]);
			}
}
//---------------------------------------------------------------------------------------------------------------------------------------
//Joystick lift system channel 2
//equation: 0.641x^3 + 0.38x
float c = vexRT[Ch2Xmtr2] / 127;
float Ch2_ClawArmLift = (0.641* pow(c,3)+ 0.38 * c)*127;

if ((SensorValue[ClawArmPot] < 2800)&& (SensorValue[Switch2]==0) ){
motor [ClawArmR]= Ch2_ClawArmLift;
while (true){
ClawliftOptimal = SensorValue[ClawArmPot];
}
}
else
PID_ClawLift(ClawliftOptimal,SensorValue[ClawArmPot]);
//--------------------------------------------------------------------------------------------------------------------------------------
//Reverse Double 4 Bar Lift Main  Control
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
// REverse DOulbe 4 Bar Partner Joystick
float d = vexRT[Ch3Xmtr2] / 127;
float Ch3_RD4BL = (0.4* pow(d,3)+ 0.6 * d)*127;

motor [TR]= Ch3_RD4BL;
motor [TL]= Ch3_RD4BL;

//--------------------------------------------------------------------------------------------
//Rollerintake Main Driver
if(vexRT[Btn5U] == 1)
		{
			while ((vexRT[Btn5U] == 1) && (SensorValue[RollTouch]==0 )){
				motor[Roller] = 100;
			}
		}

else if (vexRT[Btn5D] == 1){
				motor[Roller] = -100;
		}
else
	{
			motor[Roller] = 0;
	}
//---------------------------------------------------------------------------------------------
	//Rollerintake Secondary Driver
if(vexRT[Btn5UXmtr2] == 1)
		{
			while ((vexRT[Btn5UXmtr2] == 1) && (SensorValue[RollTouch]==0 )){
				motor[Roller] = 100;
			}
		}

else if (vexRT[Btn5DXmtr2] == 1){
				motor[Roller] = -100;
		}
else
	{
			motor[Roller] = 0;
	}
//---------------------------------------------------------------------------------------------
	if(vexRT[Btn7U] == 1)
		{
			while ((vexRT[Btn7U] == 1) && (SensorValue[BumpTouch2]==0 )){
			motor[MobileGoalR] = 120;
			motor[MobileGoalL] = 120;
		}
	}
		else if(vexRT[Btn7D] == 1)
		{
			while ((vexRT[Btn7D] == 1)  && (SensorValue[MobilePot]< 2100)){
		motor[MobileGoalR] = -120;
		motor[MobileGoalL] = -120;
}
}
		else
		{
		motor[MobileGoalR] = 0;
		motor[MobileGoalL] = 0;
			}
//-------------------------------------------------------------------------------------------------
	if(vexRT[Btn6UXmtr2] == 1)
		{
			while ((vexRT[Btn6UXmtr2] == 1) && (SensorValue[BumpTouch2]==0 )){
			motor[MobileGoalR] = 120;
			motor[MobileGoalL] = 120;
		}
	}
		else if(vexRT[Btn6DXmtr2] == 1)
		{
			while ((vexRT[Btn6DXmtr2] == 1)  && (SensorValue[MobilePot]< 2100)){
		motor[MobileGoalR] = -120;
		motor[MobileGoalL] = -120;
}
}
		else
		{
		motor[MobileGoalR] = 0;
		motor[MobileGoalL] = 0;
			}
//---------------------------------------------------------------------------------------------------
}
}
