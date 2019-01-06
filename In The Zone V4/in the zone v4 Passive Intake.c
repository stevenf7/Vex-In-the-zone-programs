#pragma config(Sensor, in1,    ClawArmPotL,    sensorPotentiometer)
#pragma config(Sensor, in2,    ClawArmPotR,    sensorPotentiometer)
#pragma config(Sensor, in3,    ArmPot,         sensorPotentiometer)
#pragma config(Sensor, in4,    MobilePotL,     sensorPotentiometer)
#pragma config(Sensor, in5,    MobilePotR,     sensorPotentiometer)
#pragma config(Sensor, dgtl2,  Switch2,        sensorTouch)
#pragma config(Sensor, dgtl3,  BumpTouch2,     sensorTouch)
#pragma config(Sensor, dgtl5,  BumpTouch1,     sensorTouch)
#pragma config(Sensor, dgtl9,  EncRight,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder)
#pragma config(Motor,  port1,           Left2,        tmotorVex393_HBridge, openLoop, reversed, driveRight)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port3,           MobileGoalL,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           ClawArmL,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           MobileGoalR,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           Right1,        tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port10,          Right2,         tmotorVex393_HBridge, openLoop, reversed, driveLeft)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

int ClawliftOptimalL;
int ClawliftOptimalR;

void PID_ClawLift(int optimalL,int optimalR,int actualL,int actualR)
{

	float kp = 1.3;
	float ki = 0.1;
	float kd = 0.2;

	// Tuning: set the kp value first, go from 0 to the nearest value which it oscillates, then adjust kd value to prevent it from oscillating
	//then increase the kp value again and repeat the procedure untill kd can no longer preventing kd from ocilating
	// then find the last kp and kd value.
	// for ki, find the value that smooth that out the most
	float ks = 1.0;

	//ks is the autostraight constant. start at 1, increase it slowly (by 0.01) untill both sides sync
	float currentL = 0;
	float errorTl;
	float lastErrorL;
	float proportionL;
	float integralL;
	float derivativeL;
	float currentR = 0;
	float errorTr;
	float lastErrorR;
	float proportionR;
	float integralR;
	float derivativeR;
	actualL = SensorValue[ClawArmPotL];
	actualR = SensorValue[ClawArmPotR];
	while(true){


		float errorL = optimalL - SensorValue[ClawArmPotL];
		float errorR = optimalR - SensorValue[ClawArmPotR];
		if (abs(errorL) < 1000 && errorL !=0)
		{
			errorTl = errorL;
		}
		else{
			errorTl = 0;
		}

		if (abs(errorR) < 1000 && errorR !=0)
		{
			errorTr = errorR;
		}
		else{
			errorTr = 0;
		}

		if (abs(errorTl)> 50 / ki){
			errorTl = 50 / ki;
		}

		if (errorL ==0){
			derivativeL = 0;
		}

		if (abs(errorTr)> 50 / ki){
			errorTr = 50 / ki;
		}

		if (errorR ==0){
			derivativeR = 0;
		}



		proportionL = errorL * kp;
		proportionR = errorR * kp;
		integralL = errorTl  * ki;
		integralR = errorTr  * ki;
		derivativeL = (errorL - lastErrorL) * kd;
		derivativeR = (errorR - lastErrorR) * kd;

		lastErrorL = errorL;
		lastErrorR = errorR;

		currentL = proportionL + integralL + derivativeL;
		currentR = proportionR + integralR + derivativeR;

		int r =SensorValue[ClawArmPotR];
		int l =SensorValue[ClawArmPotL] - 200;

		if (r > l){
			motor[ClawArmR] = currentR / ks;
			motor[ClawArmL] = currentL * ks;
		}

		else if(r <l) {
			motor[ClawArmR] = currentR * ks;
			motor[ClawArmL] = currentL / ks;
		}

		else {
			motor[ClawArmR] = currentR;
			motor[ClawArmL] = currentL;
		}


		wait1Msec(40);
	}
}
//--------------------------------------------------------------------------------------------------------------------------------------------
void MobileGoal_AutoStraight (int MobileGoal_Power){
	while (true){
		int MG_Left = SensorValue[MobilePotL];
		int MG_Right=SensorValue[MobilePotR] - 83;
		float MobileGoal_Constant = 1.0;
		if (MG_Left < MG_Right){
			motor [MobileGoalR] = MobileGoal_Power * MobileGoal_Constant;
			motor [MobileGoalL] = MobileGoal_Power / MobileGoal_Constant;
		}
		else if (MG_Left > MG_Right){
			motor [MobileGoalL] = MobileGoal_Power * MobileGoal_Constant;
			motor [MobileGoalR] = MobileGoal_Power / MobileGoal_Constant;
		}
		else{
			motor [ MobileGoalL] = motor[MobileGoalR] = MobileGoal_Power;
		}
		wait1Msec(40);
		}

}

/*
int MobileGoalOptimal;
void PID_MobileGoal(int optimalML,int optimalMR,int actualML,int actualMR)
{

float Mkp = 1.3;
float Mki = 0.1;
float Mkd = 0.2;
// Tuning: set the kp value first, go from 0 to the nearest value which it oscillates, then adjust kd value to prevent it from oscillating
//then increase the kp value again and repeat the procedure untill kd can no longer preventing kd from ocilating
// then find the last kp and kd value.
// for ki, find the value that smooth that out the most

float currentML = 0;
float errorMTl;
float lastErrorML;
float proportionML;
float integralML;
float derivativeML;
float currentMR = 0;
float errorMTr;
float lastErrorMR;
float proportionMR;
float integralMR;
float derivativeMR;
actualML = SensorValue[MobilePotL];
actualMR = SensorValue[MobilePotR];

while(true){
float errorML = optimalML - SensorValue[MobilePotL];
float errorMR = optimalMR - SensorValue[MobilePotR];
if (abs(errorML) < 1000 && errorML !=0)
{
errorMTl = errorML;
}
else{
errorMTl = 0;
}

if (abs(errorMR) < 1000 && errorMR !=0)
{
errorMTr = errorMR;
}
else{
errorMTr = 0;
}

if (abs(errorMTl)> 50 / Mki){
errorMTl = 50 / Mki;
}

if (errorML ==0){
derivativeML = 0;
}

if (abs(errorMTr)> 50 / Mki){
errorMTr = 50 / Mki;
}

if (errorMR ==0){
derivativeMR = 0;
}



proportionML = errorML * Mkp;
proportionMR = errorMR * Mkp;
integralML = errorMTl  * Mki;
integralMR = errorMTr  * Mki;
derivativeML = (errorML - lastErrorML) * Mkd;
derivativeMR = (errorMR - lastErrorMR) * Mkd;

lastErrorML = errorML;
lastErrorMR = errorMR;

currentML = proportionML + integralML + derivativeML;
currentMR = proportionMR + integralMR + derivativeMR;
motor[MobileGoalR] = currentMR;
motor[MobileGoalL] = currentML;
}
}
*/
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


		//----------------------------------------------------------------------------------------------------------------------------------TUNING
		// Claw Arm Control Main Driver
		// Claw Arm Control Main Driver
		if(vexRT[Btn6U] == 1)
		{
			while(vexRT[Btn6U] ==1) {

				PID_ClawLift(3600,3600,SensorValue[ClawArmPotL],SensorValue[ClawArmPotR]);
			}

			/*	if ((vexRT[Btn6U] == 1) && (SensorValue[ClawArmPotL] < 2800)&& (SensorValue[Switch2]==0) ){
			motor[ClawArmL]= 	motor[ClawArmR]= 100;
			}
			else if (SensorValue[Switch2]==1){
			motor[ClawArmL]=	motor[ClawArmR]=0;
			}

			else {
			ClawliftOptimal = SensorValue[ClawArmPot];
			}
			*/
		}
		else if(vexRT[Btn6D] == 1)  	//Else, if button 6D is pressed...
		{
			while(vexRT[Btn6D] ==1) {

				PID_ClawLift(1100,1100,SensorValue[ClawArmPotL],SensorValue[ClawArmPotR]);
			}
		}
		/*if ((vexRT[Btn6D] == 1) && (SensorValue[ClawArmPot] > 900) && (SensorValue[BumpTouch1] ==0)){
		motor[ClawArmL]= 	motor[ClawArmR] = -100; 		//...close the gripper.

		}
		else {
		ClawliftOptimal = SensorValue[ClawArmPot];
		}
		*/



		else {

			PID_ClawLift(ClawliftOptimalL,ClawliftOptimalR,SensorValue[ClawArmPotL],SensorValue[ClawArmPotR]);
		}
		//}
		//---------------------------------------------------------------------------------------------------------------------------------------TUNING
		//Joystick lift system channel 2
		//equation: 0.641x^3 + 0.38x
		float c = vexRT[Ch2Xmtr2] / 127;
		float Ch2_ClawArmLift = (0.641* pow(c,3)+ 0.38 * c)*127;

		if ((SensorValue[ClawArmPotL] < 2800)&& (SensorValue[Switch2]==0) ){
			motor[ClawArmL] =  motor [ClawArmR]= Ch2_ClawArmLift;

			ClawliftOptimalL = SensorValue[ClawArmPotL];
			ClawliftOptimalR = SensorValue[ClawArmPotR];
		}

		else {

			PID_ClawLift(ClawliftOptimalL,ClawliftOptimalR,SensorValue[ClawArmPotL],SensorValue[ClawArmPotR]);
		}
		//--------------------------------------------------------------------------------------------------------------------------------------
		//Reverse Double 4 Bar Lift Main  Control
		if(vexRT[Btn5U] == 1)
		{
			while ((vexRT[Btn5U] == 1) && (SensorValue[ArmPot]>1700 )){
				motor[TL] = motor[TR] = 120;
			}
		}

		else if (vexRT[Btn5D] == 1){
			while ((vexRT[Btn5D] == 1)  && (SensorValue[ArmPot]< 3400)){
				motor[TL] = motor[TR] = -120;
			}
		}
		else
		{
			motor[TL] = motor[TR] = 0;
		}
		//-------------------------------------------------------------------------------------------------------------------------------------
		// Reverse Doulbe 4 Bar Partner Joystick
		float d = vexRT[Ch3Xmtr2] / 127;
		float Ch3_RD4BL = (0.4* pow(d,3)+ 0.6 * d)*127;

		motor [TR]= motor [TL]= Ch3_RD4BL;


		//---------------------------------------------------------------------------------------------------- TUNING
		if(vexRT[Btn8U] == 1)
		{
			while ((vexRT[Btn8U] == 1) && (SensorValue[BumpTouch2]==0 )){
				MobileGoal_AutoStraight (120);

			}
		}
		else if(vexRT[Btn8D] == 1)
		{

			while ((vexRT[Btn8D] == 1)  && (SensorValue[MobilePotR]< 2100)){
				MobileGoal_AutoStraight (-120);

			}
		}
		else
		{
			motor[MobileGoalR] = 	motor[MobileGoalL] = 0;

		}
		//-------------------------------------------------------------------------------------------------TUNING
		if(vexRT[Btn6UXmtr2] == 1)
		{
			while ((vexRT[Btn6UXmtr2] == 1) && (SensorValue[BumpTouch2]==0 )){
				MobileGoal_AutoStraight (120);
			}
		}
		else if(vexRT[Btn6DXmtr2] == 1)
		{
			while ((vexRT[Btn6DXmtr2] == 1)  && (SensorValue[MobilePotL]< 2100)){
				MobileGoal_AutoStraight (-120);
			}
		}
		else
		{
			motor[MobileGoalR] =	motor[MobileGoalL] = 0;
		}
		//---------------------------------------------------------------------------------------------------
	}
}