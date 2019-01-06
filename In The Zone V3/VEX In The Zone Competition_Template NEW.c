#pragma config(Sensor, in1,    ClawArmPot,     sensorPotentiometer)
#pragma config(Sensor, in2,    ArmPot,         sensorPotentiometer)
#pragma config(Sensor, in4,    MobilePot,      sensorPotentiometer)
#pragma config(Sensor, in5,    ClawPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl2,  Switch2,        sensorTouch)
#pragma config(Sensor, dgtl3,  BumpTouch2,     sensorTouch)
#pragma config(Sensor, dgtl5,  BumpTouch1,     sensorTouch)
#pragma config(Sensor, dgtl9,  EncRight,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder)
#pragma config(Motor,  port1,           MobileGoalR,   tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, PIDControl, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port3,           Right1,        tmotorVex393_MC29, PIDControl, reversed, driveRight, encoderPort, dgtl9)
#pragma config(Motor,  port4,           Claw,          tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           Right2,        tmotorVex393_MC29, PIDControl, reversed, driveRight, encoderPort, dgtl9)
#pragma config(Motor,  port9,           Left2,         tmotorVex393_MC29, PIDControl, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port10,          MobileGoalL,   tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autostraight (int leftmotor, int rightmotor) {
if ((SensorValue[EncLeft]) < (SensorValue [EncRight])){
motor[Left1]= leftmotor + 10;
motor[Left2]= leftmotor + 10;
motor[Right1]=  rightmotor - 10;
motor[Right2] = rightmotor - 10;

}
else if ((SensorValue[EncLeft]) > (SensorValue [EncRight])){
motor[Left1]= leftmotor - 10;
motor[Left2]= leftmotor - 10;
motor[Right1]= 10 + rightmotor;
motor[Right2] = 10 + rightmotor;

}

else {
motor[Left1]= leftmotor;
motor[Left2]= leftmotor;
motor[Right1]= rightmotor;
motor[Right2] = rightmotor;
}
}

task autonomous()
{
 if (vexRT[Btn8R] ==1) {

SensorValue [EncLeft] = 0;
SensorValue[EncRight] = 0;

while ((SensorValue[ClawArmPot]>2700 )&& (SensorValue[Switch2]==0)){
	motor[ClawArmR] = 80;
}
// Clawarm move up

while ((SensorValue[MobilePot]< 2100)){
	motor[MobileGoalL] = 80;
	motor[MobileGoalR] = 80;
}
// extent mobilegoal
while (SensorValue [EncLeft] < 500) {
autostraight(100,100)	;
}
//move forward

while (SensorValue [ClawArmPot] <800) {
motor [ClawArmR] = -80;

}

//lower clawlift

while (SensorValue [ClawPot] <500) {
motor [Claw] = 80;

}
// Release cone
while (SensorValue [ClawArmPot] >2700) {
motor [ClawArmR] = 80;

}
// raise clawlift
while ((SensorValue[BumpTouch2]==0)|| (SensorValue[MobilePot] < 5)) {
	motor[MobileGoalL] = -100;
	motor[MobileGoalR] = -100;
}
// move mobilegoal up
SensorValue [EncLeft] = 0;
SensorValue[EncRight] = 0;

while (SensorValue [EncLeft] < 500){
autostraight(-100,-100);
}
//moves back

SensorValue [EncLeft] = 0;
SensorValue[EncRight] = 0;

while (SensorValue [EncLeft] <100) {
motor[Left1]= -50;
motor[Left2]= -50;
motor[Right1]= 50;
motor[Right2] = 50;
}
//turn left
SensorValue [EncLeft] = 0;
SensorValue[EncRight] = 0;

while (SensorValue [EncLeft] < 100){
autostraight(100,100);
}
//move forward to the center position

while (SensorValue [EncLeft] <100) {
motor[Left1]= -50;
motor[Left2]= -50;
motor[Right1]= 50;
motor[Right2] = 50;
}
// turn left again
while (SensorValue [EncLeft] < 100){
autostraight(70,70);
}
// mobe forward to the one point zone

while ((SensorValue[MobilePot]< 2100)){
	motor[MobileGoalL] = 80;
	motor[MobileGoalR] = 80;
}

//lower the claw and place the goal down
SensorValue [EncLeft] = 0;
SensorValue[EncRight] = 0;
while (SensorValue [EncLeft] < 100) {
autostraight(-70,-70);
}
//move back
}
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


float KP2 = -0.012;


void PID2 (int optimal2)
{
	int error2 = (optimal2 - SensorValue [ClawArmPot]);
	int output2 = error2 * KP2;
	motor [ClawArmR] = output2;

}
int Ayush2;

/*
int Ayush3;
float KP3 = -0.005;
void PID3 (int optimal3)
{
	int error3 = (optimal3 - SensorValue [ClawPot]);
	int output3 = error3 * KP3;
	motor [Claw] = output3;

}

*/

task usercontrol()
{
  // User control code here, inside the loop

  while (true)
  {
   		motor[Left1]  = (vexRT[Ch3] + vexRT[Ch1])*0.9;
		motor[Left2]  = (vexRT[Ch3] + vexRT[Ch1])*0.9;
		motor[Right1] = (vexRT[Ch3] - vexRT[Ch1])*0.9;
		motor[Right2] = (vexRT[Ch3] - vexRT[Ch1])*0.9;


//_______________________________________________________________

		if(vexRT[Btn6U] == 1)       	//If Button 6U is pressed...
		{
			if ((vexRT[Btn6U] == 1) && (SensorValue[ClawArmPot] < 2800)&& (SensorValue[Switch2]==0) ){
			motor[ClawArmR] = 100;  		// open

	}
else if (SensorValue[Switch2]==1){
motor[ClawArmR]=0;
}

	else {
	Ayush2 = SensorValue[ClawArmPot] -20;
	}
		}
	else if(vexRT[Btn6D] == 1)  	//Else, if button 6D is pressed...
		{
			if ((vexRT[Btn6D] == 1) && (SensorValue[ClawArmPot] > 900) && (SensorValue[BumpTouch1] ==0)){
			motor[ClawArmR] = -100; 		//...close the gripper.
		}
		else {
		Ayush2 = SensorValue[ClawArmPot] + 20;
		}
	}


		else {
		if((SensorValue[ClawArmPot] < 900)||(SensorValue[ClawArmPot]>2800)||(SensorValue[Switch2]==1))
	{motor[ClawArmR]=0;
		}
		//Else (neither button is pressed)...
//if ((SensorValue[ClawArmPot]> 244)&& (SensorValue[ClawArmPot]< 248))
//{
//	motor[ClawArmR] = 0;
//}
else {

		PID2(Ayush2);   		//...stop the gripper.
		}
	//	{
	//	motor[ClawArmR] = 0;


		}

	/*	if(vexRT[Btn6U] == 1)       	//If Button 6U is pressed...
		{
			while (( SensorValue [Switch2]==0)&&(vexRT[Btn6U] == 1) ){
			motor[ClawArmR] = 70;  		// open
			Ayush2 = SensorValue[ClawArmPot] -20;
	//	}
}
		}
		else if(vexRT[Btn6D] == 1)  	//Else, if button 6D is pressed...
		{
			while ((SensorValue [BumpTouch1]==0 )&& (vexRT[Btn6D] == 1)){
			motor[ClawArmR] = -40; 		//...close the gripper.
			Ayush2 = SensorValue[ClawArmPot] + 20;
	}
	//}
}
		else if ((SensorValue [Switch2]==1 )&& (vexRT[Btn6U] == 0)){                     		//Else (neither button is pressed)...

			PID2(Ayush2);   		//...stop the gripper.
	//	{
	//	motor[ClawArmR] = 0;


		}
		else if ((SensorValue [BumpTouch1]==1 )&& (vexRT[Btn6D] == 0)){                     		//Else (neither button is pressed)...

			PID2(Ayush2);   		//...stop the gripper.
	//	{
	//	motor[ClawArmR] = 0;


		}*/



//-------------------------------------------------------------------------------------
//CLAW
if(vexRT[Btn5U] == 1)       	//If Button 6U is pressed...
		{

			motor[Claw] = 60;  		// open
		//	Ayush3 = SensorValue[ClawPot];
	//	}

		}
		else if(vexRT[Btn5D] == 1)  	//Else, if button 6D is pressed...
		{

			motor[Claw] = -60; 		//...close the gripper.
//		Ayush3 = SensorValue[ClawPot];
		//}

}
		else                      		//Else (neither button is pressed)...
		{
//		PID3(Ayush3);   		//...stop the gripper.
	motor[Claw] = 0;
		}

		//----------------------------------------------------------------------------------
//Button Mode
if(vexRT[Btn8U] == 1)       	//If Button 6U is pressed...
		{
			while ((vexRT[Btn8U] == 1) && (SensorValue[ArmPot]>1700 )){
			motor[TL] = 100;
				motor[TR] = 100;


		}
	}
		//}
		else if(vexRT[Btn8D] == 1)  	//Else, if button 6D is pressed...
		{
			while ((vexRT[Btn8D] == 1)  && (SensorValue[ArmPot]< 3400)){
			motor[TL] = -100;
				motor[TR] = -100;

}
}
		else                      		//Else (neither button is pressed)...
		{
			motor[TL] = 0;
				motor[TR] = 0;




		}
//----------------------------------------------------------------------------------------------

//Button Mode
if(vexRT[Btn7U] == 1)       	//If Button 6U is pressed...
		{
			while ((vexRT[Btn7U] == 1) &&(SensorValue[MobilePot]< 2100) ){
			motor[MobileGoalL] = 80;
				motor[MobileGoalR] = 80;


		}
	}
		//}
		else if(vexRT[Btn7D] == 1)  	//Else, if button 6D is pressed...
		{
			while ((vexRT[Btn7D] == 1)  &&(SensorValue[BumpTouch2]==0 )){
		motor[MobileGoalL] = -80;
				motor[MobileGoalR] = -80;



}
}
		else                      		//Else (neither button is pressed)...
		{
		motor[MobileGoalL] = 0;
				motor[MobileGoalR] = 0;

			}
  }
}
