#pragma config(Sensor, in1,    ClawArmPot,     sensorPotentiometer)
#pragma config(Sensor, in2,    ArmPot,         sensorPotentiometer)
#pragma config(Sensor, in4,    MobilePot,      sensorPotentiometer)
#pragma config(Sensor, in5,    ClawPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  EncRight,       sensorNone)
#pragma config(Sensor, dgtl2,  Switch2,        sensorTouch)
#pragma config(Sensor, dgtl3,  BumpTouch2,     sensorTouch)
#pragma config(Sensor, dgtl4,  BumpTouch1,     sensorTouch)
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder)
#pragma config(Motor,  port1,           MobileGoalR,   tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           Right1,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           Claw,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           Right2,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           Left2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          MobileGoalL,   tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//#pragma config(Motor,  port9,           BL,            tmotorVex393_MC29, openLoop,reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
//#pragma config(Motor,  port3,           BR,            tmotorVex393_MC29, openLoop)
//bumptouch 1 is on the claw. Lower limit
//Switch 2 is on the tower, upper limit
//bumptouch 2 is for the mobile goal

/*
int Ayush3;
float KP3 = 0.1;
void PID3 (int optimal3)
{
	int error3 = (optimal3 - SensorValue [ClawPot]);
	int output3 = error3 * KP3;
	motor [Claw] = output3;

}
*/

float KP2 = -0.01;

void PID2 (int optimal2)
{
	int error2 = (optimal2 - SensorValue [ClawArmPot]);
	int output2 = error2 * KP2;
	motor [ClawArmR] = output2;

}
int Ayush2;






task main()
{
while(true){
	//____________________________________________________________________________

		motor[Left1]  = (vexRT[Ch3] + vexRT[Ch4])*0.9;
		motor[Left2]  = (vexRT[Ch3] + vexRT[Ch4])*0.9;
		motor[Right1] = (vexRT[Ch3] - vexRT[Ch4])*0.9;
		motor[Right2] = (vexRT[Ch3] - vexRT[Ch4])*0.9;


//_______________________________________________________________

		if(vexRT[Btn6U] == 1)       	//If Button 6U is pressed...
		{
			if ((vexRT[Btn6U] == 1) ){
			motor[ClawArmR] = 80;  		// open

	}

else {
	Ayush2 = SensorValue[ClawArmPot] -20;
	}
		}
		else if(vexRT[Btn6D] == 1)  	//Else, if button 6D is pressed...
		{
			if ((vexRT[Btn6D] == 1)){
			motor[ClawArmR] = -40; 		//...close the gripper.
		}
		else {
		Ayush2 = SensorValue[ClawArmPot] + 20;
		}
	}


		else {                     		//Else (neither button is pressed)...
if ((SensorValue[ClawArmPot]> 244)&& (SensorValue[ClawArmPot]< 248))
{
	motor[ClawArmR] = 0;
}
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

if(vexRT[Btn5U] == 1)       	//If Button 6U is pressed...
		{
			while ((SensorValue [ClawPot]> 300)&&(vexRT[Btn5U] == 1)){
			motor[Claw] = 50;  		// open
			//Ayush3 = SensorValue[ClawPot];
	//	}
	}
		}
		else if(vexRT[Btn5D] == 1)  	//Else, if button 6D is pressed...
		{
			while ((SensorValue [ClawPot]< 700)&&(vexRT[Btn5D] == 1)){
			motor[Claw] = -50; 		//...close the gripper.
		//	Ayush3 = SensorValue[ClawPot];
		//}
	}
}
		else                      		//Else (neither button is pressed)...
		{
		//	PID3(Ayush3);   		//...stop the gripper.
motor[Claw] = 0;
		}

		//----------------------------------------------------------------------------------
//Button Mode
if(vexRT[Btn8U] == 1)       	//If Button 6U is pressed...
		{
			while ((vexRT[Btn8U] == 1) && (SensorValue[ArmPot]>1700 )){
			motor[TL] = 80;
				motor[TR] = 80;
				//	motor[BL] = 50;
					//	motor[BR] = 50;// open

		}
	}
		//}
		else if(vexRT[Btn8D] == 1)  	//Else, if button 6D is pressed...
		{
			while ((vexRT[Btn8D] == 1)  && (SensorValue[ArmPot]< 3400)){
			motor[TL] = -80;
				motor[TR] = -80;
			//		motor[BL] = -50;
					//	motor[BR] = -50;		//...close the gripper.

		//}
	//}
}
}
		else                      		//Else (neither button is pressed)...
		{
			motor[TL] = 0;
				motor[TR] = 0;
			//		motor[BL] = 0;
				//		motor[BR] = 0;
			//
	//	}



		}
//----------------------------------------------------------------------------------------------

//Button Mode
if(vexRT[Btn7U] == 1)       	//If Button 6U is pressed...
		{
			while ((vexRT[Btn7U] == 1) && (SensorValue[BumpTouch2]==0 )){
			motor[MobileGoalL] = 80;
				motor[MobileGoalR] = 80;
				//	motor[BL] = 50;
					//	motor[BR] = 50;// open

		}
	}
		//}
		else if(vexRT[Btn7D] == 1)  	//Else, if button 6D is pressed...
		{
			while ((vexRT[Btn7D] == 1)  && (SensorValue[MobilePot]< 2100)){
		motor[MobileGoalL] = -80;
				motor[MobileGoalR] = -80;
			//		motor[BL] = -50;
					//	motor[BR] = -50;		//...close the gripper.

		//}
	//}
}
}
		else                      		//Else (neither button is pressed)...
		{
		motor[MobileGoalL] = 0;
				motor[MobileGoalR] = 0;
			}
			//		motor[BL] = 0;
				//		motor[BR] = 0;
			//
	//	}

//---------------------------------------------------------------------------------------------





}
}
