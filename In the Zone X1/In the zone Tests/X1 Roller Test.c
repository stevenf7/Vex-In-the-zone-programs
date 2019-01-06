#pragma config(Sensor, in1,    MobilePotL,     sensorPotentiometer)
#pragma config(Sensor, in2,    MobilePotR,     sensorPotentiometer)
#pragma config(Sensor, in3,    ArmPotL,        sensorPotentiometer)
#pragma config(Sensor, in4,    ArmPotR,        sensorPotentiometer)
#pragma config(Sensor, in5,    ClawArmPotL,    sensorPotentiometer)
#pragma config(Sensor, in6,    ClawArmPotR,    sensorPotentiometer)
#pragma config(Sensor, dgtl9,  EncRight,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder)
#pragma config(Motor,  port1,           BR,            tmotorVex393_HBridge, openLoop, reversed, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port3,           Roller,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           ClawArmL,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           MobileGoalR,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           Right1,        tmotorVex393_MC29, openLoop, reversed, driveRight, encoderPort, dgtl9)
#pragma config(Motor,  port10,          BL,            tmotorVex393_HBridge, openLoop, reversed, driveRight, encoderPort, dgtl9)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
while (true){
if (vexRT[Btn7DXmtr2]){
motor[Roller] = 120;
}
else if (vexRT[Btn8DXmtr2]){
motor[Roller] = -120;
}
else {
motor[Roller] = 30;
}
wait1Msec(50);
}
}
