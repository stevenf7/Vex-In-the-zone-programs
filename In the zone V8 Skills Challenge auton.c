#pragma config(Sensor, in1,    MobilePotL,     sensorPotentiometer)
#pragma config(Sensor, in2,    MobilePotR,     sensorPotentiometer)
#pragma config(Sensor, in3,    ArmPotL,        sensorPotentiometer)
#pragma config(Sensor, in4,    ArmPotR,        sensorPotentiometer)
#pragma config(Sensor, in5,    ClawArmPotL,    sensorPotentiometer)
#pragma config(Sensor, in6,    ClawArmPotR,    sensorPotentiometer)
#pragma config(Sensor, dgtl9,  EncRight,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder)
#pragma config(Motor,  port1,           Left2,         tmotorVex393_HBridge, openLoop, reversed, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port3,           MobileGoalL,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           ClawArmL,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port8,           MobileGoalR,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           Right1,        tmotorVex393_MC29, openLoop, reversed, driveRight, encoderPort, dgtl9)
#pragma config(Motor,  port10,          Right2,        tmotorVex393_HBridge, openLoop, reversed)

int optimalL;
int optimalR;
int MoptimalL;
int MoptimalR;
int CoptimalL;
int CoptimalR;

task PID_ClawLift()
{

	float ckp = 0.37; //tune
	float cki = 0.00001; //tune
	float ckd = 0.55; //tune
	//float ks = 1.0; //tune

	//ks is the autostraight constant.
	float ccurrentL ;
	float cerrorTl;
	float clastErrorL = 0;
	float cproportionL;
	float cintegralL;
	float cderivativeL;
	float ccurrentR ;
	float cerrorTr;
	float clastErrorR = 0;
	float cproportionR;
	float cintegralR;
	float cderivativeR;

	while(true){


		float cerrorL = CoptimalL - SensorValue[ClawArmPotL];
		float cerrorR = CoptimalR - SensorValue[ClawArmPotR];
		//	PidAtTargetL = abs(SensorValue[ClawArmPotL] - CoptimalL) < 50;
		//	PidAtTargetR = abs(SensorValue[ClawArmPotR] - CoptimalR) < 50;
		/*
		if ((		PidAtTargetL = abs(SensorValue[ClawArmPotL] - optimalL) < 50) || (PidAtTargetR = abs(SensorValue[ClawArmPotR] - optimalR) < 50)){
		motor [ClawArmL] = motor [ ClawArmR] = 5;
		}
		*/

		if (abs(cerrorL) < 200 && cerrorL !=0)
		{
			cerrorTl = cerrorL;
		}
		else{
			cerrorTl = 0;
		}

		if (abs(cerrorR) < 200 && cerrorR !=0)
		{
			cerrorTr = cerrorR;
		}
		else{
			cerrorTr = 0;
		}

		if (abs(cerrorTl)> 50 / cki){
			cerrorTl = 50 / cki;
		}

		if (cerrorL ==0){
			cderivativeL = 0;
		}

		if (abs(cerrorTr)> 50 / cki){
			cerrorTr = 50 / cki;
		}

		if (cerrorR ==0){
			cderivativeR = 0;
		}




		cproportionL = cerrorL * ckp;
		cproportionR = cerrorR * ckp;
		cintegralL = cerrorTl  * cki;
		cintegralR = cerrorTr  * cki;
		cderivativeL = (cerrorL - clastErrorL) * ckd;
		cderivativeR = (cerrorR - clastErrorR) * ckd;

		clastErrorL = cerrorL;
		clastErrorR = cerrorR;

		ccurrentL = cproportionL + cintegralL + cderivativeL;
		ccurrentR = cproportionR + cintegralR + cderivativeR;
		// Autostraightening
		//int r =SensorValue[ClawArmPotR];
		//int l =SensorValue[ClawArmPotL] - 200; //tune, find difference

		/*if (r > l){
		motor[ClawArmR] = currentR / ks;
		motor[ClawArmL] = currentL * ks;
		}

		else if(r <l) {
		motor[ClawArmR] = currentR * ks;
		motor[ClawArmL] = currentL / ks;
		}

		else {
		*/
		motor[ClawArmR] = ccurrentR;
		motor[ClawArmL] = ccurrentL;
		//	}


		wait1Msec(40);

	}
	return;
}

task PID_MG()
{
	float kp = 0.3; //tune
	float ki = 0.00001; //tune
	float kd = 0.2; //tune

	float currentL ;
	float errorTl;
	float lastErrorL = 0;
	float proportionL;
	float integralL;
	float derivativeL;
	float currentR ;
	float errorTr;
	float lastErrorR = 0;
	float proportionR;
	float integralR;
	float derivativeR;

	while(true){


		float errorL = MoptimalL - SensorValue[MobilePotL];
		float errorR = MoptimalR - SensorValue[MobilePotR];
		//PidAtTargetL = abs(SensorValue[ArmPotL] - optimalL) < 50;
		//PidAtTargetR = abs(SensorValue[ArmPotR] - optimalR) < 50;


		if (abs(errorL) < 200 && errorL !=0)
		{
			errorTl = errorL;
		}
		else{
			errorTl = 0;
		}

		if (abs(errorR) < 200 && errorR !=0)
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

		motor[MobileGoalR] = currentR;
		motor[MobileGoalL] = currentL;



		wait1Msec(40);

	}
	return;
}

task PID_RD4B()
{
	float kp = 0.5; //tune
	float ki = 0.0001; //tune
	float kd = 0.4; //tune

	float currentL ;
	float errorTl;
	float lastErrorL = 0;
	float proportionL;
	float integralL;
	float derivativeL;
	float currentR ;
	float errorTr;
	float lastErrorR = 0;
	float proportionR;
	float integralR;
	float derivativeR;

	while(true){


		float errorL = optimalL - SensorValue[ArmPotL];
		float errorR = optimalR - SensorValue[ArmPotR];
		//PidAtTargetL = abs(SensorValue[ArmPotL] - optimalL) < 50;
		//PidAtTargetR = abs(SensorValue[ArmPotR] - optimalR) < 50;


		if (abs(errorL) < 200 && errorL !=0)
		{
			errorTl = errorL;
		}
		else{
			errorTl = 0;
		}

		if (abs(errorR) < 200 && errorR !=0)
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

		motor[TL] = -currentR;
		motor[TR] = -currentL;



		wait1Msec(40);

	}
	return;
}



int drive_optimalL;
int drive_optimalR;

task PID_Drive()
{

	float kp = 0.7; //tune
	float ki = 0.0001; //tune
	float kd = 0.8; //tune
	//float ks = 1.0; //tune

	//ks is the autostraight constant.
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
	SensorValue[EncLeft] = 0;
	SensorValue[EncRight] = 0;

	//clear the enc values first)

	while(true){


		float errorL = drive_optimalL - SensorValue[EncLeft];
		float errorR = drive_optimalR - SensorValue[EncRight];
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

		motor[Right1] = motor [Right2] = currentR;
		motor[Left1] = motor [Left2] = currentL;



		wait1Msec(40);
	}
}

//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------




void drive(int LeftDrive, int RightDrive){
	SensorValue (EncLeft) = SensorValue (EncRight) = 0;
	drive_optimalL = LeftDrive;
	drive_optimalR = RightDrive;
	//waitUntil (abs(SensorValue[EncLeft]) < LeftDrive - 50);
	wait1Msec (50);
	return;
}

void RD4B (int a, int b){
	optimalL = a;
	optimalR = b;
	waitUntil ( abs(SensorValue[ArmPotL]) > abs(a - 100));
}


void MG (int x, int y){
	MoptimalL = x;
	MoptimalR = y;
	waitUntil ( abs(SensorValue[MobilePotL]) > abs(x - 100));
}
void MG_mid () {
	MoptimalL = 2000;
	MoptimalR = 2100;

}
void MG_up (){
	MoptimalL = 3756;
	MoptimalR = 3850;
	waitUntil ( abs(SensorValue[MobilePotL]) > abs(3756 - 100));
}

void MG_down() {
	MoptimalL = 1400;
	MoptimalR = 1463;
	waitUntil ( abs(SensorValue[MobilePotL]) > abs(1400 - 100));
}

void a() {
RD4B (1000,1000);
wait1Msec(300);
return;
}

void b() {
MG_down();
wait1Msec(500);
return;
}

void c() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(1000,1000);
wait1Msec(1200);
return;
}

void d(){
MG_up();
wait1Msec(400);
return;
}


void e2() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(-500,-500);
wait1Msec(500);
return;
}

void e3() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(-310,310);
wait1Msec(400);
return;
}

void e4() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(-300,300);
wait1Msec(400);
return;
}

void e5() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(-310,310);
wait1Msec(400);
return;
}

void e6() {
MG_mid();
wait1Msec(300);
return;
}

//drive into the 20 point zone
void e6() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(600,600);
wait1Msec(800);
return;
}

void e7() {
	MG_mid();
wait1Msec(300);
return;
}

void e8() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(-60,-60);
wait1Msec(500);
return;
}

void e9(){
		MG_mid();
wait1Msec(300);
return;
}
void e10(){
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(-600,-600);
wait1Msec(800);
return;
}

void f1() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(-310,310);
wait1Msec(400);
return;
}

void f2() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(200,200);
wait1Msec(300);
return;
}

void f3() {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(-310,310);
wait1Msec(400);
return;
}

//grab the second blue mg
void f4() {
MG_down():
wait1Msec(400);
return;
}

void f5() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(600,600);
wait1Msec(800);
return;
}

void f6(){
MG_up():
wait1Msec(400);
return;
}

void f7() {
drive(620,-620);
wait1Msec(600);
return;
}

void f8(){
MG_mid();
wait1Msec(300);
return;
}

void f9() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(600,600);
wait1Msec(800);
return;
}

void f10(){
MG_down();
wait1Msec(300);
return;
}

void f10a(){
MG_mid();
wait1Msec(300);
return;
}

void g1(){
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(620,-620);
wait1Msec(600);
return;
}

void g2(){
MG_down();
wait1Msec(300);
return;
}

void g3(){
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(1500,1500);
wait1Msec(1800);
return;
}

void g4(){
MG_up();
wait1Msec(300);
return;
}

void g5() {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(600,600);
wait1Msec(800);
return;
}

void g6(){
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(-310,310);
wait1Msec(400);
return;
}

void g7 (){
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(600,600);
wait1Msec(800);
return;
}

void g8 (){
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(310,-310);
wait1Msec(400);
return;
}
void g9 (){
MG_mid();
wait1Msec(300);
return;
}

void g10(){
		SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive(800,800);
wait1Msec(1000);
return;
}

void g11() {
MG_down();
wait1Msec(300);
return;
}

//-----------------------------------------------------
//-----------------------------------------------------
task main()
{
	optimalL = SensorValue [ArmPotL];
	optimalR = SensorValue [ArmPotR];
	MoptimalL = SensorValue [MobilePotL];
	MoptimalR = SensorValue [MobilePotR];
	CoptimalL = SensorValue [ClawArmPotL];
	CoptimalR = SensorValue [ClawArmPotR];
	startTask(PID_RD4B);
	startTask(PID_Drive);
	startTask(PID_MG);
	startTask(PID_ClawLift);

}
