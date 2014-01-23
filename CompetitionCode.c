#pragma config(Sensor, in1,    AutoPot,         sensorPotentiometer)
#pragma config(Sensor, in4,    LFright,             sensorLineFollower)
#pragma config(Sensor, in5,    LFrear,              sensorLineFollower)
#pragma config(Sensor, in6,    LFleft,              sensorLineFollower)
#pragma config(Sensor, in7,    armAngle,            sensorPotentiometer)
#pragma config(Sensor, dgtl1,  QEtower,             sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  SOLclaw,             sensorDigitalOut)
#pragma config(Sensor, dgtl4,  QEleft,              sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  QEright,             sensorQuadEncoder)
#pragma config(Sensor, dgtl10, clawLimitSensor,     sensorTouch)
#pragma config(Sensor, dgtl11, startButton,         sensorTouch)
#pragma config(Motor,  port1,           leftRear,      tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port3,           arm1,          tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port4,           arm2,          tmotorNormal, openLoop)
#pragma config(Motor,  port5,           tower1,        tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port6,           tower2,        tmotorNormal, openLoop)
#pragma config(Motor,  port8,           rightFront,    tmotorNormal, openLoop)
#pragma config(Motor,  port9,           rightRear,     tmotorNormal, openLoop)
#pragma config(Motor,  port10,          leftFront,     tmotorNormal, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#ifndef MAIN_USED
  #define CODE2011_IS_MAIN
  #define MAIN_USED
#endif

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!
#include "controlCode.c"
#include "autonomous.c"
#include "examRC.c"

void pre_auton(){

}

task autonomous(){
  autonomousCode();//in autonomous.c
}

task usercontrol(){
  /*while (true){
    setDriveMotors();
    grabAndLiftAfterTouchLoopContents(false);
  }*/
  examRCmain();
}