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

/**  *****FILE COMMENT*****
FOR: "grabAndLiftAfterTouch.c"
AUTHOR: Matthew Walker 

just supposed to grab an object when somethine depresses
the claw limit sensor(or the driver presses the force bechaviors button),
and lift the arm and/or tower to the goal set by setTargetAngleToButtons()
in armPID.c, release the claw on the next F.B. press, and lower the arm on the next,
but turned into a mess, because of a naïeve first implementation.
*/


#ifndef MAIN_USED
  #define GALAT_IS_MAIN
  #define MAIN_USED
#endif

#include "controlCode.c"
#include "armPID.c"
#include "QEstraightLine.c"

const int MOVE_BACK_CM = 15;

bool hasObject = false;

bool clawLimitSensorReleased = true;
bool forceBehaviours = false;
bool behaviorForcingReleased = true;


void grabObject(){
  setClaw(CLAW_CLOSED);
  hasObject = true;
  //writeDebugStreamLine("grabbed object");
}

void releaseObject(){
  setClaw(CLAW_OPEN);
  hasObject = false;
  //writeDebugStreamLine("released object");
}


void setForceBehaviours(bool forceVal){
  forceBehaviours = forceVal;
}

/**
  General reset of the GALAT system to be used if something goes wrong
  Very useful for when you have dropped something. Also useful to save
  some battery charge, as it causes the arm to go limp, instead of PID'ing
  it down. Also, sometimes, the arm goes down, but not quite, and is held
  in place by armPID. Finally, armPID does't just prevent the arm from falling,
  it holds it in place, which wastes some charge too.
*/
void resetGrabAndLift(){
  behaviorForcingReleased = true;
  forceBehaviours = false;
  setTargetArmAngleIndex(downArmAngleIndex);
  stopArmPid();
  releaseObject();
  lowerTower(false);
  setArm(-50);// give the arm a nudge in the downwards direction
  wait1Msec(50);
  setArm(0);
}

/**
I don't expect you to, or expect you to want to, understand this method.
I, myself, even though I wrote it, don't get it anymore, and admit that
it is terrible code. Because it is designed to be called once per iteration
of a loop, it **tries** to cover every possible configuration that the bot
could be in while grabbing and lifting without having very much information
about its previous state. As a result there are very many complex conditions
that seem to make no sense. Oh yeah, and while it is doing all this, it is
often waiting until the user pushes the forceBehaviors button after
repositioning the robot(eg. over a goal after lifting).

This should be reimplemented as a task with a few
while (!forceBehaviors){wait1Msec(100)}
to wait for user input

There is some sort of bug that happens whan the clawLimitSensor ins't depressed
(or is broken off...) while lifting, that causes the claw to release.

These three parameters are afterthoughts that I addded so I might be able to
use this code in automomous scripting, but I never did and the parameters aren't
100% useful. This was also when I looked it over and realized that it should
be rewritten, after realizing that tasks are a much better wap to do many things.

autonomousForce - if false, will force behaviours with Btn6U
autonomousChoice - if false, will choose heights with setTargetAngleToButtons()
autonomousMoveAway - if true, will move backward and forward as needed regardless of user imput
*/
void grabAndLiftAfterTouchLoopContents(bool autonomousForce, bool autonomousChoice, bool autonomousMoveAway){
    moveTower();
    if (vexRT[Btn5D] && vexRT[Btn5U] && vexRT[Btn6D]){//all three angle buttons
      resetGrabAndLift();
    }
    if (!autonomousForce){
      setForceBehaviours((bool)vexRT[Btn6U]);
    }
    if((!isArmDown() && forceBehaviours)){
      if (!autonomousChoice){
        setTargetAngleToButtons();//in armPID.c
      }
      if (targetArmAngleIndex == highTowerArmAngleIndex){// the special case where the high tower is selected
        raiseTower(false);//blocks
        //writeDebugStreamLine("high tower selected1");
      }
      //writeDebugStreamLine("selected tower index %i = %i",targetArmAngleIndex,armAngles[targetArmAngleIndex]);
      if (targetArmAngleIndex != downArmAngleIndex){
        startArmPid();//starts arm PID in another task
      }
    }else if (!hasObject && ((SensorValue[clawLimitSensor] && clawLimitSensorReleased) || (isArmDown() && forceBehaviours))){
      if (SensorValue[clawLimitSensor] == 1){
        clawLimitSensorReleased = false;
      }
      grabObject();
      if (!autonomousChoice){
        setTargetAngleToButtons();//in armPID.c
      }
      if (autonomousMoveAway || !(vexRT[Btn8D])){
        wait1Msec(250);
        goBackwardCm(MOVE_BACK_CM,false);
      }
      //writeDebugStreamLine("selected tower index %i = %i",targetArmAngleIndex,armAngles[targetArmAngleIndex]);
      if (targetArmAngleIndex == highTowerArmAngleIndex){
        raiseTower(false);
        //writeDebugStreamLine("high tower selected2");
      }
      if (targetArmAngleIndex != downArmAngleIndex){
        startArmPid();//starts arm PID in another task
      }
    }else if (SensorValue[clawLimitSensor] == 0){
      clawLimitSensorReleased = true;
    }


    if (behaviorForcingReleased){
      if (forceBehaviours == true){
        behaviorForcingReleased = false;
        if (hasObject){
          if (autonomousMoveAway || !(vexRT[Btn8D])){
	          //goForwardCm(MOVE_BACK_CM,true);
	        }
          releaseObject();
        }else{
          //writeDebugStreamLine("going down");
          if (autonomousMoveAway || !(vexRT[Btn8D])){
            goBackwardCm(MOVE_BACK_CM,false);
          }
          lowerTower(false);
          liftArmTo(downArmAngleIndex);
        }
      }
    }else if (forceBehaviours == false){
      behaviorForcingReleased = true;
    }
}

#ifdef GALAT_IS_MAIN
task main(){
#else
void grabAndLiftAfterTouchMain(){
#endif
  while (true){
    setDriveMotorsToRC();
    grabAndLiftAfterTouchLoopContents(false,false,false);
  }
}
