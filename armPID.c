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

/**  *****FILE COMMENT*****
FOR: "armPID.c"
AUTHOR: Matthew Walker 

The PID controller for the arm. Usefull and works well
ever since hebert suggested to decrease the delay at the 
end of the PID function. It is important to remember that
this controller tries its hardest to get the arm to where
it is set to be, and keep it there, even if the arm appears
to be resting at the downArmAngle, for example or it can't
quite physically get to the target angle. The arm is the
first place that you will notice that the batteries are
dying, so I assume that it takes alot of power.

*/

#ifndef MAIN_USED
  #define APID_IS_MAIN
  #define MAIN_USED
#endif

//#define USE_SONAR_STOP_AND_DETAIL_MODE

#include "controlCode.c"

float armKp = 0.35;//     P tweak
float armKi = 0.01;//     I tweak
float armKd = 0.08;//     D tweak

float armPwr = 0;
int armIntegralErr = 0;
//int change = 0;
int oldArmError = 0;
float armError = 0;
//const int motorBasePower = 64;

const int downArmAngleIndex = 0;
const int ballOnCylinderArmAngleIndex = 1;
const int lowTowerArmAngleIndex = 2;
const int mediumTowerArmAngleIndex = 3;
const int highTowerArmAngleIndex = 4;


const int numArmAngles = 5;

const int armAngles[numArmAngles] = {130,750,1500,2300,2300};
const int ARM_ANGLE_OFFSET = 00; //this is added to the above angles when the angle is retrieved in getArmAngle()

int targetArmAngleIndex = downArmAngleIndex;


int getArmAngle(int index){
  if (-1 < index && index < (numArmAngles)){
    return armAngles[index]+ARM_ANGLE_OFFSET;
  }
  return 0;
}

bool isArmDown(){
  return getArmAngle()<(getArmAngle(downArmAngleIndex)+100);
}

/**
  if (successful)
*/
bool setTargetArmAngleIndex(int i){
  if (-1 < i && i < (numArmAngles)){
    targetArmAngleIndex = i;
    return true;
  }
  return false;
}

void resetTargetArmAngleIndex(){
  targetArmAngleIndex = 0;
}

void setTargetAngleToButtons(){
  if (vexRT[Btn6D] && vexRT[Btn5D]){
    targetArmAngleIndex = ballOnCylinderArmAngleIndex;
  }else if (vexRT[Btn5D]){
    targetArmAngleIndex = lowTowerArmAngleIndex;
  }else if (vexRT[Btn5U]){
    targetArmAngleIndex = mediumTowerArmAngleIndex;
  }else if (vexRT[Btn6D]){
    targetArmAngleIndex = highTowerArmAngleIndex;
  }else if (vexRT[Btn7R]){
    targetArmAngleIndex = downArmAngleIndex;
  }
}

void armPidLoopContents(){
  armError = calcArmError(getArmAngle(targetArmAngleIndex));
  if (abs(armError) > 50){//acceptable error

    if (armError < 0){
      armError /= 4;
    }

    armIntegralErr += armError;

    if (armIntegralErr > 200){
      armIntegralErr = 200;
    }else if (armIntegralErr < -200){
      armIntegralErr = -200;
    }
    armPwr = armKp*armError + armKi*armIntegralErr + armKd*(oldArmError - armError);

    oldArmError = armError;
  }
  /*while (vexRT[Btn5U] != 0){//hold 5U to pause
    setArm(0);
  }*/
  //writeDebugStreamLine(" setting arm");
  setArm(armPwr);
  wait1Msec(5);//this used to be higher but the arm would start spazzing near the top and take a while to settle.
}

task armPidTask(){
  while (true){
    armPidLoopContents();
  }
}

/**
  stops the arm PID task if it is already running
  starts the arm PID task
*/
void startArmPid(){
  StopTask(armPidTask);
  StartTask(armPidTask);
}

void stopArmPid(){
  StopTask(armPidTask);
}

/**
  sets the angleindex, and starts the arm PID task
*/
void liftArmTo(int angleIndex){
  if (setTargetArmAngleIndex(angleIndex)){
    startArmPid();
  }
}



#ifdef APID_IS_MAIN
task main(){
#else
void armPidMain(){
#endif

  while(vexRT[Btn5D] != 1){//press Btn5D and exit button to end

    //++++++++++++++ == RC/adjusting part == ++++++++++++++

    //writeDebugStreamLine("NORMAL!");
    while(vexRT[Btn6U] != 1){//press Btn6U to get out

      //float KPchangeSample = armKp*calcArmError();
      float armErrorSample = calcArmError(armAngles[targetArmAngleIndex]);

      if (vexRT[Btn5D] == 0 && vexRT[Btn6D] == 0){
        setMotorsToRC();
        setTargetAngleToButtons();
        }else{

        if (vexRT[Btn7U] != 0 && B7UReleased){//if down
          armKp += 0.01;
          B7UReleased = false;
          } else if (vexRT[Btn7U] != 1){
          B7UReleased = true;
        }
        if (vexRT[Btn7D] != 0 && B7DReleased){//if down
          armKp -= 0.01;
          B7DReleased = false;
          } else if (vexRT[Btn7D] != 1){
          B7DReleased = true;
        }

        if (vexRT[Btn7R] != 0 && B7RReleased){//if down
          armKd += 0.01;
          B7RReleased = false;
          } else if (vexRT[Btn7R] != 1){
          B7RReleased = true;
        }
        if (vexRT[Btn7L] != 0 && B7LReleased){//if down
          armKd -= 0.01;
          B7LReleased = false;
          } else if (vexRT[Btn7L] != 1){
          B7LReleased = true;
        }
        if (vexRT[Btn8U] != 0 && B8UReleased){//if down
          armKi += 0.01;
          B8UReleased = false;
          } else if (vexRT[Btn8U] != 1){
          B8UReleased = true;
        }
        if (vexRT[Btn8D] != 0 && B8DReleased){//if down
          armKi -= 0.01;
          B8DReleased = false;
          } else if (vexRT[Btn8D] != 1){
          B8DReleased = true;
        }
      }
    }

    //++++++++++++++ == PID part == ++++++++++++++

    //("PID!");
    while(vexRT[Btn6D] != 1){//press Btn6D to get out
      //setMotorsToRC(); NO. The manual arm controls will interfere
      setTargetAngleToButtons();
      armPidLoopContents();
    }
  }
}
