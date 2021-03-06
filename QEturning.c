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
FOR: "QEturning.c"
AUTHOR: Matthew Walker 

  A PID controler for turning/spinning. Uses data from left and right QEs.
  
*/

#ifndef MAIN_USED
  #define QET_IS_MAIN
  #define MAIN_USED
#endif

#include "controlCode.c"
#include "QEstop.c"

float ONE_DEG = 3.3;//old chasssis -> 3.7;
float qetPidVars[] = {1.25,0.01,0.10};//old chassis -> {1.25,0.01,0.10};
//                     P tweak
//                          I tweak
//                               D tweak

//float QEtTurnErrorWeight = 0.50; //the weight of the turn error
                                 //in the turning pid loop contents

bool turnDirection = 0;

/**
true -> right
false -> left
*/
void setTurnDir(bool dir){
  turnDirection = dir;
}

/**
right -> 1
left -> -1
*/
int getTurnDirAsInt(){
  return (((int)turnDirection)*2 - 1);//woooo!!
}


/**
  will spin on the spot
*/
void QeSpinPidLoopContents(){
  static int integralErr = 0;
  static int oldError = 0;

  float RightPwr = 0;
  float leftPwr = 0;
  int change = 0;
  int error = 0;

  wait1Msec(50);

  error = getRawTurnError();
  integralErr += error;
  if (integralErr > 64){
    integralErr = 64;
    }else if (integralErr < -64){
    integralErr = -64;
  }
  change = getTurnDirAsInt()*(qetPidVars[KP]*error + qetPidVars[KI]*integralErr + qetPidVars[KD]*(oldError - error));

  RightPwr = -(change+96)*getTurnDirAsInt();
  leftPwr  = (-change+96)*getTurnDirAsInt();

  oldError = error;
  /*while (vexRT[Btn5U] != 0){//hold 5U to pause
  setRight(0);
  setLeft(0);
  }*/
  setRight(RightPwr);
  setLeft(leftPwr);
}

/**
  more useful for turning.
  actually it isn't. this code needs its own P,I and D constants
*/
void QeTurnPidLoopContents(int length){
  static int integralErrR = 0;
  static int integralErrL = 0;
  static int oldErrorR = 0;
  static int oldErrorL = 0;

  float RightPwr = 0;
  float leftPwr = 0;
  int changeR = 0;
  int changeL = 0;
  int errorR = 0;
  int errorL = 0;

  wait1Msec(50);

  errorR = (getQER()+length) + getRawTurnError()*QEtTurnErrorWeight;
  errorL = (getQEl()-length) + getRawTurnError()*QEtTurnErrorWeight;
  integralErrR += errorR;
  integralErrL += errorL;
  if (integralErrR > 64){
    integralErrR = 64;
    }else if (integralErrR < -64){
    integralErrR = -64;
  }
  if (integralErrL > 64){
    integralErrL = 64;
    }else if (integralErrL < -64){
    integralErrL = -64;
  }
  changeR = getTurnDirAsInt()*(qetPidVars[KP]*errorR + qetPidVars[KI]*integralErrR + qetPidVars[KD]*(oldErrorR - errorR));
  changeL = getTurnDirAsInt()*(qetPidVars[KP]*errorL + qetPidVars[KI]*integralErrL + qetPidVars[KD]*(oldErrorL - errorL));

  RightPwr = -(changeR+96)*getTurnDirAsInt();
  leftPwr  = (-changeL+96)*getTurnDirAsInt();

  oldErrorR = errorR;
  oldErrorL = errorL;
  /*while (vexRT[Btn5U] != 0){//hold 5U to pause
  setRight(0);
  setLeft(0);
  }*/
  setRight(RightPwr);
  setLeft(leftPwr);
}

/**
resets quads and turns by given degrees
true for right false for left
blocks until finished or forceStop == true
*/
void turnDegrees(int degrees, bool dir, bool stopThere, bool* forceStop){
  int length = ONE_DEG*degrees;
  resetWheelQEs();
  setTurnDir(dir);
  //time1[T1] = 0;
  while ((( abs(getQEl()) + abs(getQEr()) )/2 < length) && !forceStop){
    QeSpinPidLoopContents();
  }
  if (stopThere){
    stopBothWheels();
    wait1Msec(WAIT_AFTER_STOP_TIME);
    releaseBothWheels();
  }
}

void turnDegrees(int degrees, bool dir, bool stopThere){
  turnDegrees(degrees,dir,stopThere,FARCE);
}

/**
resets quads and turns by 90 degrees
true for right false for left
blocks until finished or forceStop == true
*/
void turn90(bool dir,bool stopThere,bool* forceStop){
  turnDegrees(90,dir,stopThere,forceStop);
}

void turn90(bool dir,bool stopThere){
  turnDegrees(90,dir,stopThere,FARCE);
}
#ifdef QET_IS_MAIN
task main(){
#else
void QEturningMain(){
#endif

  while(vexRT[Btn5D] != 1){//press Btn5D and exit button to end
    resetWheelQEs();

    // ++++++++++++++ == RC/adjusting part == ++++++++++++++

    bool B7UReleased = true;
    bool B7DReleased = true;
    bool B7RReleased = true;
    bool B7LReleased = true;
    bool B8UReleased = true;
    bool B8DReleased = true;

    while(vexRT[Btn6U] != 1){//press Btn6U to get out

      if (vexRT[Btn5D] == 0 && vexRT[Btn6D] == 0){
        setMotorsToRC();
        static int errorSample = getRawTurnError();
        }else{
        if (vexRT[Btn7U] != 0 && B7UReleased){//if down
          qetPidVars[KP] += 0.01;
          //writeDebugStreamLine("qetPidVars[KP] = %f", qetPidVars[KP]);
          B7UReleased = false;
          } else if (vexRT[Btn7U] != 1){
          B7UReleased = true;
        }
        if (vexRT[Btn7D] != 0 && B7DReleased){//if down
          qetPidVars[KP] -= 0.01;
          //writeDebugStreamLine("qetPidVars[KP] = %f", qetPidVars[KP]);
          B7DReleased = false;
          } else if (vexRT[Btn7D] != 1){
          B7DReleased = true;
        }

        if (vexRT[Btn7R] != 0 && B7RReleased){//if down
          qetPidVars[KD] += 0.01;
          //writeDebugStreamLine("qetPidVars[KD] = %f", qetPidVars[KD]);
          B7RReleased = false;
          } else if (vexRT[Btn7R] != 1){
          B7RReleased = true;
        }
        if (vexRT[Btn7L] != 0 && B7LReleased){//if down
          qetPidVars[KD] -= 0.01;
          //writeDebugStreamLine("qetPidVars[KD] = %f", qetPidVars[KD]);
          B7LReleased = false;
          } else if (vexRT[Btn7L] != 1){
          B7LReleased = true;
        }
        if (vexRT[Btn8U] != 0 && B8UReleased){//if down
          qetPidVars[KI] += 0.01;
          //writeDebugStreamLine("qetPidVars[KI] = %f", qetPidVars[KI]);
          B8UReleased = false;
          } else if (vexRT[Btn8U] != 1){
          B8UReleased = true;
        }
        if (vexRT[Btn8D] != 0 && B8DReleased){//if down
          qetPidVars[KI] -= 0.01;
          //writeDebugStreamLine("qetPidVars[KI] = %f", qetPidVars[KI]);
          B8DReleased = false;
          } else if (vexRT[Btn8D] != 1){
          B8DReleased = true;
        }
        if (vexRT[Btn8R] != 0 && B8UReleased){//if down
          qetPidVars[KI] += 0.01;
          //writeDebugStreamLine("qetPidVars[KI] = %f", qetPidVars[KI]);
          B8UReleased = false;
          } else if (vexRT[Btn8U] != 1){
          B8UReleased = true;
        }
        if (vexRT[Btn8L] != 0 && B8DReleased){//if down
          qetPidVars[KI] -= 0.01;
          //writeDebugStreamLine("qetPidVars[KI] = %f", qetPidVars[KI]);
          B8DReleased = false;
          } else if (vexRT[Btn8D] != 1){
          B8DReleased = true;
        }
      }
    }

    //++++++++++++++ == PID part == ++++++++++++++

    resetWheelQEs();
    while(vexRT[Btn6D] != 1){//press Btn6D to get out
      //setMotorsToRC();
      QeSpinPidLoopContents();
      if (vexRT[Btn8R]){
        turn90(RIGHT,true,stopNow);
      }else if (vexRT[Btn8L]){
        turn90(LEFT,true,stopNow);
      }
    }
  }
}
