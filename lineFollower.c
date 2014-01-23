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
FOR: "lineFollower.c"
AUTHOR: Matthew Walker

  Almost was a PID controler. Never could get a good source of error from
  line sensors; perhaps you will have more luck. Wasted a few days on that
  with different weighting and calculation methods. I still believe that
  it is possible to write a controller from a minimum of at most three if
  not only two light sensors (line followers). Each sensor's value should
  be proportional to its distance from the line (to an extent), so that
  distance *should* be calcuable with some accuracy, especially if both
  sensors are right beside each other, of course only if the line is under
  one of them. This calculated distance would make an excellent error source
  for a PID controller.

  This...imposter...for a PID controller will follow a line sometimes, but not very well.
  
*/

#ifndef MAIN_USED
  #define LF_IS_MAIN
  #define MAIN_USED
#endif

#include "controlCode.c"
#include "QEstraightLine.c"

float lfPidVars[] = {1.25,0.05,0.4};
//                     P tweak (anything higher than this will work)
//                          I tweak
//                              D tweak


void LfPidLoopContents(int length){
  static float rightPwrLF = 0;
  static float leftPwrLF = 0;
  static int integralErr = 0;
  static int change = 0;
  static int oldError = 0;
  static int error = 0;
  /* error = getRawStraightLineError()*LFStraightErrorWeight;
  integralErr += error;
  if (integralErr > 200){
  integralErr = 200;
  }else if (integralErr < -200){
  integralErr = -200;
  }
  change = lfPidVars[Kp]*error + lfPidVars[Ki]*integralErr + lfPidVars[Kd]*(oldError - error);

  rightPwrLF  = (-change+110);
  leftPwrLF  = (+change+110);
  if ((length - getWheelLfAverage())<300){
  rightPwrLF *=abs(((float)getLFr() - (float)length)/((float)length*2));
  leftPwrLF *=abs(((float)getLFl() - (float)length)/((float)length*2));
  }
  oldError = error;
  /*while (vexRT[Btn5U] != 0){//hold 5U to pause
  setRight(0);
  setLeft(0);
  }*/



  //when it is to the left of the line turn it right and vice versa

  if(getLFr()<2000){// && getLFr()>2000){
    rightPwrLF = 127;
    leftPwrLF = 95;
  }else if(getLFl()<2000){// && getLFr()>2000){
    rightPwrLF = 127;
    leftPwrLF = 95;
  /*}else if(getLFl()>2000 && getLFr()>2000){
    //do nothing; keep the same power values
  */
  }else{
    rightPwrLF = 127;
    leftPwrLF = 127;
  }
  setRight(rightPwrLF);
  setLeft(leftPwrLF);
  wait1Msec(50);
}

void goForward(int length){
  resetWheelQEs();
  while (getWheelQeAverage()<length){
    LfPidLoopContents(length);
  }
}

void goForwardUntilLimit(){
  resetWheelQEs();
  while (SensorValue[clawLimitSensor]!=1){
    int length = 200000;
    LfPidLoopContents(length);
  }
}

#ifdef LF_IS_MAIN
task main(){
#else
void LFstraightMain(){
#endif

  while(vexRT[Btn5D] != 1){//press Btn5D and exit button to end

    //++++++++++++++ == RC/adjusting part == ++++++++++++++

    bool B7UReleased = true;
    bool B7DReleased = true;
    bool B7RReleased = true;
    bool B7LReleased = true;
    bool B8UReleased = true;
    bool B8DReleased = true;

    while(vexRT[Btn6U] != 1){//press Btn6U to get out

      if (vexRT[Btn5D] == 0 && vexRT[Btn6D] == 0){
        setMotorsToRC();
        int errorSample = getRawStraightLineError();
        }else{
        if (vexRT[Btn7U] != 0 && B7UReleased){//if down
          LfKp += 0.01;
          //writeDebugStreamLine("LfKp = %f", LfKp);
          B7UReleased = false;
          } else if (vexRT[Btn7U] != 1){
          B7UReleased = true;
        }
        if (vexRT[Btn7D] != 0 && B7DReleased){//if down
          LfKp -= 0.01;
          //writeDebugStreamLine("LfKp = %f", LfKp);
          B7DReleased = false;
          } else if (vexRT[Btn7D] != 1){
          B7DReleased = true;
        }

        if (vexRT[Btn7R] != 0 && B7RReleased){//if down
          LfKd += 0.01;
          //writeDebugStreamLine("LfKd = %f", LfKd);
          B7RReleased = false;
          } else if (vexRT[Btn7R] != 1){
          B7RReleased = true;
        }
        if (vexRT[Btn7L] != 0 && B7LReleased){//if down
          LfKd -= 0.01;
          writeDebugStreamLine("LfKd = %f", LfKd);
          B7LReleased = false;
          } else if (vexRT[Btn7L] != 1){
          B7LReleased = true;
        }
        if (vexRT[Btn8U] != 0 && B8UReleased){//if down
          LfKi += 0.01;
          //writeDebugStreamLine("LfKi = %f", LfKi);
          B8UReleased = false;
          } else if (vexRT[Btn8U] != 1){
          B8UReleased = true;
        }
        if (vexRT[Btn8D] != 0 && B8DReleased){//if down
          LfKi -= 0.01;
          //writeDebugStreamLine("LfKi = %f", LfKi);
          B8DReleased = false;
          } else if (vexRT[Btn8D] != 1){
          B8DReleased = true;
        }
      }
    }

    //++++++++++++++ == PID part == ++++++++++++++

    resetWheelQEs();
    while(vexRT[Btn6D] != 1){//press Btn6D to get out
      LfPidLoopContents(2000);
    }
  }
}
