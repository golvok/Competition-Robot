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
FOR: "hebert_snipps.c"
AUTHOR: Hebert, with some of my (Matthew Walker ) code copied in?

Not sure why this is still here. Also I don't
remember exactly why he made this is the first place...
I seem to remember something to do with last minute
competition scrambling, Mr. Hebert's laptop, and him
staying up till 3am
*/

#include "controlCode.c"
#include "QEstraightLine.c"
#include "QEturning.c"
#include "armPID.c"
#include "grabAndLiftAfterTouch.c"



//get the value of AutoPot


void AutoCode ()
{
  while (true)
  {
    if (pgrmSelect() == 1)//AUTO 1 == Interaction Zone Red
      {
      //a square!
      goForwardCm(177);
      turn90(false);
      goForwardCm(177);
      turn90(false);
      goForwardCm(177);
      turn90(false);
      goForwardCm(177);
      turn90(false);
      }
    else if (pgrmSelect() == 2)//AUTO 2 == Interaction Zone Blue
    {
    goForwardCm(500);
    goBackwardCm(500);
    }
    else if (pgrmSelect() == 3)//AUTO 3 == Isolation Zone Red 1
    {}
    else if (pgrmSelect() == 4)//Auto 4 == Isolation Zone Red 2
    {}
    else if (pgrmSelect() == 5)//Auto 5 == Isolation Zone Blue 1
    {
	    //Move forward 15cm
	    goForwardCm(15);

      /*
      Drop (high)
	    Re adjust to straight forward (press button when done)
			Forward until line
			turn right on line
			foreward 120 (non blocking)
			(pick up middle cylinder while moving)
			continue foreward 120
			Drop (low)
			Back up 10cm
			Turn 90o right
			Forward 30cm
			(pick up sphere)
			Drop (medium)
			Turn 60o left
			Go forward 20cm
			180o turn
			Forward 20cm
			Drop (medium)
    */
    }
    else if (pgrmSelect() == 6)//Auto 6 == Isolation Zone Blue 2
    {}
    else if (pgrmSelect() == 7)//Auto 7 == Programming Skills
    {}
}
}








    /*
    for PROGRAMMING SKILLS -- RED ISOLATION ZONE
    position pointing at tower near start at 45 degree angle
    with enough room to lift claw and preloads. Put a barrel
    on a ball in the claw.

    if (vexRT[Btn7U]){
      grabObject();
      liftArmTo(highTowerArmAngleIndex);//doesn't block
      raiseTower(true);//blocks, takes longer
      goForwardCm(15);
      releaseObject();
      for(int i=0 ; i<3 ; i++){//3 times
        while(!getClawLimit()){}//wait until an object
        grabObject();
        releaseObject();
      }
    }
    /*
    for TESTING LINE SENSOR CODE

    if (vexRT[Btn7D]){
      goForwardUntilLine(177*TICKS_CM);
      turnAtLine(true);
      goForwardCm(50);
    }
    /*

    if (true){//vexRT[Btn7R]){
      grabObject();
      liftArmTo(highTowerArmAngleIndex);//doesn't block
      raiseTower(true);//blocks, takes longer
      goForwardCm(15);
      releaseObject();
      goBackwardCm(10);
      while (!getStartButton()){}//press button when done realigning
      goForwardUntilLine(25);
      turnAtLine(false);
      while(!getClawLimit()){
        QePidLoopContents(120,false);
      }
      grabObject();
      liftArmTo(lowTowerArmAngleIndex);
      goForwardCm(85);
      releaseObject();
    }
  }
}



//////////////////////////////////////////////////////////////////////////
/*int returnFwdSpeedLimit(){
  if (getFSONAR() < 20){
    return 0;
    }else if (getFSONAR() < 51){
    return pow((getFSONAR() - 20)/6,2)*5;//f(x) = 5(x/6)^2                 //Looks like this can be used for QE Distance Control
  }
  return 127;
*/
