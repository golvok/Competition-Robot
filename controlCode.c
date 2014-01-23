//using:

/*
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
*/

/**  *****FILE COMMENT*****
FOR: "controlCode.c"
AUTHORS: Matthew Walker  & Mark

A collection of the lowest level control and access code.
Included in most files for its many convenience methods,
and is the lowest level of abstraction
*/


bool B7UReleased = true;
bool B7DReleased = true;

bool B7RReleased = true;
bool B7LReleased = true;
bool B8UReleased = true;
bool B8DReleased = true;
bool B8RReleased = true;
bool B8LReleased = true;

//a whole bunch of constants\\

const int CLAW_CLOSED = 1;
const int CLAW_OPEN = 0;
/**
  ticks on a quad per centimeter traveled -- CHANGE ME IF YOU USE THIS CODE ON ANOTHER ROBOT
*/
const float TICKS_CM = 8.3243; //for old chassis -> 10.452;

const int WHITE_FLOOR = 300;

//fake enums
int tower = 0;
const int UP = 1;
const int DOWN = -1;
const int STOP = 0;

bool BTN6UReleased = true;

const int TOWER_QE_MAX = 820;//what it stops at while coming up
const int TOWER_QE_MIN = 10;//what it stops at while going down

const bool RIGHT = true;
const bool LEFT = false;

const bool BACKWARD = true;
const bool FORWARD = false;

const bool DO_STOP = true;
const bool DONT_STOP = false;

/*
if const worked for the purposes that these are used for
(as references of constant value), they would be const. */
bool FARCE = false;
bool TRUTH = true;

//for accessing the arrays of constants
const int KP = 0;
const int KI = 1;
const int KD = 2;

const float DRIVE_MOTOR_SPEED_DIVISOR = 1;

//used to store the persistent errors. sometimes.
typedef struct {
  int oldError;
  int integralError;
} OIerrors;

/*
this global and its accompaning task are used in
a couple places to stop things like PID movement
that would otherwise block until completion. What
happens when it runs into something and you want 
it to stop trying? The task monitors the buttons
and updates the global at a useful interval.
*/
bool stopNow = false;
task stopper(){
  while (true){
    stopNow = (bool)vexRT[Btn7U] && (bool)vexRT[Btn7D];
    wait1Msec(100);
  }
}

/*
wasn't really used
By detail it means slower for more accuracy
see setLeft(int) & setRight(int)
*/
bool isDetailMode(){
  return vexRT[Btn5U] == 1;
}

/*
get the angle of the arm
*/
int getArmAngle(){
  return SensorValue[armAngle];
}

/*
get the value of the tower QE
*/
int getQEtower(){
  return SensorValue[QEtower];
}

/*
set the tower QE to 0
*/
void resetTowerQE(){
  SensorValue[QEtower] = 0;
}

int getFSONAR(){
  return 0;
}

/**
get the right Quad Encoder
*/
int getQEr(){
  return SensorValue[QEright];
}

/**
get the left Quad Encoder
*/
int getQEl(){
  return SensorValue[QEleft];
}

/**
adds the two wheel QE's together and divides by 2
*/
int getWheelQeAverage(){
  return (getQEl()+getQEr())/2;
}

/**
get the right LineFollower
*/
int getLFr(){
  return SensorValue[LFright];
}


/**
get the left LineFollower
*/
int getLFl(){
  return SensorValue[LFleft];
}

/**
get the front LineFollower
*/
int getLFrear(){
  return SensorValue[LFrear];
}

int getClaw(){
  return SensorValue[SOLclaw];
}

bool getClawLimit(){
  return (bool)SensorValue[clawLimitSensor];
}

bool getStartButton(){
  return (bool)SensorValue[startButton];
}

int getAutoPot(){
  return SensorValue[AutoPot];
}

//useful for autonomous
int pgrmSelect(){
  if (getAutoPot() < 200)  {
    return 1;
  }else if ((getAutoPot() >200) && (getAutoPot() <800))  {
    return 2;
  }else if ((getAutoPot() >800) && (getAutoPot() <2200))  {
    return 3;
  }else if ((getAutoPot() >3000) && (getAutoPot() <3700))  {
    return 4;
  }else if ((getAutoPot() >3900)/* && (getAutoPot() <400)*/)  {
    return 5;
  }
  /*else if ((getAutoPot() >200) && (getAutoPot() <400))  {
    return 6;
  }else if ((getAutoPot() >200) && (getAutoPot() <400))  {
    return 7;
  }*/
  return 0;
}

int returnFwdSpeedLimit(){
  if (getFSONAR() < 20){
    return 0;
    }else if (getFSONAR() < 51){
    return pow((getFSONAR() - 20)/6,2)*5;//f(x) = 5(x/6)^2       //Looks like this can be used for QE Distance Control
  }
  return 127;
}

void setClaw(int cStat){
  SensorValue[SOLclaw]=cStat;
}

void setRight(int rPwr){
#ifdef USE_SONAR_STOP_AND_DETAIL_MODE
  if (isDetailMode()){
    motor[rightFront] = rPwr / 3;
    motor[rightRear] = rPwr / 3;
    }else if (rPwr > returnFwdSpeedLimit()){
    motor[rightFront] = returnFwdSpeedLimit();
    motor[rightRear] = returnFwdSpeedLimit();
    }else{
    motor[rightFront] = rPwr;
    motor[rightRear] = rPwr;
  }
#else
  if (abs(rPwr) <= 127){
    motor[rightFront] = rPwr/DRIVE_MOTOR_SPEED_DIVISOR;
    motor[rightRear] = rPwr/DRIVE_MOTOR_SPEED_DIVISOR;
  }else{
    //multiply 127 by the sign of the given value without using a conditional
    motor[rightFront] = abs(rPwr)/rPwr*127/DRIVE_MOTOR_SPEED_DIVISOR;
    motor[rightRear] = abs(rPwr)/rPwr*127/DRIVE_MOTOR_SPEED_DIVISOR;
  }
#endif
}

void setLeft(int lPwr){
#ifdef USE_SONAR_STOP_AND_DETAIL_MODE
  if (isDetailMode()){
    motor[leftFront] = lPwr / 3;
    motor[leftRear] = lPwr / 3;
    }else if (lPwr > returnFwdSpeedLimit()){
    motor[leftFront] = returnFwdSpeedLimit();
    motor[leftRear] = returnFwdSpeedLimit();
    }else{
    motor[leftFront] = lPwr;
    motor[leftRear] =lPwr;
  }
#else
  if (abs(lPwr) <= 127){
    motor[leftFront] = lPwr/DRIVE_MOTOR_SPEED_DIVISOR;
    motor[leftRear] = lPwr/DRIVE_MOTOR_SPEED_DIVISOR;
  }else{
    //multiply 127 by the sign of the given value without using a conditional
    motor[leftFront] = abs(lPwr)/lPwr*127/DRIVE_MOTOR_SPEED_DIVISOR;
    motor[leftRear] = abs(lPwr)/lPwr*127/DRIVE_MOTOR_SPEED_DIVISOR;
  }
#endif
}

void setTower(int tPwr){
  motor[tower1] = tPwr;
  motor[tower2] = tPwr;
}

void setArm(int aPwr){
  //writeDebugStreamLine("set Arm to %i",aPwr);
  motor[arm1] = aPwr;
  motor[arm2] = aPwr;
}
/*
just sets the right and left drive motors to ch 2 & 3 respectively. nothing else.
meant te be part of a loop.
*/
void setDriveMotorsToRC(){
  //To adjust for the controller never giving a 0 value (yes I did calibrate it)
  setRight(vexRT[Ch2]-1);
  setLeft(vexRT[Ch3]-1);
}

/*
  if it was open, close it
  if it was closed, open it
*/
void toggleClaw(){
  /*if(getClaw()==1){
    setClaw(0);
    }else{
    setClaw(1);
  }*/
  setClaw((int)!(bool)getClaw());
}

/**
finish==true  blocks until done.
finish==false uses the moveTower() thing
*/
void lowerTower(bool finish){
  if (!finish){
    tower = DOWN;
  }else{
    tower = STOP;
    while (getQEtower()>TOWER_QE_MIN){
      setTower(-127);
    }
    setTower(0);
  }
}


/**
finish==true  blocks until done.
finish==false uses the moveTower() thing
*/
void raiseTower(bool finish){
  if (!finish){
    tower = UP;
  }else{
    tower = STOP;
    while (getQEtower()<TOWER_QE_MAX){
      setTower(127);
    }
    setTower(0);
  }
}

/*
sets the tower motors based on the state of the tower variable (see "fake enums"
near the top) and resets it to STOP when it has moved far enough.
meant te be part of a loop.
*/
void moveTower(){
  if(tower == STOP){
    setTower(0);
    return;
    }else if(tower == UP){
    if(getQEtower()<TOWER_QE_MAX){
      setTower(127);
      }else{
      setTower(0);
      tower = STOP;
    }
    }else{
    if(getQEtower()>TOWER_QE_MIN){
      setTower(-127);
      }else{
      setTower(0);
      tower = STOP;
    }
  }
}

/**
complete manual control if I ever did see it!
the only automated part is the tower but it still obeys the driver's every whim
meant te be part of a loop.
Controls:
  setDriveMotorsToRC() controls, plus:
  8U - set the tower going up. Will stop before too far
  8D - set the tower going down. Will stop before too far
  5U - set arm motors to up. sets to 0 on release
  5D - set arm motors to down. sets to 0 on release
  6U - toggle the state of the claw
*/
void setMotorsToRC(){
  setDriveMotorsToRC();
  if(vexRT[Btn8U]){
    tower = UP;
  }else if(vexRT[Btn8D]){
    tower = DOWN;
  }
  moveTower();
  if(vexRT[Btn5U]){
    setArm(127);
  }else if(vexRT[Btn5D]){
    setArm(-127);
  }else{
    setArm(0);
  }
  if(vexRT[Btn6U]){
    if(BTN6UReleased == true){
      toggleClaw();
      BTN6UReleased = false;
    }
  }else{
    BTN6UReleased = true;
  }
  //if ()
}

/**
favour left is negative, right is positive
*/
int getRawStraightLineError(){
  return getQEr() - getQEl();
}

/**
favour left is negative, right is positive
*/
int getRawTurnError(){
  return (getQEr() + getQEl());
}

/*
this never worked properly. don't use it.
*/
float calcLineSensorError(float side, float otherSide, float scaleUp){
  //return scaleUp * ((getLFf()-side) / getLff());
  return scaleUp * ((1800-(side-950)) / 1800);
}

/**
favour left is negative, right is positive
this never worked properly. don't use it.
*/
int getRawLineSensorError(){
  float scaleUp = 250;//error is scaled to this
  if (getLFr() < getLFl()){
    //favour right
    return calcLineSensorError(getLFr(),getLFl(),scaleUp);
    }else{
    //favour left
    return -calcLineSensorError(getLFl(),getLFr(),scaleUp);
  }
}

/*
favour down is positive, up negative
*/
int calcArmError(int targetAngle){
  return (targetAngle - getArmAngle());
}

/**
set both Quads to 0
*/
void resetWheelQEs(){
  SensorValue[QEright] = 0;
  SensorValue[QEleft] = 0;
}
/**
set the left Quad to 0
*/
void resetLeftWheelQE(){
  SensorValue[QEleft] = 0;
}
/**
set the right Quad to 0
*/
void resetRightWheelQE(){
  SensorValue[QEright] = 0;
}
