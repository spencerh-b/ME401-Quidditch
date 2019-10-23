#include <SoftPWMServo.h>
#include "ME401_Claw.h"

SoftServo rightwheel, leftwheel;

#define MY_ROBOT_ID 8

// Pins
#define RIGHTWHEELPIN 32
#define LEFTWHEELPIN 31

// Table Sizes
#define X_MIN -250
#define X_MAX 2450

#define Y_MIN -250
#define Y_MAX 2450

// Robot Variables
const int BASE_SPEED = 90;
int leftBaseSpeed = BASE_SPEED;
int rightBaseSpeed = -BASE_SPEED;

const int maxAngleForward = 200;

int init_x_pose = -1000;
int homeLine = init_x_pose; // a variable signifying the x position of our homeline
RobotPose myRobot;

bool isYellowBall = false;

// Function Headers
float distanceFromRobot(int ballIndex);
int findNearestBall();
void moveForward(int ballNumber);
bool returnHome();
void rotateToBall(int ballNumber);
int toBallOrientation(int ballNumber);
bool orientedTo(int ballNumber);
bool homeSide(int ballIndex);
bool isYellow(int ballIndex);
bool atBall(int ballIndex);
void stopRotation();
bool isHome();
void determineSide(int setpoint);
void escape();


//Input of -100 to 100 for easiest backward v Forward
void leftWheelWrite(int leftSpeed)
{
  if(leftSpeed >= -BASE_SPEED && leftSpeed <= BASE_SPEED){
    leftwheel.write(map(leftSpeed,-100,100,1300,1700));
  }
}

void rightWheelWrite(int rightSpeed)
{
  if(rightSpeed >= -BASE_SPEED && rightSpeed <= BASE_SPEED){
    rightwheel.write(map(rightSpeed,-100,100,1300,1700));
  }
}

void driveSetup()
{
  Serial.println("DRIVE SETUP");
  rightwheel.attach(RIGHTWHEELPIN);
  leftwheel.attach(LEFTWHEELPIN);

  if(init_x_pose < (X_MIN-X_MAX)/2){
    init_x_pose = myRobot.x;
  }
}

bool movingState()
{
  myRobot = getRobotPose(MY_ROBOT_ID);
  if(!myRobot.valid){
    stopRotation();
  }
  //Find the nearest ball
  int nearestBall = findNearestBall();

  // If there exists a "nearest ball"
  if(clawState()){
    return true;
  }
  else if(orientedTo(nearestBall)){
    Serial.println("Move Forward");
    moveForward(nearestBall);
  }
  else{
    Serial.println("Rotate To Ball");
    rotateToBall(nearestBall);
  }
  return false;
}

bool returnHome()
{
  int nearestBall = -2;
  myRobot = getRobotPose(MY_ROBOT_ID);
  if(!isHome()){
    if(orientedTo(nearestBall)){
      Serial.println("Move Forward");
      moveForward(nearestBall);
    }
    else{
      Serial.println("Rotate To Ball");
      rotateToBall(nearestBall);
    }
    return false;
  }
  else{
    openClaw();
    escape();
    return true;
  }
}

bool isHome()
{
  Serial.print("Home: ");
  Serial.print(homeLine);
  Serial.print("  Cur: ");
  Serial.println(myRobot.x);
  if(homeLine > (X_MAX-X_MIN)/2){
    return myRobot.x >= homeLine ? true : false;
  }
  else{
    return myRobot.x <= homeLine ? true : false;
  }
}

void escape()
{
  leftWheelWrite(-BASE_SPEED);
  rightWheelWrite(BASE_SPEED);
  delay(500);
}

void determineSide(int team, int PIDangle)
{
  int robotTheta = myRobot.theta;
  int sensorTheta = PIDangle*.1333333*(PI/360.0)*1000;

  if(abs(robotTheta - sensorTheta) < (PI/2*1000)){
    if(team == 0){
      homeLine = X_MIN + (X_MAX-X_MIN) / 6 * 5;
    }
    else{
      homeLine = X_MIN + (X_MAX-X_MIN) / 6;
    }
  }
  else{
    if(team == 0){
      homeLine = X_MIN + (X_MAX-X_MIN) / 6;
    }
    else{
      homeLine = X_MIN + (X_MAX-X_MIN) / 6 * 5;
    }
  }
}

void stopRotation()
{
  rightWheelWrite(0);
  leftWheelWrite(0);
}

void rotateToFind(){
  rightWheelWrite(BASE_SPEED);
  leftWheelWrite(BASE_SPEED);
  delay(1000);
  stopRotation();
}

void rotateToBall(int ballNum)
{
  //Find necessary rotation
  double phi = toBallOrientation(ballNum);
  
  //Maninpulate rotation speed based upon size of angle
  int rotateSpeed = map(phi,-PI*1000,PI*1000,-50,50);

  rotateSpeed = abs(rotateSpeed) < 10 ? rotateSpeed / abs(rotateSpeed) * 10 : rotateSpeed;
 
  leftWheelWrite(-rotateSpeed);
  rightWheelWrite(-rotateSpeed);
  if(rotateSpeed < 0){
    Serial.print(" Turning Right: ");
    Serial.println(rotateSpeed);
  }
  else{
    Serial.print(" Turning Left:   ");
    Serial.println(rotateSpeed);
  }
}

void moveForward(int ballNumber)
{
  leftBaseSpeed = BASE_SPEED;
  rightBaseSpeed = -BASE_SPEED;

  //Get the angle away from the ball
  int phi = toBallOrientation(ballNumber);

  // Manipulate speed to correct little angle offsets
  float speedAdjust = map(phi,-maxAngleForward,maxAngleForward,-20,20);

  if(speedAdjust < 0){
    rightBaseSpeed -= speedAdjust;
  }
  else{
    leftBaseSpeed -= speedAdjust;
  }

  Serial.print("Left: ");
  Serial.print(leftBaseSpeed);
  Serial.print("  Right: ");
  Serial.println(rightBaseSpeed);

  leftWheelWrite(leftBaseSpeed);
  rightWheelWrite(rightBaseSpeed);
}

//Find the necessary rotation from robot to ball
int toBallOrientation(int ballNumber)
{
  float XVrbo;
  float YVrbo;
  if(ballNumber == -2){
    XVrbo = (float)homeLine - myRobot.x;
    YVrbo = 0.0;
  }
  else
  {
    XVrbo = (float)(ballPositions[ballNumber].x - myRobot.x);
    YVrbo = (float)(ballPositions[ballNumber].y - myRobot.y);
  }

  float robotTheta = myRobot.theta;
  float Xrbo = cos(robotTheta/1000.0)*XVrbo + sin(robotTheta/1000.0)*YVrbo;
  float Yrbo = -sin(robotTheta/1000.0)*XVrbo + cos(robotTheta/1000.0)*YVrbo;
  return atan2( Yrbo, Xrbo) * 1000;
}

//Is the robot orriented to the particular ball
bool orientedTo(int ballNumber)
{
  int thetaRad = toBallOrientation(ballNumber);
  Serial.print("Necessary Orientation: ");
  Serial.println(thetaRad);
  return (thetaRad < maxAngleForward && thetaRad > -maxAngleForward)? true : false;
}

//Returns index of the nearest ball
int findNearestBall()
{
  // Onlt do this if the position of the robot is not seen
  if(!myRobot.valid){
    return -1;
  }
  
  float minDist = -1.0;
  int minDistIndex = -1;
  Serial.print("Num Balls: ");
  Serial.println(numBalls);
  //For every ball see how close the ball is to the robot
  for(int ballIndex = 0; ballIndex < numBalls; ballIndex++){
    float tempDistance = distanceFromRobot(ballIndex);
    if(isYellow(ballIndex)){
      isYellowBall = true;
      return ballIndex;
    }
    if (!homeSide(ballIndex) && (minDistIndex == -1 || tempDistance < minDist)){
      minDist = tempDistance;
      minDistIndex = ballIndex;
    }
  }
  
  Serial.print("Nearest Ball: ");
  Serial.println(minDistIndex);
  return minDistIndex;
}

// Is the yellow ball on the court
bool isYellow(int ballIndex)
{
  return ballPositions[ballIndex].hue < 33 ? true : false;
}

// Is the ball on the same side in which you started
bool homeSide(int ballIndex)
{
  int x_ballPose = ballPositions[ballIndex].x;
  Serial.print("Home Line: ");
  Serial.print(homeLine);
  Serial.print(" Ball Pose: ");
  Serial.println(x_ballPose);
  if((homeLine < (X_MAX - X_MIN)/2) && (x_ballPose < (X_MAX - X_MIN)/3)){
    return true;
  }
  else if((homeLine > (X_MAX - X_MIN)/2) && (x_ballPose > (X_MAX - X_MIN) / 3 * 2)){
    return true;
  }
  return false;
}

// Returns distance to x and y coordinates
float distanceFromRobot(int ballIndex)
{
  if(myRobot.valid)
    return sqrt(pow(ballPositions[ballIndex].x-myRobot.x,2)+pow(ballPositions[ballIndex].y - myRobot.y,2));
}
