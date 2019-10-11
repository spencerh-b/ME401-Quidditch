#include <SoftPWMServo.h>
#include "ME_401_Claw.h"

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
const int BASE_SPEED = 100;
int leftBaseSpeed = BASE_SPEED;
int rightBaseSpeed = -BASE_SPEED;

const int maxAngleForward = 200;

RobotPose homePose;
int greenOffset = 100;
int yellowOffset = 10;
int endTheta = -3140;

bool isYellowBall = false;

// Function Headers
float distanceFromRobot(int ballIndex);
int findNearestBall();
void moveForward(int ballNumber, int offset);
void rotateToHit();
void rotateToBall(int ballNumber, int offset);
int toBallOrientation(int ballNumber, int offset);
int orientedTo(int ballNumber, int offset);
bool homeSide(int ballIndex);
bool isYellow(int ballIndex);
bool atBall(int ballIndex, int offset);


//Input of -100 to 100 for easiest backward v Forward
void leftWheelWrite(int leftSpeed)
{
  if(leftSpeed >= -100 && leftSpeed <= 100)
  {
    leftwheel.write(map(leftSpeed,-100,100,1300,1700));
  }
}

void rightWheelWrite(int rightSpeed)
{
  if(rightSpeed >= -100 && rightSpeed <= 100)
  {
    rightwheel.write(map(rightSpeed,-100,100,1300,1700));
  }
}

void driveSetup()
{
  Serial.println("DRIVE SETUP");
  rightwheel.attach(RIGHTWHEELPIN);
  leftwheel.attach(LEFTWHEELPIN);

  homePose = robotPoses[MY_ROBOT_ID];

  if(homePose.x < (X_MIN-X_MAX)/2){
    greenOffset *= -1;
    endTheta = 0;
  }
}

bool movingState()
{
  int nearestBall = findNearestBall();
  if(nearestBall != -1)
  {
    int offset = greenOffset;
    if(isYellowBall){
      offset = yellowOffset;
    }
    if(atBall(nearestBall,offset)){
      bool claw = clawState(); //true corrilates to closed
      if(isYellowBall){
        // Close Claw on yellow ball
      }
      else{
        // Hit ball to our side by spinning in full circle CCW
        rotateToHit();
      }
    }
    else if(orientedTo(nearestBall,offset)){
      Serial.println("Move Forward");
      moveForward(nearestBall, greenOffset);
    }
    else{
      Serial.println("Rotate To Ball");
      rotateToBall(nearestBall, greenOffset);
    }
  }
}
void rotateToHit()
{
  int prevTheta = abs(robotPoses[MY_ROBOT_ID].theta), curTheta;
  int totalRotation = 0;
  while(totalRotation < 2*3141)
  {
    rightWheelWrite(BASE_SPEED);
    leftWheelWrite(-BASE_SPEED);

    curTheta = abs(robotPoses[MY_ROBOT_ID].theta);
    
    totalRotation += prevTheta < curTheta ? curTheta - prevTheta : prevTheta - curTheta;

    prevTheta = curTheta;
  }
}

void rotateToBall(int ballNum, int offset)
{
  /*ball - robot
  greater than 3200 or less than -3200 rotate left
  less than 3200 and greater than -3200 rotate right */
  
  int phi = toBallOrientation(ballNum, offset);

  int rotateSpeed = map(phi,-PI*1000,PI*1000,-20,20);

  if(distanceFromRobot(ballNum) < 400){
    rotateSpeed += rotateSpeed < 0 ? -10: 10;
  }
  else{
    rotateSpeed *= 3;
  }
 
  leftWheelWrite(-rotateSpeed);
  rightWheelWrite(-rotateSpeed);
  if(rotateSpeed < 0){
    Serial.print(" Turning Right: ");
    Serial.println(rotateSpeed);
  }
  else{
    Serial.print(" Turning Left: ");
    Serial.println(rotateSpeed);
  }
}

void moveForward(int ballNumber, int offset)
{
  int phi = toBallOrientation(ballNumber, offset);

  float speedAdjust = map(phi,-maxAngleForward,maxAngleForward,-10,10);

  if(speedAdjust < 0){
    leftBaseSpeed += speedAdjust;
  }
  else{
    rightBaseSpeed -= speedAdjust;
  }

  leftWheelWrite(leftBaseSpeed);
  rightWheelWrite(rightBaseSpeed);
}

int toBallOrientation(int ballNumber, int offset)
{
  float XVrbo = (float)(ballPositions[ballNumber].x-robotPoses[MY_ROBOT_ID].x);
  float YVrbo = (float)(ballPositions[ballNumber].y + offset - robotPoses[MY_ROBOT_ID].y);

  float robotTheta = robotPoses[MY_ROBOT_ID].theta;
  float Xrbo = cos(robotTheta/1000.0)*XVrbo + sin(robotTheta/1000.0)*YVrbo;
  float Yrbo = -sin(robotTheta/1000.0)*XVrbo + cos(robotTheta/1000.0)*YVrbo;
  return atan2( Yrbo, Xrbo) * 1000;
}

int orientedTo(int ballNumber, int offset)
{
  int thetaRad = toBallOrientation(ballNumber, offset);
  Serial.print("Necessary Orientation: ");
  Serial.println(thetaRad);
  return (thetaRad < maxAngleForward && thetaRad > -maxAngleForward)? 1 : 0;
}

bool atBall(int ballIndex, int offset)
{
  return distanceFromRobot(ballIndex) <= offset ? true : false;
}

//Returns index of the nearest ball
int findNearestBall()
{
  if(!robotPoses[MY_ROBOT_ID].valid)
  {
    return -1;
  }
  float minDist = -1.0;
  int minDistIndex = -1;
  for(int ballIndex = 0; ballIndex < numBalls; ballIndex++){
    float tempDistance = distanceFromRobot(ballIndex);
    if(isYellow(ballIndex)){
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

bool isYellow(int ballIndex)
{
  return ballPositions[ballIndex].hue < 33 ? true : false;
}

bool homeSide(int ballIndex)
{
  int x_ballPose = ballPositions[ballIndex].x;
  if((homePose.x < (X_MAX - X_MIN)/2) && (x_ballPose < (X_MAX - X_MIN)/3)){
    return true;
  }
  else if((homePose.x > (X_MAX - X_MIN)/2) && (x_ballPose > (X_MAX - X_MIN) / 3 * 2)){
    return true;
  }
  return false;
}

// Returns distance to x and y coordinates
float distanceFromRobot(int ballIndex)
{
  if(robotPoses[MY_ROBOT_ID].valid)
    return sqrt(pow(ballPositions[ballIndex].x-robotPoses[MY_ROBOT_ID].x,2)+pow(ballPositions[ballIndex].y - robotPoses[MY_ROBOT_ID].y,2));
}
