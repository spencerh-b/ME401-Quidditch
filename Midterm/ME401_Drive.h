#include <SoftPWMServo.h>

SoftServo rightwheel, leftwheel;

#define MY_ROBOT_ID 8
#define RIGHTWHEELPIN 32
#define LEFTWHEELPIN 31

const int BASE_SPEED = 100;
int leftBaseSpeed = BASE_SPEED;
int rightBaseSpeed = -BASE_SPEED;

const int maxAngleForward = 200;

float distanceFromRobot(int ballIndex);
int findNearestBall();
void moveForward(int ballNumber);
void rotateToBall(int ballNumber);
int toBallOrientation(int ballNumber);
int orientedTo(int ballNumber);

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
}

void movingState()
{
  int nearestBall = findNearestBall();
  if(orientedTo(nearestBall))
  {
    Serial.println("Move Forward");
    moveForward(nearestBall);
  }
  else
  {
    Serial.println("Rotate To Ball");
    rotateToBall(nearestBall);
  }
  
}

void rotateToBall(int ballNum)
{
  /*ball - robot
  greater than 3200 or less than -3200 rotate left
  less than 3200 and greater than -3200 rotate right */
  
  int phi = toBallOrientation(ballNum);

  int rotateSpeed = map(phi,-PI*1000,PI*1000,-20,20);

  if(distanceFromRobot(ballNum) < 400)
  {
    rotateSpeed += rotateSpeed < 0 ? -10: 10;
  }
  else
  {
    rotateSpeed *= 3;
  }
 
  leftWheelWrite(-rotateSpeed);
  rightWheelWrite(-rotateSpeed);
  if(rotateSpeed < 0)
  {
    Serial.print(" Turning Right: ");
    Serial.println(rotateSpeed);
  }
  else
  {
    Serial.print(" Turning Left: ");
    Serial.println(rotateSpeed);
  }
}

void moveForward(int ballNumber)
{
  int phi = toBallOrientation(ballNum);

  float speedAdjust = map(phi,-maxAngleForward,maxAngleForward,-10,10);

  if(speedAdjust < 0)
  {
    leftBaseSpeed += speedAdjust;
  }
  else
  {
    rightBaseSpeed -= speedAdjust;
  }

  
  //Need to finish implmentation
  leftWheelWrite(leftBaseSpeed);
  rightWheelWrite(rightBaseSpeed);
}

int toBallOrientation(int ballNumber)
{
  float XVrbo = (float)(ballPositions[ballNumber].x-robotPoses[MY_ROBOT_ID].x);
  float YVrbo = (float)(ballPositions[ballNumber].y-robotPoses[MY_ROBOT_ID].y);

  float robotTheta = robotPoses[MY_ROBOT_ID].theta;
  float Xrbo = cos(robotTheta/1000.0)*XVrbo + sin(robotTheta/1000.0)*YVrbo;
  float Yrbo = -sin(robotTheta/1000.0)*XVrbo + cos(robotTheta/1000.0)*YVrbo;
  return atan2( Yrbo, Xrbo) * 1000;
}

int orientedTo(int ballNumber)
{
  int thetaRad = toBallOrientation(ballNumber);
  Serial.print("Necessary Orientation: ");
  Serial.println(thetaRad);
  return (thetaRad < maxAngleForward && thetaRad > -maxAngleForward)? 1 : 0;
}

//Returns index of the nearest ball
int findNearestBall()
{
  float minDist = -1.0;
  int minDistIndex = -1;
  for(int ballIndex = 0; ballIndex < numBalls; ballIndex++){
    float tempDistance = distanceFromRobot(ballIndex);
    if (minDistIndex == -1 || tempDistance < minDist)
    {
      minDist = tempDistance;
      minDistIndex = ballIndex;
    }
  }
  Serial.print("Nearest Ball: ");
  Serial.println(minDistIndex);
  return minDistIndex;
}

// Returns distance to x and y coordinates
float distanceFromRobot(int ballIndex)
{
  if(robotPoses[MY_ROBOT_ID].valid)
    return sqrt(pow(ballPositions[ballIndex].x-robotPoses[MY_ROBOT_ID].x,2)+pow(ballPositions[ballIndex].y-robotPoses[MY_ROBOT_ID].y,2));
}
