#include <Servo.h>

Servo rightwheel, leftwheel;

#define MY_ROBOT_ID 8
#define RIGHTWHEELPIN 32
#define LEFTWHEELPIN 31

const int BASE_SPEED = 100;
int leftBaseSpeed = BASE_SPEED;
int rightBaseSpeed = -BASE_SPEED;

float distanceFromRobot(int x, int y);
int findNearestBall();
void moveForward(int ballNumber);
void rotateToBall(int ballNumber);
int toBallOrientation(int ballNumber);
int orientedTo(int ballNumber);

void driveSetup()
{
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
  
  if(robotPoses[MY_ROBOT_ID].theta < phi)
  {
    leftwheel.write(-leftBaseSpeed);
    rightwheel.write(rightBaseSpeed);
    Serial.println(" Turning Left");
  }
  else
  {
    leftwheel.write(leftBaseSpeed);
    rightwheel.write(-rightBaseSpeed);
    Serial.println(" Turning Right");
  }
}

void moveForward(int ballNumber)
{
  //Need to finish implmentation
  leftwheel.write(leftBaseSpeed);
  rightwheel.write(rightBaseSpeed);
}

int toBallOrientation(int ballNumber)
{
  return 1000 * atan((float)(ballPositions[ballNumber].x-robotPoses[MY_ROBOT_ID].x) / (float)(ballPositions[ballNumber].y-robotPoses[MY_ROBOT_ID].y));
}

int orientedTo(int ballNumber)
{
  int thetaRad = toBallOrientation(ballNumber);
  Serial.print("Robot Orientation: ");
  Serial.println(robotPoses[MY_ROBOT_ID].theta);
  Serial.print("Necessary Orientation: ");
  Serial.println(thetaRad);
  return (thetaRad < robotPoses[MY_ROBOT_ID].theta + 150 && thetaRad > robotPoses[MY_ROBOT_ID].theta - 150)? 1 : 0;
}

//Returns index of the nearest ball
int findNearestBall()
{
  float minDist = -1.0;
  int minDistIndex = -1;
  for(int ballIndex = 0; ballIndex < numBalls; ballIndex++){
    float tempDistance = distanceFromRobot(ballPositions[ballIndex].x,ballPositions[ballIndex].y);
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
float distanceFromRobot(int x, int y)
{
  if(robotPoses[MY_ROBOT_ID].valid)
    return sqrt(pow(x-robotPoses[MY_ROBOT_ID].x,2)+pow(y-robotPoses[MY_ROBOT_ID].y,2));
}
