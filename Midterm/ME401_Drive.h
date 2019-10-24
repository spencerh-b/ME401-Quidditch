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
bool wasYellowBall = false; // has the yellow ball been seen

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
void stopMovement();
bool isHome();
void determineSide(int setpoint);
void escape();
int isYellowBall();


/*********************************************************************
* Functions: WheelWrite                                              *
* Encapsulation of drive commands into two functions, to reduce the  *
* level of confussion as well as always check for proper inputs.     *
*********************************************************************/
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

/*********************************************************************
* Function: Drive Setup                                              *
* Initializes the motors and records the initial position before any *
* execution begins in the main loop                                  *
*********************************************************************/
void driveSetup()
{
  Serial.println("DRIVE SETUP");
  rightwheel.attach(RIGHTWHEELPIN);
  leftwheel.attach(LEFTWHEELPIN);

  if(init_x_pose < (X_MIN-X_MAX)/2){
    init_x_pose = myRobot.x;
  }
}

/*********************************************************************
* Function: movingState                                              *
* Controls the actions of the robot while it is attempting to        *
* retrieve balls. It returns a boolean value representing the        *
* achievement of capturing an individual ball.                       *
* Logic:                                                             *
* 1) Check that robot is visible to camera                           *
*     - If not stop wheel movement                                   *
* 2) Update robot pose global variable                               *
* 3) If the claw is being utilized [return true]                     *
* 4) Find the nearest ball                                           *
* 5) If the robot is oriented toward the ball move forward           *
* 6) Otherwise rotate to the ball                                    *
* [return false]                                                     *
*********************************************************************/
bool movingState()
{
  if(!myRobot.valid){
    stopMovement();
  }

  myRobot = getRobotPose(MY_ROBOT_ID);
  
  if(clawState()){
    return true;
  }

  int nearestBall = findNearestBall();
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

/*********************************************************************
* Function: Return Home                                              *
* Controls the actions of the robot when it is has captured a ball.  *
* It returns a boolean value representing whether a ball has been    *
* successfully returned to the homeside                              *
* Logic:                                                             *
* 1) Update robot pose golbal variable                               *
* 2) If the robot is not yet to the home side                        *
*   - a) If oriented to the homeside move forward                    *
*   - b) Otherwise rotate towards the homeside                       *
*   - [return false]                                                 *
* 3) If the robot is "home"                                          *
*   - open the claw to release the ball                              *
*   - drive away from ball                                           *
*   - [return true]                                                  *
*********************************************************************/
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

/*********************************************************************
* Function: isHome                                                   *
* Determines whether the robot is on it's homeside or not            *
*********************************************************************/
bool isHome()
{
  Serial.print("Home: ");
  Serial.print(homeLine);
  Serial.print("  Cur: ");
  Serial.println(myRobot.x);

  // Is the homeline is on the > half size, the robot must be greater than homeline, otherwise less than homeline
  if(homeLine > (X_MAX-X_MIN)/2){
    return myRobot.x >= homeLine ? true : false;
  }
  else{
    return myRobot.x <= homeLine ? true : false;
  }
}

/*********************************************************************
* Function: escape                                                   *
* Moves the robot in reverse direction for .5 seconds to "escape"    *
*********************************************************************/
void escape()
{
  leftWheelWrite(-BASE_SPEED);
  rightWheelWrite(BASE_SPEED);
  delay(500);
}

/*********************************************************************
* Function: determineSide                                            *
* Takes input parameters of the team number and the angle of the pid *
* sensor, and with the use of geometry is responsible for determining*
* the homeside of the robot. The "homeline" is the value of x that is*
* coorilated to the placement of captured balls.                     *
*********************************************************************/
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

/*********************************************************************
* Function: stopMovement                                             *
* Used to stop the robot from moving.                                *
*********************************************************************/
void stopMovement()
{
  rightWheelWrite(0);
  leftWheelWrite(0);
}

/*********************************************************************
* Function: rotateToFind                                             *
* Used to rotate the robot an arbitrary amount. This was initially   *
* implemented for the FINDCORNER state when the corner has not been  *
* seen on a given pass                                               *
*********************************************************************/
void rotateToFind(){
  rightWheelWrite(BASE_SPEED);
  leftWheelWrite(BASE_SPEED);
  delay(500);
  stopMovement();
}

/*********************************************************************
* Function: rotateToBall                                             *
* Called to rotate the robot toward a given ball. This function is   *
* aswell be setup to rotate toward the home side. It is important to *
* note that the rotation speed is variable depending on the amount of*
* rotation necessary.                                                *
* I.E. large rotation necessary, faster rotation speed               *
*********************************************************************/
void rotateToBall(int ballNum)
{
  //Find necessary rotation
  double phi = toBallOrientation(ballNum);
  
  //Maninpulate rotation speed based upon size of angle
  int rotateSpeed = map(phi,-PI*1000,PI*1000,-50,50);

  rotateSpeed = abs(rotateSpeed) < 10 ? rotateSpeed / abs(rotateSpeed) * 10 : rotateSpeed;
 
  leftWheelWrite(-rotateSpeed);
  rightWheelWrite(-rotateSpeed);

  // Debugging Statements
  if(rotateSpeed < 0){
    Serial.print(" Turning Right: ");
    Serial.println(rotateSpeed);
  }
  else{
    Serial.print(" Turning Left:   ");
    Serial.println(rotateSpeed);
  }
}

/*********************************************************************
* Function: moveForward                                              *
* Called when the robot is oriented toward a target ball and needs   *
* to move upon a straight line. The function as well adjusts its     *
* wheel speed to account for little deviations in angle toward the   *
* target. This function is as well instantiated to handle moving     *
* toward the homeside.                                               *
*********************************************************************/
void moveForward(int ballNumber)
{
  leftBaseSpeed = BASE_SPEED;
  rightBaseSpeed = -BASE_SPEED;

  //Get the angle away from the ball
  int phi = toBallOrientation(ballNumber);

  // Manipulate speed to correct little angle offsets
  float speedAdjust = map(phi,-maxAngleForward,maxAngleForward,-25,25);

  if(speedAdjust < 0){
    rightBaseSpeed -= speedAdjust;
  }
  else{
    leftBaseSpeed -= speedAdjust;
  }

  // Debug Statements
  Serial.print("Left: ");
  Serial.print(leftBaseSpeed);
  Serial.print("  Right: ");
  Serial.println(rightBaseSpeed);

  leftWheelWrite(leftBaseSpeed);
  rightWheelWrite(rightBaseSpeed);
}

/*********************************************************************
* Function: toBallOrientation                                        *
* Handles the calculation of angle between the robot and a desired   *
* point. This functions primary target is ball poses; however, if -2 *
* is passed into the funciton it is able to drive to the closest     *
* point at "home".    Function returns milliradians                  *
*********************************************************************/
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

/*********************************************************************
* Function: orientedTo                                               *
* Returns a boolean value that represents if the orientation of the  *
* robot is facing the desired target within a given range of values  *
*                                                                    *
* -maxAngle < robotTheta < maxAngle                                  *
*********************************************************************/
bool orientedTo(int ballNumber)
{
  int thetaRad = toBallOrientation(ballNumber);
  Serial.print("Necessary Orientation: ");
  Serial.println(thetaRad);
  return (thetaRad < maxAngleForward && thetaRad > -maxAngleForward)? true : false;
}

/*********************************************************************
* Function: findNearestBall                                          *
* Returns the index of the target ball to capture. These balls are   *
* prioritiezed as follows:                                           *
*   1) Return index of yellow ball if it exists on the board         *
*   2) Return index of the nearest non yellow ball to capture first  *
* The function as well ensures that the robot is visible in order to *
* mitigate any chance of seg fault. If there are no balls or the     *
* robot is not visible the function returns -1;                      *
*********************************************************************/
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
  
  // If the yellow ball exists, return it's index
  int yellowIndex = -1;
  if((yellowIndex = isYellowBall()) != -1)
  {
    return yellowIndex;
  }

  //For every ball see how close the ball is to the robot
  for(int ballIndex = 0; ballIndex < numBalls; ballIndex++){
    float tempDistance = distanceFromRobot(ballIndex);
    // If the ball is not on the homeside and (minDist has not been set or the distance is shorter) -> set the values
    if (!homeSide(ballIndex) && (minDistIndex == -1 || tempDistance < minDist)){ 
      minDist = tempDistance;
      minDistIndex = ballIndex;
    }
  }
  
  Serial.print("Nearest Ball: ");
  Serial.println(minDistIndex);
  return minDistIndex;
}

/*********************************************************************
* Function: isYellow                                                 *
* Returns a boolean representing weather the ball is yellow or not   *
* based upon its hue value                                           *
*********************************************************************/
bool isYellow(int ballIndex)
{
  return ballPositions[ballIndex].hue < 33 ? true : false;
}

/*********************************************************************
* Function: homeSide                                                 *
* Returns a boolean which coorilates to the position of the ball     *
* being on the home side or not                                      *
*********************************************************************/
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

/*********************************************************************
* Function: distanceFromRobot                                        *
* Returns the distance from the robot to the ball passed in          *
*********************************************************************/
float distanceFromRobot(int ballIndex)
{
  if(myRobot.valid)
    return sqrt(pow(ballPositions[ballIndex].x-myRobot.x,2)+pow(ballPositions[ballIndex].y - myRobot.y,2));
}

/*********************************************************************
* Function: isYellowBall                                             *
* Returns the index of yellow ball if it exists, otherwise return -1 *
*********************************************************************/
int isYellowBall()
{
  for(int ballIndex = 0; ballIndex < numBalls; ballIndex++){
    if(isYellow(ballIndex)){
      wasYellowBall = true;
      return ballIndex;
    }
  }
  wasYellowBall = false;
  return -1;
}
