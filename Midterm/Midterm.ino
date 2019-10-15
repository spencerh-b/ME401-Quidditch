#include "ME401_Radio.h"
#include "ME401_PID_IR.h"
#include "ME401_Drive.h"


void setup() {

  // Set up the serial port in case we want output or input
  Serial.begin(115200);

  // Initialize the RFM69HCW radio
  ME401_Radio_initialize();

  // Initialize the PID and IR interrupts
  setupPIDandIR();

  // Initialize Drive Servos
  driveSetup();

  clawSetup();

  // TODO: Do any other setups you need for limit switch inputs, etc.
  
}

void loop() {


  Serial.print("millis:");
  Serial.println(millis());

  /* Simple example of sweeping the DC motor
  setpoint += 5;
  if (setpoint > 60)
    setpoint = -60;
  */
  // Simple example of looking for the corner beacon
  if (readIRFrequency() == CORNER)
  {
    Serial.println("I see the corner");
  }
  else
  {
    Serial.print("Can't see the corner:");
    Serial.println(frequency);
  }


  /* 1. Find Corner all the time
   * 
   * 2. Look for yellow ball
   *  2a. If yellow ball - Drive and Capture
   *  2b. If green ball - Drive and Hit
   *      If the ball is directly in front, capture, rotate and throw
   *      
   * - Drive -- if switches are triggered execute evasive Tactics.
   */
  
  

  // Simple example of reading the robot and ball positions from the radio
  updateRobotPoseAndBallPositions();
  RobotPose robot = getRobotPose(MY_ROBOT_ID);
  
  if (robot.valid == true)
  {
    Serial.println("The camera can see my robot");
    printRobotPose(robot);
  }

  BallPosition ballPos[20];
  int numBalls = getBallPositions(ballPos);
  Serial.print("NUM BALLS: ");
  Serial.println(numBalls);
  printBallPositions(numBalls, ballPositions);

  //Find closest ball, not on our side

  bool moveState = movingState(); //Move to that ball

  if(moveState)
  {
    return;
  }

  delay(20);
}
