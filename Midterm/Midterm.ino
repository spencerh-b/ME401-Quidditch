#include "ME401_Radio.h"
#include "ME401_PID_IR.h"

typedef enum states{
  FINDCORNER = 0,
  GETBALLS = 1
};

states state = FINDCORNER;

void setup() {

  // Set up the serial port in case we want output or input
  Serial.begin(115200);

  // Initialize the RFM69HCW radio
  ME401_Radio_initialize();

  // Initialize the PID and IR interrupts
  setupPIDandIR();
  updateRobotPoseAndBallPositions();
  // Initialize Drive Servos
  driveSetup();

  clawSetup();

  // TODO: Do any other setups you need for limit switch inputs, etc.
  
}

void loop() {
  Serial.print("millis:");
  Serial.println(millis());

  // Simple example of looking for the corner beacon
  if(cornerFound == true)
  {
    Serial.println("I've seen the corner");
  }
  bool cornerState;
  switch(state)
  {
    case FINDCORNER:
      cornerState = PID_IR_State();
      if(cornerState)
      {
        state = GETBALLS;
        disablePIDandIR();
      }
      break;
    case GETBALLS:
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
      printBallPositions(numBalls, ballPositions);
    
      bool moveState = movingState(); //Find closest ball, not on our side and move to that ball
    
      if(moveState) // Captured yellow ball
      {
        return;
      }
      break;
  }  

  delay(20);
}
