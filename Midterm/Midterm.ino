#include "ME401_Radio.h"
#include "ME401_PID_IR.h"
#include "ME401_Switches.h"
#include "ME401_Team.h"

typedef enum states{
  FINDCORNER = 0,
  GETBALLS = 1,
  RETURNHOME = 2,
  SWITCHINTERUPT = 3
};

states robotState = FINDCORNER;

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

  switchSetup();

  teamSetup();
  // TODO: Do any other setups you need for limit switch inputs, etc.
  
}

void loop() {
  Serial.print("millis:");
  Serial.println(millis());
  
  

  // Simple example of looking for the corner beacon
  if(cornerFound == true){
    Serial.println("I've seen the corner");
  }
  int PIDangle = -1;
  bool returnState = false;
  int numBalls;

  if(switchState()){
    robotState = SWITCHINTERUPT;
  }

  if(teamState()){
    robotState = FINDCORNER;
  }
  switch(robotState)
  {
    case FINDCORNER:
      PIDangle = PID_IR_State();
      Serial.println(PIDangle);
      if(PIDangle != -1){
        robotState = GETBALLS;
        disablePIDandIR();
        determineSide(currentTeam,PIDangle);
      }
      break;
    case GETBALLS:
      {
        // Simple example of reading the robot and ball positions from the radio
        updateRobotPoseAndBallPositions();
        RobotPose robot = getRobotPose(MY_ROBOT_ID);
        
        if (robot.valid == true){
          Serial.println("The camera can see my robot");
          printRobotPose(robot);
        
          BallPosition ballPos[20];
          numBalls = getBallPositions(ballPos);
          printBallPositions(numBalls, ballPositions);
        
          returnState = movingState(); //Find closest ball, not on our side and move to that ball
          if(returnState == true){
            if(isYellowBall){
              return;
              winning();
            }
            robotState = RETURNHOME;
          }
        }
        else{
          Serial.println("No communications with Robot");
          stopRotation();
        }
      }
      break;
    /*case RETURNHOME:
      returnState = returnHome();
      if(returnState){
        robotState = GETBALLS;
      }
      
      break;
    case SWITCHINTERUPT:
      escape();
      robotState = GETBALLS;
      break;*/
  }
  Serial.print("State: ");
  Serial.println(robotState);
}
