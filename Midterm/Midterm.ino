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
  Serial.print("BEGIN");
  // Initialize the RFM69HCW radio
  ME401_Radio_initialize();

  // Initialize the PID and IR interrupts
  setupPIDandIR();
  updateRobotPoseAndBallPositions();

  //Init switches
  switchSetup();

  //Init team determination
  teamSetup();  

  //Init Drice State
  driveSetup();
        
  // Init front claw
  clawSetup(); 
}



void loop() {
  Serial.print("millis:");
  Serial.println(millis());
  
  // Update robot and ball positions every itteration to ensure proper data is used
  updateRobotPoseAndBallPositions();
  RobotPose robot = getRobotPose(MY_ROBOT_ID);
  
  //Dubugging Statements
  printRobotPose(robot);     
  BallPosition ballPos[20];
  numBalls = getBallPositions(ballPos);
  printBallPositions(numBalls, ballPositions);

  if(cornerFound == true){
    Serial.println("I've seen the corner");
  }

  int PIDangle = -1;
  bool returnState = false;
  int numBalls;

  // If the switch is hit, always execute interupt state
  if(switchState()){
    robotState = SWITCHINTERUPT;
  }

  // If the team is changed, restart execution process
  if(teamState()){
    robotState = FINDCORNER;
  }

  // State machine
  switch(robotState)
  {
    case FINDCORNER:
      // Get the angle of the IR sensor to know where it is looking
      PIDangle = PID_IR_State();
      Serial.println(PIDangle);

      // If an angle was returned use it to calculate position and determine the where "home" is
      if(PIDangle != -1){
        robotState = GETBALLS; // Next state is GETBALLS
        disablePIDandIR(); //Reduce computational load by disconnecting interupt
        determineSide(currentTeam,PIDangle); // Determine side
      }
      break;
    case GETBALLS:
      {       
        if (robot.valid == true){
          //Debugging statement
          Serial.println("The camera can see my robot");
               
          returnState = movingState(); //Find closest ball, not on our side and move to that ball
          // If return state is true that means that a ball has been captured
          if(returnState){
            // If the captured ball is yellow, GAMEOVER
            if(wasYellowBall){
              stopMovement();
              exit(0);
            }
            // If the ball is not yellow bring it home
            robotState = RETURNHOME;
          }
        }
        else{
          //Debugging statement
          Serial.println("No communications with Robot");
          stopMovement();
        }
      }
      break;
    case RETURNHOME:
      // Hold on the ball with less chance of error be reducing the IR val neccesary to hold on
      returnState = clawState(100);

      // if the claw is not closed [false] go find a ball again or if the yellow ball now exists, drop the ball
      if(!returnState  || isYellowBall() != -1){
        openClaw();
        escape();
        robotState = GETBALLS;
      }
      // Return the ball to "home"
      returnState = returnHome();
      // if return state is true (robot is "home") go get a new ball
      if(returnState){
        robotState = GETBALLS;
      }
      
      break;
    case SWITCHINTERUPT:
      // If the interupt switches are hit, back away from the situation.
      escape();
      robotState = GETBALLS;
      break;
  }

  // Dubugging statement
  Serial.print("State: ");
  Serial.println(robotState);
}


/*********************************************************************
* Function:                                                  *
*  *
*  *
*  *
*  *
*********************************************************************/
