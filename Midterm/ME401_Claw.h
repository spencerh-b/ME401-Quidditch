#include <Servo.h>

int irSensor1Pin = A0;
int clawPin = 30;

const int openClawVal = 2500;
const int closedClawVal = 1900;

const int closeVal = 350;

Servo clawServo;

/*********************************************************************
* Function: clawSetup                                                *
* Initialize claw pin and ensure that the claw is open for use       *
*********************************************************************/
void clawSetup()
{
  clawServo.attach(clawPin);
  clawServo.write(openClawVal);
  delay(150);
}

/*********************************************************************
* Function: openClaw                                                 *
* Open the claw even if in possesion of a captured ball              *
*********************************************************************/
void openClaw()
{
  clawServo.write(openClawVal);
}

/*********************************************************************
* Function: clawState                                                *
* Returns boolean variable signifying if a ball has been captured or *
* not. The input parameter allows the IR value to be raised or       *
* lowered if the IR Sensor needs to be more or less sensative        *
*********************************************************************/
bool clawState(int newCloseVal = closeVal)
{ 
  //Serial.println("Claw State");
  int irSensor1Value = analogRead(irSensor1Pin);

  //Debuggin Statment
  Serial.print("IR1:");
  Serial.println(irSensor1Value);

  // If the ir value is greater than the set limit, close the claw
  if(irSensor1Value > newCloseVal){
    clawServo.write(closedClawVal);
    return true;
  }
  // Otherwise open the claw
  else{
    clawServo.write(openClawVal);
  }
  return false;
}
