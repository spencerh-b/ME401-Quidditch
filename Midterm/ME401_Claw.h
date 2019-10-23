#include <Servo.h>

int irSensor1Pin = A0;
int clawPin = 30;

const int openClawVal = 2500;
const int closedClawVal = 1900;

const int closeVal = 350;

Servo clawServo;

void clawSetup()
{
  clawServo.attach(clawPin);
  clawServo.write(openClawVal);
  delay(150);
}

bool openClaw()
{
  clawServo.write(openClawVal);
}

bool clawState(int newCloseVal = closeVal)
{ 
  //Serial.println("Claw State");
  int irSensor1Value = analogRead(irSensor1Pin);
  Serial.print("IR1:");
  Serial.println(irSensor1Value);

  if(irSensor1Value > newCloseVal){
    clawServo.write(closedClawVal);
    return true;
  }
  else{
    clawServo.write(openClawVal);
  }
  return false;
}
