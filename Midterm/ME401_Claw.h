#include <Servo.h>

int irSensor1Pin = A0;
int clawPin = 30;

const int openClawVal = 2250;
const int closedClawVal = 1755;
const int interClawVal = 1900;

const int closeVal = 325;

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

bool clawState()
{ 
  //Serial.println("Claw State");
  int irSensor1Value = analogRead(irSensor1Pin);
  Serial.print("IR1:");
  Serial.println(irSensor1Value);

  if(irSensor1Value > closeVal){
    clawServo.write(closedClawVal);
    return true;
  }
  else if(irSensor1Value > closeVal/3){
    int tempClaw = map(irSensor1Value,closeVal/3,closeVal,openClawVal,interClawVal);
    clawServo.write(tempClaw);
  }
  else{
    clawServo.write(openClawVal);
  }
  return false;
}
