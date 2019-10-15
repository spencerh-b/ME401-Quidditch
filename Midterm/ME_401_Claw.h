#include <Servo.h>

int irSensor1Pin = A0;

Servo clawServo;

void clawSetup()
{
  clawServo.attach(30);
  clawServo.write(1450);
}

bool clawState()
{
  Serial.println("Claw State");
  int irSensor1Value = analogRead(irSensor1Pin);

  if(irSensor1Value > 400)
  {
    Serial.print("IR1:");
    Serial.print(irSensor1Value);
    clawServo.write(1500); // NEEDS TO COORILATE TO VALUE OF CLOSE ANGLE
  }
  
}
