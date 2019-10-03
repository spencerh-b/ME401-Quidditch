#include <Servo.h>

Servo rightwheel, leftwheel;

int leftwheelSpeed = 0, rightwheelSpeed = 0;

void driveSetup()
{
  Serial.begin(9600);
  rightwheel.attach(17);
  leftwheel.attach(18);

  leftwheelSpeed = map(leftwheelSpeed, -100, 100, 0, 180);
  rightwheelSpeed = map(rightwheelSpeed, 100, -100, 0, 180);
  leftwheel.write(leftwheelSpeed);
  rightwheel.write(rightwheelSpeed);
}

void movingState(int xPos, int yPos)
{
  leftwheelSpeed = 10; 
  rightwheelSpeed = 10;
  leftwheelSpeed = map(leftwheelSpeed, -100, 100, 0, 180);
  rightwheelSpeed = map(rightwheelSpeed, 100, -100, 0, 180);
  leftwheel.write(leftwheelSpeed);
  rightwheel.write(rightwheelSpeed);
}
