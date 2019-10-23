#include <SoftPWMServo.h>
#include <PID_v1.h>
#include "ME401_Drive.h"

#define CORNER 1
#define NO_CORNER_LOW      -1
#define NO_CORNER_HIGH      -2

// Pin definitions
// DO NOT CHANGE THESE ONES
const int EncoderAPin = 2;
const int EncoderBPin = 20;
const int MotorDirectionPin = 4;
const int MotorPWMPin = 3;

// YOU CAN CHANGE THIS ONE IF YOU WANT TO
const int IRSensorInputPin = 9;


// PID tuning gains

double ku = 85, tu=.116;
double kp=.6*ku, ki=2.0*kp/tu, kd=kp*tu/8.0;


// Global variables for quadrature decoder
static volatile char lastLeftA;
static volatile char lastLeftB;
static volatile bool errorLeft;
volatile long position = 0;
volatile int angle;

// Global variables for the timer interrupt handling
int pidSampleTime = 10;
int irSampleTime = 1;
long counterPID=1;
long counterIR=1;


// Global variables for the PID controller
double input=0, PIDoutput=0, setpoint=0;
PID myPID(&input, &PIDoutput, &setpoint,kp,ki,kd, DIRECT);


// Global variables for the IR beacon detector
int windowTime = 100;   // ms
float sampleTime = 0.01;     // ms
int windowIters = windowTime/sampleTime;
float frequency = 0;

// Our own variables
bool cornerFound = false;
long PIDcounter = 1;
int pointChange = 1;
// Forward declaration of functions to be used that are defined later than their first use
uint32_t MyCallback(uint32_t currentTime);
int readIRFrequency ();

void disablePIDandIR(void){ 
    detachCoreTimerService(MyCallback);
}

void setupPIDandIR(void)
{
  
  // Set up the quadrature inputs
  pinMode(EncoderAPin, INPUT);
  pinMode(EncoderBPin, INPUT);

  errorLeft = false;
  lastLeftA = digitalRead(EncoderAPin);
  lastLeftB = digitalRead(EncoderBPin);

  // Set up the motor PIDoutputs
  pinMode(MotorPWMPin, OUTPUT);
  pinMode(MotorDirectionPin, OUTPUT);

  digitalWrite(MotorPWMPin,0);
  digitalWrite(MotorDirectionPin,0);

  SoftPWMServoPWMWrite(MotorPWMPin, 0);


  //Setup the pid 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(pidSampleTime);
  myPID.SetOutputLimits(-40,40);

  // Initialize the sensor on pin 8 as an input
  pinMode(IRSensorInputPin,INPUT);

  // Initialize the timer interrupt that decodes the IR beacon signal
  attachCoreTimerService(MyCallback);
}

int overShootCounter = 0;

int PID_IR_State(void)
{
  if (readIRFrequency() == CORNER)
  {
    Serial.println("I see the corner");
    cornerFound = true;
    return setpoint;
  }
  else
  {
    Serial.print("Can't see the corner:");
    Serial.println(frequency);
    setpoint += pointChange;

    if (setpoint > 30 || setpoint < -20){
      pointChange *= -1;
      overShootCounter++;
    }
    if(overShootCounter == 2){
      rotateToFind();
      overShootCounter = 0;
    }
    return -1;
  }
}



uint32_t MyCallback(uint32_t currentTime) {
  static int lastVal = digitalRead(IRSensorInputPin);
  static int iters = 0;
  static int IRcounter = 0;
  int newVal = digitalRead(IRSensorInputPin);

  if (iters < windowIters)
  {
    
    if (newVal==HIGH && lastVal == LOW)
    {
      IRcounter++;
    }
    lastVal = newVal;
  }
  else
  {
    frequency = 1000.0*(float)IRcounter/(float)windowTime;
    IRcounter = 0;   
    lastVal = newVal;
    iters = 0;
  }

  iters++;
  //return (currentTime + CORE_TICK_RATE*sampleTime);

  //IR Sensor ^
  char newLeftA = digitalRead(2);
  char newLeftB = digitalRead(20);
  
  position += (newLeftA ^ lastLeftB) - (lastLeftA ^ newLeftB);
  
  if((lastLeftA ^ newLeftA) & (lastLeftB ^ newLeftB))
  {
      errorLeft = true;
  }
  
  lastLeftA = newLeftA;
  lastLeftB = newLeftB;
    
  
  if (PIDcounter % 100*pidSampleTime == 0)
  {   
    input = position*.13333333;      
        
    myPID.Compute();
    
    if (PIDoutput > 0)
    {
      digitalWrite(MotorDirectionPin,1); // Left
    }
    else
    {
      digitalWrite(MotorDirectionPin,0); // Right
    }  
    SoftPWMServoPWMWrite(3,abs(PIDoutput));

    PIDcounter = 0;
  }
  PIDcounter++;
  return (currentTime + CORE_TICK_RATE/100);
}

// The readFrequency function returns a value indicating whether it is detecting the different corners or a freqency that is
// either too high or too low.
int readIRFrequency ()
{
  if (frequency < 50)
  {
    return NO_CORNER_LOW;
  }
  else if (frequency >= 50 && frequency < 450)
  {
    return CORNER;
  }
  else
  {
    return NO_CORNER_HIGH;
  }
}
