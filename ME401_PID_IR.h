#include <SoftPWMServo.h>
#include <PID_v1.h>

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
double input=0, output=0, setpoint=0;
PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);


// Global variables for the IR beacon detector
int windowTime = 100;   // ms
int sampleTime= 1;     // ms
int windowIters = windowTime/sampleTime;
float frequency = 0;

// Forward declaration of functions to be used that are defined later than their first use
uint32_t MyCallback(uint32_t currentTime);

void setupPIDandIR(void)
{
  // Set up the quadrature inputs
  pinMode(EncoderAPin, INPUT);
  pinMode(EncoderBPin, INPUT);

  errorLeft = false;
  lastLeftA = digitalRead(EncoderAPin);
  lastLeftB = digitalRead(EncoderBPin);

  // Set up the motor outputs
  pinMode(MotorPWMPin, OUTPUT);
  pinMode(MotorDirectionPin, OUTPUT);

  digitalWrite(MotorPWMPin,0);
  digitalWrite(MotorDirectionPin,0);

  SoftPWMServoPWMWrite(MotorPWMPin, 0);


  //Setup the pid 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(pidSampleTime);
  myPID.SetOutputLimits(-255,255);

  // Initialize the sensor on pin 8 as an input
  pinMode(IRSensorInputPin,INPUT);

  // Initialize the timer interrupt that decodes the IR beacon signal
  attachCoreTimerService(MyCallback);
}



uint32_t MyCallback(uint32_t currentTime) {
  char newLeftA = digitalRead(EncoderAPin);
  char newLeftB = digitalRead(EncoderBPin);

  position += (newLeftA ^ lastLeftB) - (lastLeftA ^ newLeftB); 

  if((lastLeftA ^ newLeftA) & (lastLeftB ^ newLeftB))
  {
    errorLeft = true;
  }

  lastLeftA = newLeftA;
  lastLeftB = newLeftB;

  if (counterPID % 100*pidSampleTime == 0)
  {
    angle = position*0.133333;
    input = angle;      

    myPID.Compute();
    if (output < 0)
    {
      digitalWrite(MotorDirectionPin,1);
    }
    else
    {
      digitalWrite(MotorDirectionPin,0);
    }  
    SoftPWMServoPWMWrite(MotorPWMPin,abs(output));
    counterPID = 0;
  }
  counterPID++;
  
  if (counterIR % 100*irSampleTime == 0)
  {
    static int lastVal = digitalRead(IRSensorInputPin);
    static int iters = 0;
    static int ircounter = 0;

    if (iters < windowIters)
    {
      int newVal = digitalRead(IRSensorInputPin);
      if (newVal==HIGH && lastVal == LOW)
      {
        ircounter++;
      }
      lastVal = newVal;
    }
    else
    {
      frequency = 1000.0*(float)ircounter/(float)windowTime;
  
      ircounter = 0;
      int newVal = digitalRead(IRSensorInputPin);    
      lastVal = newVal;
      iters = 0;
    }
  
    iters++;
    
    counterIR=0;
  }
  counterIR++;

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
  else if (frequency >= 50 && frequency < 150)
  {
    return CORNER;
  }
  else
  {
    return NO_CORNER_HIGH;
  }
}
