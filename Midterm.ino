#include "ME401_Radio.h"
//#include "IR_Sensor.h"

void setup()
{
  // Open a serial port so we can send keystrokes to the module:  
  Serial.begin(115200);
  Serial.println("PRE-INITIALIZE");
    
  // Initialize the RFM69HCW:
  ME401_Radio_initialize();


  
  Serial.println("POST-INITIALIZE");
}

int K = 0;
void loop()
{
  radioState();
  
  delay(50);
}
