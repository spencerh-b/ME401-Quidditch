//int red_light_pin= 6;
int green_light_pin = 8;
int blue_light_pin = 35;
int buttonPin = 6;

//Function Header
void RGB_color(int red_light_value, int green_light_value, int blue_light_value);

/*********************************************************************
* Function: teamSetup                                                *
* Initialize the led and button pins to signify team designation     *
*********************************************************************/
void teamSetup()
{
  //pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);
  pinMode(buttonPin, INPUT);
}

int currentTeam = 0;

/*********************************************************************
* Function: teamState                                                *
* Returns boolean value signifying if the button has been push in    *
* order to change teams. Team 1 is represented by green. Team 2 is   *
* represented by blue.                                               *
*********************************************************************/
bool teamState()
{
  int buttonState = digitalRead(buttonPin);

  bool firstIttr = true;
  while(buttonState == 0){
    if(firstIttr){
      currentTeam = currentTeam == 0 ? 1 : 0;
      firstIttr = false;
    }
    buttonState = digitalRead(buttonPin);
  }
  
  //Red For Team 1
  if (currentTeam == 0)
  {
    RGB_color(0, 255, 0); // Green
  }
    //Blue for Team 2
  if (currentTeam == 1)
  {
    RGB_color(0, 0, 255); // Blue
  }
  if(firstIttr == false)
  {
    return true;
  }
  return false;
}

/*********************************************************************
* Function: RGB_color                                                *
* write to the RGB LED to change it's color                          *
*********************************************************************/
void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
{
  //analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}
