//int red_light_pin= 6;
int green_light_pin = 8;
int blue_light_pin = 35;
int buttonPin = 39;

void RGB_color(int red_light_value, int green_light_value, int blue_light_value);

void teamSetup()
{
  //pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);
  pinMode(buttonPin, INPUT);
}

int currentTeam = 0;

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
    RGB_color(0, 255, 0); // Red
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

void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
{
  //analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}
