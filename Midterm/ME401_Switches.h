int buttonStateLeft;
int buttonStateRight;

const int rightSwitchPin = 36;
const int leftSwitchPin = 37;

/*********************************************************************
* Function: switchSetup                                              *
* Initialize switch pins for determining state                       *
*********************************************************************/
void switchSetup()
{
  pinMode(rightSwitchPin,INPUT);
  pinMode(leftSwitchPin,INPUT);
}

/*********************************************************************
* Function: switchState                                              *
* Returns boolean representing if either switch is being pressed     *
*********************************************************************/
bool switchState()
{
  buttonStateLeft=digitalRead(leftSwitchPin);
  buttonStateRight=digitalRead(rightSwitchPin);

  // If either of the buttons are hit return true value
  if(buttonStateRight == 0 || buttonStateLeft == 0)
  {
    return true;
  }
  return false;
}
