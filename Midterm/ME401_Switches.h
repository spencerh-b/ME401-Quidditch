int buttonStateLeft;
int buttonStateRight;

void switchSetup()
{
  pinMode(36,INPUT);
  pinMode(37,INPUT);
}

bool switchState()
{
  buttonStateLeft=digitalRead(37);
  buttonStateRight=digitalRead(36);
  if(buttonStateRight == 0 || buttonStateLeft == 0)
  {
    return true;
  }
  return false;
}
