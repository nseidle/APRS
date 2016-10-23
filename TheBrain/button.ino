//This just does basic checking of the magic button

void setupButton()
{
  pinMode(DUIT_BUTTON, INPUT_PULLUP);
  pinMode(DUIT_LED, OUTPUT);
}

//If pressed for more than 2 seconds, confirm if we should go into nav mode or not
void buttonPressed()
{
  //User is pressing button, should we begin nav?

  dogDelay(1000); //Wait 1s to see if they are really serious
  
  if (digitalRead(DUIT_BUTTON) == LOW)
  {
    clearLCD();
    msg = "Begin Nav? Press to confirm";
    SerialUSB.println(msg);
    sendStringLCD(msg);

    //Wait for user to release button
    while (digitalRead(DUIT_BUTTON) == LOW) dogDelay(1);

    //Give user 5 seconds to confirm
    long startTime = millis();
    while (millis() - startTime < 5000)
    {
      dog.reset(); //pet the dog

      if (digitalRead(DUIT_BUTTON) == LOW)
      {
        delay(25); //Small debounce
        if (digitalRead(DUIT_BUTTON) == LOW)
        {
          //Ok, let's do this!

          //Start Nav
          navigate();

          //If we exit from navigate, return to stopped condition
          batmobile.Underway = UNDERWAY_STOPPED;
          writeUnderway();

          while (1)
          {
            dog.reset(); //pet the dog
          }
        }
      }

      //Display countdown
      int millisecondsPassed = millis() - startTime;
      if ( millisecondsPassed % 1000 == 0)
      {
        int secondsLeft = 5 - (millisecondsPassed / 1000);
        msg = " " + (String)secondsLeft;
        SerialUSB.println(msg);
        sendStringLCD(msg);

        delay(10); //So we don't print twice
      }
    }
    clearLCD();
    msg = "Nav aborted. Returning to normal.";
    SerialUSB.println(msg);
    sendStringLCD(msg);
  } //End 2s button hold check

  //digitalWrite(DUIT_LED, DUIT_LED_ON);
  //delay(250);
  //digitalWrite(DUIT_LED, DUIT_LED_OFF);
  //delay(250);
}
