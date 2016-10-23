/*
  These are all the functions specific to steering and the linear actuator

*/

//Sets up the various pins
void setupSteering()
{
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  steerStop(); //Turn off the linear actuator
  steerState = STEER_STOP;

  pinMode(STEER_POT, INPUT);
  pinMode(STEER_POT_HIGH, OUTPUT);
  pinMode(STEER_POT_LOW, OUTPUT);
  digitalWrite(STEER_POT_HIGH, HIGH); //This puts 5V across the trim pot
  digitalWrite(STEER_POT_LOW, LOW);

  //Check every so often to see if steering has reach requested state
  steeringID = timer.setInterval(STEERING_CHECK_TIME, steeringUpdate); 

  readSystemSettings(); //Load all system steering settings from EEPROM
  requestedSteeringPosition = readSteerPot(); //Assume we are where we are supposed to be
  currentSteeringPosition = readSteerPot();
}

//Every 100ms check to see if we have arrived at the position we were tasked to get to
void steeringUpdate()
{
  wdt_reset(); //Pet the dog

  currentSteeringPosition = readSteerPot(); //Where are we at?

  //Check emergency stops
  //The linear actuator has its own stops but they are past where the mechanics will allow it
  if( (currentSteeringPosition - settingPotDelta) >= settingAngleMax)
  {
    steerStop();
  }
  else if( (currentSteeringPosition + settingPotDelta) <= settingAngleMin)
  {
    steerStop();
  }

  //Check to see if we have arrived at the requested position
  if( currentSteeringPosition <= (requestedSteeringPosition + settingPotDelta) && currentSteeringPosition >= (requestedSteeringPosition - settingPotDelta))
  {
    if(steerState != STEER_USER_EMERGENCY_STOP)
    {
      steerStop();
      steerState = STEER_ARRIVED;
    }
  }
  else if( currentSteeringPosition < (requestedSteeringPosition + settingPotDelta))
  {
    if(steerState != STEER_USER_EMERGENCY_STOP) steerLeft();
  }
  else if( currentSteeringPosition > (requestedSteeringPosition - settingPotDelta))
  {
    if(steerState != STEER_USER_EMERGENCY_STOP) steerRight();
  }
}

//Turn off the actuator
void steerStop()
{
  digitalWrite(RELAY1, STEER_RELAY_OFF);
  digitalWrite(RELAY2, STEER_RELAY_OFF);
  if(steerState != STEER_USER_EMERGENCY_STOP) steerState = STEER_STOP;
}

//Turn on actuator so that it begins driving right
void steerRight()
{
  digitalWrite(RELAY1, STEER_RELAY_OFF);
  digitalWrite(RELAY2, STEER_RELAY_ON);
  if(steerState != STEER_USER_EMERGENCY_STOP) steerState = STEER_RIGHT;
}

//Turn on actuator so that it begins driving left
void steerLeft()
{
  digitalWrite(RELAY1, STEER_RELAY_ON);
  digitalWrite(RELAY2, STEER_RELAY_OFF);
  if(steerState != STEER_USER_EMERGENCY_STOP) steerState = STEER_LEFT;
}

//Read the position of the steering pot
int readSteerPot()
{
  int potPosition = averageAnalogRead(STEER_POT);

  //Map the analog value to a angle of attack
  int steeringAngle = potPosition;
  
  return(steeringAngle);
}

//Calibrate center steering position
//Allow user to very slightly adjust the 'drive straight' point of the trimpot
void calibrateCenter(void)
{
  requestedSteeringPosition = settingAngleStraight; //Tell steering to go to center

  int newCenter = settingAngleStraight;

  while(1)
  {
    wdt_reset(); //Pet the dog
    Serial.println();
    Serial.print(F("Center: "));
    Serial.println(newCenter);
  
    Serial.println(F("L)eft 5"));
    Serial.println(F("R)ight 5"));
    Serial.println(F("l)eft 1"));
    Serial.println(F("r)ight 1"));
    Serial.print(F(">"));

    requestedSteeringPosition = newCenter; //Tell steering to go to this new center

    //Wait for user input
    while(Serial.available() == 0)
    {
      timer.run(); //This will drive/control the actuator
      wdt_reset(); //Pet the dog
    }
    
    char incoming = Serial.read();

    if(incoming == 'L')
      newCenter += 5;
    else if(incoming == 'R')
      newCenter -= 5;
    else if(incoming == 'l')
      newCenter += 1;
    else if(incoming == 'r')
      newCenter -= 1;
    else if(incoming == 'x')
    {
      settingAngleStraight = newCenter;
      writeEEPROMInt(LOCATION_STRAIGHT_ANGLE, settingAngleStraight); //Record to EEPROM
      
      Serial.println(F("Exiting"));
      return; //Look for escape character
    }

    //Error correct
    if(newCenter > 1023) newCenter = 1023;
    if(newCenter < 1) newCenter = 1;
  }
}
