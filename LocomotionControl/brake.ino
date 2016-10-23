/*
  These are all the functions specific to the mechanical brake and switch
*/

void setupBrake()
{
  pinMode(BRAKE_SWITCH, INPUT_PULLUP);

  brakeCheckID = timer.setInterval(BRAKE_CHECK_TIME, brakeCheck); //Check every few ms to see if user is hitting the brake
}

//Checks the brake switch
//Returns true if user is squeezing the mechanical brake
boolean userBraking()
{
  if (digitalRead(BRAKE_SWITCH) == LOW) return true;
  else return false;
}

//Every few ms check the brake to see if user is emergency stopping
void brakeCheck()
{
  //Check to see if user is hitting the brake
  if (userBraking() == true)
  {
    brakeOn = true;
    
    //Stop everything!
    motorState = MOTOR_USER_EMERGENCY_STOP;
    motorSpeed = 0;
    setThrottle(motorSpeed); //Shut it down

    steerStop();
    steerState = STEER_USER_EMERGENCY_STOP;
  }
  else
  {
    brakeOn = false;
    
    //We've released the brakes, Are we coming from a stop state?
    if(motorState == MOTOR_USER_EMERGENCY_STOP && steerState == STEER_USER_EMERGENCY_STOP)
    {
      //Yes, then reset everything
      motorState = MOTOR_STOP;
      steerState = STEER_STOP;
    }
  }
}

