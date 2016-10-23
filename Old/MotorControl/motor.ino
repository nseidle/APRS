

void setupMotor()
{
  setPwmFrequency(THROTTLE_OUT, 1); //Set the freq of this pin to its base freq so we can't hear it
  pinMode(THROTTLE_OUT_VCC, OUTPUT);
  digitalWrite(THROTTLE_OUT_VCC, HIGH);

  pinMode(THROTTLE_IN, INPUT);

  timeOutID = timer.setInterval(MAX_TIMEOUT, motorSafetyCheckIn); //If after Xms we have no update then shut off motor
  motorUpdateID = timer.setInterval(MOTORUPDATE_TIME, motorUpdate); //Update motor every Xms
  
  motorSpeed = 0;
  motorState = MOTOR_STOP;
}
//Every few ms read the hall effect sensor on the user throttle
void throttleUpdate()
{
  int reading = averageAnalogRead(THROTTLE_IN);

  //Serial.print("analog reading: ");
  //Serial.println(reading);

  //This is a safety check. Shouldn't be allowed to happen but I've seen it happen when
  //the throttle becomes disconnected  
  if(reading < 10) //Pull down brings this to 0
  {
    if(millis() - throttleOffline > 1000)
    {
      //Throttle hasn't been disconnected for 2s, oh shit
      motorState = MOTOR_STOP;
      motorSpeed = 0;
      analogWrite(THROTTLE_OUT, motorSpeed); //Shut it down
  
      Serial.println("Throttle disconnect detected. Powering down.");
      blinkError(ERROR_THROTTLE_RANGE);
    }
  }
  else
  {
    //Throttle was detected so update its time stamp
    throttleOffline = millis();
  }

  throttlePosition = map(reading, 0, 1023, 0, 99); //Map the throttle reading to the expected 0 to 99 % control value

  //We need to map the hall effect sensor that goes from 1-4V to 0 to 99
  throttlePosition = map(throttlePosition, HALL_MIN, HALL_MAX, 0, 99);

  throttlePosition = constrain(throttlePosition, 0, 99); //Make sure we don't go above 99 or below 0
}

//Check to see if we have received an update
//If we haven't heard from the main controller within a certain amount of time then shut down
void motorSafetyCheckIn()
{
  wdt_reset(); //Pet the dog

  if (motorState != MOTOR_STOP && (millis() - timeSinceLastUpdate) > MAX_TIMEOUT)
  {
    if(motorState != MOTOR_USER_EMERGENCY_STOP) motorState = MOTOR_STOP;
    motorSpeed = 0;
    analogWrite(THROTTLE_OUT, motorSpeed); //Shut it down

    Serial.println("No update. Shutting down.");
  }
}

//Every few 100 ms update the motor spped
void motorUpdate()
{
  byte motorValue = 0;

  //Here is where we decide whether to listen to the human or not
  //If we have input over I2C we take it
  //If not, we take the throttle value
  //If not throttle value then go to zero

  if (motorSpeed > 0) //motorSpeed comes from I2C or serial command
    motorValue = motorSpeed;
  else if (systemMode == HUMAN && throttlePosition > 0)
  {
    //throttlePosition comes from reading the user's input or throttle position
    //It only comes into play if the Manual Override Switch is activated
    //And we only get this far if the system is *not* receiving an I2C command (motorSpeed)
    motorValue = throttlePosition; 
  }
  else
    motorValue = 0; //If nothing else, just stop

  motorValue = constrain(motorValue, 0, 99); //Make sure we don't go above 99
  byte pwmValue = map(motorValue, 0, 99, 0, 255);

  Serial.print(" motorValue[");
  Serial.print(motorValue);
  Serial.print("]");
  
  analogWrite(THROTTLE_OUT, pwmValue);
}

//Comes from http://playground.arduino.cc/Code/PwmFrequency
//Allows us to set the base PWM freq for various PWM pins
//Pins 3, 9, 10, and 11 has base freq of 31250 Hz
//Pins 5, 6 has base freq of 62500 Hz
//Note that this function will have side effects on anything else that uses timers:
//Changes on pins 3, 5, 6, or 11 may cause the delay() and millis() functions to stop working. Other timing-related functions may also be affected.
//Changes on pins 9 or 10 will cause the Servo library to function incorrectly.
void setPwmFrequency(int pin, int divisor)
{
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

