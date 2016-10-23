
#include <SoftwareWire.h> //Needed for software I2C to MCP4725
SoftwareWire softWire(7, 6, false); //SDA, SCL, don't use internal pullups
#define MCP4725_ADDR 0x60   


void setupMotor()
{
  pinMode(THROTTLE_IN, INPUT);

  softWire.begin(); //Setup the software based I2C for analog motor control

  timeOutID = timer.setInterval(MAX_TIMEOUT, motorSafetyCheckIn); //If after Xms we have no update then shut off motor
  motorUpdateID = timer.setInterval(MOTORUPDATE_TIME, motorUpdate); //Update motor every Xms
  
  motorSpeed = 0;
  motorState = MOTOR_STOP;
}
//Every few ms read the hall effect sensor on the user throttle
void throttleUpdate()
{
  int reading = averageAnalogRead(THROTTLE_IN);

  Serial.print("analog reading: ");
  Serial.println(reading);

  //This is a safety check. Shouldn't be allowed to happen but I've seen it happen when
  //the throttle becomes disconnected  
  if(reading < 10) //Pull down brings this to 0
  {
    if(millis() - throttleOffline > 1000)
    {
      //Throttle hasn't been disconnected for 2s, oh shit
      motorState = MOTOR_STOP;
      motorSpeed = 0;
      setThrottle(motorSpeed); //Shut it down
  
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
    setThrottle(motorSpeed); //Shut it down

    Serial.println("No update. Shutting down.");
  }
}

//Every few 100 ms update the motor spped
void motorUpdate()
{
  //Here is where we decide whether to listen to the human or not
  //If we have input over I2C we take it
  //If not, we take the throttle value
  //If not throttle value then go to zero

  if (systemMode == AUTO && motorSpeed > 0 ) //motorSpeed comes over I2C
  {
    masterMotorValue = motorSpeed;
  }
  else if (systemMode == HUMAN && throttlePosition > 0)
  {
    //throttlePosition comes from reading the user's input or throttle position
    //It only comes into play if the Manual Override Switch is activated
    //And we only get this far if the system is *not* receiving an I2C command (motorSpeed)
    masterMotorValue = throttlePosition; 
  }
  else
    masterMotorValue = 0; //If nothing else, just stop

  masterMotorValue = constrain(masterMotorValue, 0, 99); //Make sure we don't go above 99
  byte pwmValue = map(masterMotorValue, 0, 99, 0, 255);

  setThrottle(pwmValue);
}

//Controls the MCP4725 over software I2C to set the analog value sent to the motor controller
//Input is a PWM value of 0 to 255
//Output is 0 to 5V
void setThrottle(byte throttleValue)
{
  uint16_t mcpValue = map(throttleValue, 0, 255, 0, 4095);

  softWire.beginTransmission(MCP4725_ADDR);
  softWire.write(64); //Command to update the DAC
  softWire.write(mcpValue >> 4); // the 8 most significant bits...
  softWire.write(mcpValue << 4); // the 4 least significant bits...
  softWire.endTransmission();
}

