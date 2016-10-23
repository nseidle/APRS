

//If we receive new I2C bytes record them as the angle we need to go to
void receiveEvent(int howMany)
{
  //Serial.print("howMany: ");
  //Serial.println(howMany);

  if (howMany == 3) //Some basic error filtering
  {
    whichSetting = Wire.read();
    byte chunk1 = Wire.read();
    byte chunk2 = Wire.read();
    int value = (int)chunk2 << 8 | chunk1;

    if (whichSetting == 4 || whichSetting == 6)
    {
      Serial.print("Recording to address: ");
      Serial.print(whichSetting);
      Serial.print(" value: ");
      Serial.println(value);
    }
    else
    {
      Serial.println("Received I2C Address out of range");
    }

    //Record this value into the proper setting
    switch (whichSetting)
    {
      case 4:
        //RSP comes in a -99 (full right) to 99 (full left). We need to map the request to actual min/max

        requestedSteeringPosition = value;
        requestedSteeringPosition = constrain(requestedSteeringPosition, -99, 99); //Limit this value to -100 to 99
        requestedSteeringPosition = map(requestedSteeringPosition, -99, 99, settingAngleMax, settingAngleMin);

        //Example
        //settingAngleMax = 550; //Max value to the right
        //settingAngleMin = 290; //Max value to the left
        //requestedSteeringPosition = map(requestedSteeringPosition, -99, 99, 550, 290);

        Serial.print("new RSP: ");
        Serial.println(requestedSteeringPosition);

        break;
      case 6:
        motorSpeed = value;
        motorSpeed = constrain(motorSpeed, 0, 99); //Limit this value to 0 to 99%

        timeSinceLastUpdate = millis();
        if (motorState != MOTOR_USER_EMERGENCY_STOP) motorState = RECEIVED_SETTING; //Realize that we have gotten an update over I2C
        break;
      default:
        //Master can't write to anything but requested steering and motorSpeed
        break;
    }
  }
  else if (howMany == 1)
  {
    //Then we are just getting the address or variable that the master wants to read
    whichSetting = Wire.read();

    Serial.print("Storing setting address: ");
    Serial.println(whichSetting);
  }

}

//If master asks for bytes back, send em!
void requestEvent()
{
  int value;

  //We should have been told previously which setting the master wanted.
  //We don't automatically incrementing the 'memory address' like normal i2c devices
  switch (whichSetting)
  {
    case 0:
      value = settingAngleStraight;
      break;
    case 1:
      value = settingAngleMax;
      break;
    case 2:
      value = settingAngleMin;
      break;
    case 3:
      value = settingPotDelta;
      break;
    case 4:
      value = requestedSteeringPosition;
      break;
    case 5:
      value = currentSteeringPosition;
      break;
    case 6:
      value = motorSpeed;
      break;
    case 7:
      value = throttlePosition;
      break;
    case 8:
      value = systemMode;
      break;
    case 9:
      value = brakeOn;
      break;
  }

  Serial.print("Sending value: ");
  Serial.println(value);

  //Report this value to the master
  //This is really screwy. We have to split the values into an array and send the array
  byte data[2];
  data[0] = value;
  data[1] = value >> 8;
  Wire.write(data, 2);
}


