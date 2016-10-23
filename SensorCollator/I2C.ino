
//If we receive new I2C bytes record them as the angle we need to go to
void receiveEvent(int howMany)
{
  if (howMany == 1)
  {
    //We are just getting the address or variable that the master wants to read
    whichSetting = Wire.read();
  }
}

//If master asks for bytes back, send em!
void requestEvent()
{
  float value = 0.0;

  long tempAge = millis() - gpsAge; //Calculate the age of this current packet

  //We should have been told previously which setting the master wanted.
  //We don't automatically incrementing the 'memory address' like normal i2c devices
  switch (whichSetting)
  {
    case 0: 
      value = (float)compassHeading;
      break;
    case 1:
      value = (float)gpsLat;
      break;
    case 2:
      value = (float)gpsLong;
      break;
    case 3:
      value = (float)gpsAltitude;
      break;
    case 4:
      value = (float)gpsHDOP;
      break;
    case 5:
      value = (float)gpsSIV;
      break;
    case 6:
      value = (float)tempAge;
      break;
    default:
      SerialUSB.println("Out of range");
      break;
  }

  //Report this value to the master
  //This is really screwy. We have to split the values into an array and send the array
  byte data[sizeof(float)];
  memcpy(data, &value, sizeof value);
  Wire.write(data, sizeof(float)); 
}

//I think the SAMD21 is getting fouled up when it's checking the compass
//on SERCOM1 I2C, and then receives a main bus I2C request
//These are assigned to the I2C interrupt handler when we don't want listen to the main bus
//Then they are assigned back to the main event handlers
//It's a hack
void dummyrequestEvent()
{
  
}

void dummyreceiveEvent(int howMany)
{
  
}

