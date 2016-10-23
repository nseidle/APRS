
//If we receive new I2C bytes record them as the angle we need to go to
void receiveEvent(int howMany)
{
  if (howMany == 1)
  {
    //Then we are just getting the address or variable that the master wants to read
    whichSetting = Wire.read();

    SerialUSB.print("Received request for address: ");
    SerialUSB.println(whichSetting);
  }
  else
  {
    SerialUSB.print("Received request for too many bytes: ");
    SerialUSB.println(howMany);
  }

}

//If master asks for bytes back, send em!
void requestEvent()
{
  long value = 0;

  //We should have been told previously which setting the master wanted.
  //We don't automatically incrementing the 'memory address' like normal i2c devices
  /*
  0x00: Send all 6 datums in order, 4*6 = 24 bytes
  0x01: distance 1 in mm (long, four bytes)
  0x02: distance 2
  0x03: distance 3
  0x04: age of reading 1 in ms (long, four bytes)
  0x05: age of reading 2
  0x06: age of reading 3
  */

  if (whichSetting == 0x00)
  {
    //Send all the settings!
    for (byte setting = 1 ; setting < 7 ; setting++)
    {
      switch (setting)
      {
        case 1:
          value = laser1.measurement.currentDistance;
          break;
        case 2:
          value = laser2.measurement.currentDistance;
          break;
        case 3:
          value = laser3.measurement.currentDistance;
          break;
        case 4:
          value = millis() - laser1.measurement.time;
          break;
        case 5:
          value = millis() - laser2.measurement.time;
          break;
        case 6:
          value = millis() - laser3.measurement.time;
          break;
        default:
          SerialUSB.println("Out of range");
          break;
      }
      SerialUSB.print("Sending value: ");
      SerialUSB.println(value);

      //Report this value to the master
      byte data[sizeof(long)];
      memcpy(data, &value, sizeof value);
      Wire.write(data, sizeof(long));
    }

  }
  else //Just send one setting
  {
    switch (whichSetting)
    {
      case 1:
        value = laser1.measurement.currentDistance;
        break;
      case 2:
        value = laser2.measurement.currentDistance;
        break;
      case 3:
        value = laser3.measurement.currentDistance;
        break;
      case 4:
        value = millis() - laser1.measurement.time;
        break;
      case 5:
        value = millis() - laser2.measurement.time;
        break;
      case 6:
        value = millis() - laser3.measurement.time;
        break;
      default:
        SerialUSB.println("Out of range");
        break;
    }

    SerialUSB.print("Sending value: ");
    SerialUSB.println(value);

    //Report this value to the master
    //This is really screwy. We have to split the values into an array and send the array
    byte data[sizeof(long)];
    memcpy(data, &value, sizeof value);
    Wire.write(data, sizeof(long));
  }
}
