
//Read a given addr from the sensor controller
float readSensor(byte addr)
{
  Wire.beginTransmission(SENSOR_COLLATOR_ADDRESS);
  Wire.write(addr); //Set 'memory address' to whatever the caller tells us
  Wire.endTransmission(); //Removed to work on SAMD21

  //delay(10);

  //Read data from this device
  //Floats from sensor collator are 4 bytes
  if (Wire.requestFrom(SENSOR_COLLATOR_ADDRESS, 4) == 4)
  {
    byte data[sizeof(float)];
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();

    float value;
    memcpy(&value, data, sizeof value);

    //Compass adjust
    /*if (addr == SENSE_HEADING)
    {
      rawHeading = value;
      //Correct heading for whatever orientation it's in
      value -= (float)compassOffset;
      if (value < 0) value += (float)360;
      
      value = 360 - value; //Flip around because it's reverse of actual
    }*/

    return (value);
  }
  else
  {
    dog.reset(); //Pet the dog
    SerialUSB.println("GPS failed to respond.");
    return (-1);
  }
}

void compassCalibrate()
{
  while (1)
  {
    batmobile.heading = readSensor(SENSE_HEADING);
    batmobile.gpsLat = readSensor(SENSE_LAT) / (double)100000;
    batmobile.gpsLong = readSensor(SENSE_LONG) / (double)100000;
    batmobile.gpsAge = readSensor(SENSE_AGE);
    batmobile.gpsSIV = readSensor(SENSE_SIV);
    batmobile.gpsHDOP = readSensor(SENSE_HDOP);

    //Print out values of each axis
    SerialUSB.print("raw heading[");
    SerialUSB.print(rawHeading);
    SerialUSB.print("]adjusted head[");
    SerialUSB.print(batmobile.heading);
    SerialUSB.print("]gpsLat[");
    SerialUSB.print(batmobile.gpsLat, 5);
    SerialUSB.print("]gpsLong[");
    SerialUSB.print(batmobile.gpsLong, 5);
    SerialUSB.print("]gpsSIV[");
    SerialUSB.print(batmobile.gpsSIV);
    SerialUSB.print("]gpsHDOP[");
    SerialUSB.print(batmobile.gpsHDOP, 2);
    SerialUSB.print("]gpsAge[");
    SerialUSB.print(batmobile.gpsAge, 0);
    SerialUSB.print("]offset[");
    SerialUSB.print(compassOffset);
    SerialUSB.print("]");
    SerialUSB.println();

    dogDelay(250);

    if (SerialUSB.available())
    {
      char incoming = SerialUSB.read();

      if (incoming == 'a')
      {
        compassOffset += 10;
      }
      else if (incoming == 'z')
      {
        compassOffset -= 10;
      }
      else if (incoming == 's')
      {
        compassOffset += 1;
      }
      else if (incoming == 'x')
      {
        compassOffset -= 1;
      }
      else if (incoming == 'q')
      {
        break;
      }

      if (compassOffset > 360) compassOffset -= 360;
      if (compassOffset < 0) compassOffset += 360;
      eeWrite(COMPASS_LOCATION, compassOffset);
    }
  }

}

void gpsTest()
{
  while (1)
  {
    batmobile.heading = (int)readSensor(SENSE_HEADING);
    batmobile.gpsLat = readSensor(SENSE_LAT) / (double)100000;
    batmobile.gpsLong = readSensor(SENSE_LONG) / (double)100000;
    batmobile.gpsHDOP = readSensor(SENSE_HDOP);
    batmobile.gpsAge = readSensor(SENSE_AGE);
    batmobile.gpsSIV = readSensor(SENSE_SIV);

    //Print out values of each axis
    SerialUSB.print("heading[");
    SerialUSB.print(batmobile.heading);
    SerialUSB.print("]gpsLat[");
    SerialUSB.print(batmobile.gpsLat, 5);
    SerialUSB.print("]gpsLong[");
    SerialUSB.print(batmobile.gpsLong, 5);
    SerialUSB.print("]gpsSIV[");
    SerialUSB.print(batmobile.gpsSIV);
    SerialUSB.print("]gpsHDOP[");
    SerialUSB.print(batmobile.gpsHDOP, 2);
    SerialUSB.print("]gpsAge[");
    SerialUSB.print(batmobile.gpsAge, 0);
    SerialUSB.print("]");
    SerialUSB.println();

    dogDelay(250);

  }
}

