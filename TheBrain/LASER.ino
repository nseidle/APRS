
//Read a given addr from the distance controller
unsigned long readDistance(byte addr)
{
  Wire.beginTransmission(LASER_COLLATOR_ADDRESS);
  Wire.write(addr); //Set 'memory address' to whatever the caller tells us
  Wire.endTransmission();

  //delay(10);

  //Read data from this device
  //Data from distance collator are 4 bytes
  if (Wire.requestFrom(LASER_COLLATOR_ADDRESS, 4) == 4)
  {
    byte data[sizeof(unsigned long)];
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();

    unsigned long value;
    memcpy(&value, data, sizeof value);
    return (value);
  }
  else
  {
    dog.reset(); //Pet the dog
    SerialUSB.println("Lasers failed to respond.");
    return (-1);
  }
}

