/*
  These are all the functions that talk to the sub boards and receive sensor data and
  send out control commands
*/

//Read a given addr from the locomotion controller
int readLocomotion(byte addr)
{
  Wire.beginTransmission(LOCOMOTION_ADDRESS);
  Wire.write(addr); //Set 'memory address' to whatever the caller tells us
  Wire.endTransmission();

  delay(10); //Required for Pro Mini

  //Read the data at this address
  if (Wire.requestFrom(LOCOMOTION_ADDRESS, 2) == 2)
  {
    byte chunk1 = Wire.read();
    byte chunk2 = Wire.read();
    return ((int)chunk2 << 8 | chunk1);
  }
  else
  {
    dog.reset(); //Pet the dog
    SerialUSB.println("Motor failed to respond.");
    return (-1);
  }
}

//Send a value to a spot on the locomotion controller
void writeLocomotion(byte addr, int value)
{
  Wire.beginTransmission(LOCOMOTION_ADDRESS);
  Wire.write(addr); //Spot 4 is requestedPosition

  delay(10);

  //This is really screwy. We have to split the values into an array and send the array
  byte data[2];
  data[0] = value;
  data[1] = value >> 8;
  Wire.write(data, 2);

  Wire.endTransmission();
}


