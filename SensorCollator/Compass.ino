/*
 This contains all the functions to read and setup the compass
*/

//Configures the compass for continuous readings
void setupCompass()
{
  SerialUSB.println("Initialize HMC5883L");

  byte tries = 0;
  while (!compass.begin())
  {
    SerialUSB.println("Could not find a valid HMC5883L sensor, check wiring!");
    if (tries++ > 3) return; //Give up
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  //compass.setDataRate(HMC5883L_DATARATE_15HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  //compass.setSamples(HMC5883L_SAMPLES_4);

  // Set calibration offset. See HMC5883L_calibration.ino
  //I think this comes from _calibrate.ino, the last two numbers around the :
  compass.setOffset(-39, 3); //Original
  //compass.setOffset(-94, 7); //offX, offY Update on 9/5/16
}

//When called, reads current heading into SOTW
void readHeading(void)
{
  //During compass reading ignore the main I2C interrupts
  //Wire.onReceive(dummyreceiveEvent);
  //Wire.onRequest(dummyrequestEvent);

  pingCompass();
  
  Vector norm = compass.readNormalize();

  //Assign interrupts back to the main handlers
  //Wire.onReceive(receiveEvent); //When we get I2C bytes, call this function
  //Wire.onRequest(requestEvent); //When master is asking for something, call this function

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  //float declinationAngle = (8.0 + (28.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180 / M_PI;

  //Correct reverse readings
  headingDegrees = 360 - headingDegrees;

  //Correct for offset
  //For some reason my compass reading doesn't point north.
  //This offset was found using cell phone from desk indoors
  //Around lots of computers. Seems to work superbly outside.
  headingDegrees = headingDegrees - 82;
  if (headingDegrees < 0) headingDegrees += 360;

  SerialUSB.print("^");
  //return (headingDegrees);
  //currentVector.heading = headingDegrees;
  compassHeading = headingDegrees;
  //SerialUSB.print("Heading: ");
  //SerialUSB.println(compassHeading);

}



void testCompass()
{
  SerialUSB.println("Compass test");

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

  TwoWire testI2C(&sercom1, 11, 13); //SDA = 11, SCL = 13

  testI2C.begin();

  //Put the HMC5883 IC into the correct operating mode
  testI2C.beginTransmission(address); //open communication with HMC5883
  testI2C.write(0x02); //select mode register
  testI2C.write((uint8_t)(0x00)); //continuous measurement mode
  testI2C.endTransmission();

  int x, y, z; //triple axis data

  while (1)
  {
    //Tell the HMC5883L where to begin reading data
    testI2C.beginTransmission(address);
    testI2C.write(0x03); //select register 3, X MSB register
    testI2C.endTransmission();

    //Read data from each axis, 2 registers per axis
    testI2C.requestFrom(address, 6);
    if (6 <= testI2C.available())
    {
      x = testI2C.read() << 8; //X msb
      x |= testI2C.read(); //X lsb
      z = testI2C.read() << 8; //Z msb
      z |= testI2C.read(); //Z lsb
      y = testI2C.read() << 8; //Y msb
      y |= testI2C.read(); //Y lsb
    }

    //Print out values of each axis
    SerialUSB.print("x: ");
    SerialUSB.print(x);
    SerialUSB.print("  y: ");
    SerialUSB.print(y);
    SerialUSB.print("  z: ");
    SerialUSB.println(z);

    delay(250);
  }
}

//Used to unlock I2C comm when it freezes
void pingCompass()
{

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

  TwoWire testI2C(&sercom1, 11, 13); //SDA = 11, SCL = 13

  testI2C.begin();

  //Put the HMC5883 IC into the correct operating mode
  testI2C.beginTransmission(address); //open communication with HMC5883
  testI2C.write(0x02); //select mode register
  testI2C.write((uint8_t)(0x00)); //continuous measurement mode
  testI2C.endTransmission();

  int x, y, z; //triple axis data

  //Tell the HMC5883L where to begin reading data
  testI2C.beginTransmission(address);
  testI2C.write(0x03); //select register 3, X MSB register
  testI2C.endTransmission();

  //Read data from each axis, 2 registers per axis
  testI2C.requestFrom(address, 6);
  if (6 <= testI2C.available())
  {
    x = testI2C.read() << 8; //X msb
    x |= testI2C.read(); //X lsb
    z = testI2C.read() << 8; //Z msb
    z |= testI2C.read(); //Z lsb
    y = testI2C.read() << 8; //Y msb
    y |= testI2C.read(); //Y lsb
  }

}


