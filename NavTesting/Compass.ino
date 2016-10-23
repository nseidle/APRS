/*
 This contains all the functions to read and setup the compass
*/

//When called, reads current heading into SOTW
void readHeading(void)
{
  Vector norm = compass.readNormalize();

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
  if(headingDegrees < 0) headingDegrees += 360;

  //return (headingDegrees);
  batmobile.heading = headingDegrees;  
}

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

  //Set a timer to check the compass at 4Hz
  compassTimerID = timer.setInterval(250, readHeading);

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
  //compass.setOffset(-39, 3); //Original
  compass.setOffset(-94, 7); //offX, offY
}


