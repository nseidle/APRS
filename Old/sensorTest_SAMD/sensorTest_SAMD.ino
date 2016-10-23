#include <Wire.h> //I2C Arduino Library

#define SENSOR_COLLATOR_ADDRESS 0x0E //Address of sensor collator

#define SENSE_HEADING 0
#define SENSE_LAT 1
#define SENSE_LONG 2
#define SENSE_ALT 3
#define SENSE_HDOP 4
#define SENSE_SIV 5
#define SENSE_AGE 6

volatile double gpsLat;
volatile double gpsLong;
volatile float gpsAge;
volatile float gpsHDOP;
volatile float gpsAltitude;
volatile int gpsSIV;
volatile boolean gpsChecksum;
volatile int compassHeading;

volatile int rawHeading;

//We're not sure how the compass will be stuck to car. Use cell phone to figure out offset from N.
int compassOffset = 267; //61, Add a number that is the difference between cell phone N and what nav says is N

void setup()
{
  SerialUSB.begin(115200);
  //while(!SerialUSB);

  SerialUSB.println("SAMD Test sensor");

  Wire.begin();
}

void loop()
{
  compassHeading = (int)readSensor(SENSE_HEADING);
  gpsLong = readSensor(SENSE_LONG) / (double)100000;
  gpsHDOP = readSensor(SENSE_HDOP);
  gpsLat = readSensor(SENSE_LAT) / (double)100000;
  gpsAge = readSensor(SENSE_AGE);
  gpsSIV = readSensor(SENSE_SIV);

  //Print out values of each axis
  SerialUSB.print("raw heading[");
  SerialUSB.print(rawHeading);
  SerialUSB.print("]adjusted head[");
  SerialUSB.print(compassHeading);
  SerialUSB.print("]gpsLat[");
  SerialUSB.print(gpsLat, 5);
  SerialUSB.print("]gpsLong[");
  SerialUSB.print(gpsLong, 5);
  SerialUSB.print("]gpsSIV[");
  SerialUSB.print(gpsSIV);
  SerialUSB.print("]gpsHDOP[");
  SerialUSB.print(gpsHDOP, 2);
  SerialUSB.print("]gpsAge[");
  SerialUSB.print(gpsAge, 0);
  SerialUSB.print("]");
  SerialUSB.println();

  delay(250);

  if(SerialUSB.available())
  {
    char incoming = SerialUSB.read();

    if(incoming == 'a')
    {
      headingOffset += 10;
    }
    else if (incoming == 'z')
    {
      headingOffset -= 10;
    }
    else if(incoming == 's')
    {
      headingOffset += 1;
    }
    else if(incoming == 'x')
    {
      headingOffset -= 1;
    }

    if(headingOffset > 360) headingOffset -= 360;
    if(headingOffset < 0) headingOffset += 360;
  }
}

//Read a given addr from the sensor controller
float readSensor(byte addr)
{
  Wire.beginTransmission(SENSOR_COLLATOR_ADDRESS);
  Wire.write(addr); //Set 'memory address' to whatever the caller tells us
  Wire.endTransmission(); //Removed to work on SAMD21

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
    if (addr == SENSE_HEADING)
    {
      rawHeading = value; //Save for later
      //Correct heading for whatever orientation it's in
      value -= compassOffset;
      if (value < 0) value += 360;
    }


    return (value);
  }
  else
  {
    SerialUSB.println("GPS failed to respond.");
    return (-1);
  }
}


