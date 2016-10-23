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

#define SERIAL_TYPE SerialUSB
//#define SERIAL_TYPE Serial


void setup()
{
  SERIAL_TYPE.begin(115200);

  SERIAL_TYPE.println("Test sensor");

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
  SERIAL_TYPE.print("heading[");
  SERIAL_TYPE.print(compassHeading);
  SERIAL_TYPE.print("]gpsLat[");
  SERIAL_TYPE.print(gpsLat, 5);
  SERIAL_TYPE.print("]gpsLong[");
  SERIAL_TYPE.print(gpsLong, 5);
  SERIAL_TYPE.print("]gpsSIV[");
  SERIAL_TYPE.print(gpsSIV);
  SERIAL_TYPE.print("]gpsHDOP[");
  SERIAL_TYPE.print(gpsHDOP, 2);
  SERIAL_TYPE.print("]gpsAge[");
  SERIAL_TYPE.print(gpsAge);
  SERIAL_TYPE.print("]");
  SERIAL_TYPE.println();

  //delay(25);
}

//Read a given addr from the sensor controller
float readSensor(byte addr)
{
  Wire.beginTransmission(SENSOR_COLLATOR_ADDRESS);
  Wire.write(addr); //Set 'memory address' to whatever the caller tells us
  Wire.endTransmission(); //Removed to work on SAMD21

  delay(10);

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

    return (value);
  }
  else
  {
    SERIAL_TYPE.println("GPS failed to respond.");
    return (-1);
  }
}


