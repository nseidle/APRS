/*
 Sensor Collator - Collects a variety of sensor data and presents it over I2C
 Nathan Seidle
 SparkFun Electronics
 3/23/2016

 This sensing board reads GPS, compass, and distance sensors.

 We read the GPS incoming stream over software serial at 57600bps. We read the compass over I2C (should use 3.3V).

 GPS looks like this:
 4001.07977,N,10516.95624,W

 We only care about and store only the last 5 digits. These are the 1/60 fraction of the minute section.
 A float will not hold 5 decimal places but a double will. Because of this we use a SAMD21 instead of a proMini.

 v1.6.5 seems to work on home computer for SAMD21 (softwareserial lib issue).

 Connections:
  13: Status LED - This is bad because it conflicts with SCL
  8: Pro mini output to GPS RX (ylw wire) NOT USED
  5: Pro Mini input to GPS TX (blu wire)
  13: Software SCL (red wire) to compass
  11: Software SDA (blk wire) to compass
  3.3V to GPS (red wire) (GPS module has a 3.3V regulator on board. Should be able to handle 5-10V)
  GND to GPS (black wire)

 Master can read 8 types of data:
    0: Heading (0.0 to 359.9)
    1: Lat
    2: Long
    3: Altitude
    4: HDOP
    5: Sats used for position
    6:

 Ublox Configuration:
    Baud Rate: 57600 bps
    Rate: Set to 200ms for measurement period (5Hz)
    Sentences that we need turned on (MSG menu): GGA for lat, long, HDOP, altitude, sats used, time
    Sentences to turn off:
      GSA needed only for PDOP: (multiple, one for each satellite) for PDOP, HDOP, VDOP
      GSV: Satellite info
      GLL: Lat, long, time. Duplicate of GGA
      RMC: Lat, long, time
      VTG: Speed info
   Once GPS is configured as you want it, save settings by: Receiver->Action->Save Config

 This board acts as a slave on I2C bus.
 To communicate with the compass as master we use a SoftwareWire library and hack the HMC library
 to use it.

  I2C Connections:
  SDA: Blue on bus
  SCL: Green on Bus, yellow on cables

*/

#include <SimpleTimer.h> //https://github.com/jfturcot/SimpleTimer
#include <Wire.h> //I2C Arduino Library
#include <HMC5883L.h> //From https://github.com/jarzebski/Arduino-HMC5883L

#include "wiring_private.h" //Needed for pinPeripheral() function

HMC5883L compass;

//Uncomment the following line to show the incoming GPS serial stream
//Used for debugging
#define SHOW_GPS

//#define STAT 13 //Bad - LED is on I2C line 13

//0x0D is locomotion controller
//0x0E is Sensor Collator
//0x0F is Laser Collator
//0x1E is HMC5883L magneto

#define LOCOMOTION_ADDRESS 0x0D
#define SENSOR_COLLATOR_ADDRESS 0x0E
#define LASER_COLLATOR_ADDRESS 0x0F

SimpleTimer timer;
long secondTimerID;
long compassTimerID;

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
boolean statLEDon = false; //Keep track of the state of the status LED because there is MOSFET controlling it
boolean newInfo = false; //Print status update when new stuff is available

byte whichSetting = 0; //This is the pseudo 'memory address' that the master will set to read/write I2C stuff

volatile double gpsLat;
volatile double gpsLong;
volatile float gpsAge;
volatile float gpsHDOP;
volatile float gpsAltitude;
volatile int gpsSIV;
volatile boolean gpsChecksum;
volatile int compassHeading;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  SerialUSB.begin(115200);

  //pinPeripheral(11, PIO_SERCOM_ALT);
  //pinPeripheral(13, PIO_SERCOM_ALT);

  Wire.begin(SENSOR_COLLATOR_ADDRESS); //Join I2C bus as slave at this address
  Wire.onReceive(receiveEvent); //When we get I2C bytes, call this function
  Wire.onRequest(requestEvent); //When master is asking for something, call this function

  //We will get stuck here if USB cable is not attached.
  //Uncomment for debug/testing
  //while (!SerialUSB); //Magic code that waits for serial port to be allowed to print

  //testCompass();
  setupCompass();
  setupGPS();

  //I'm having problems trusting that a timer will always run
  //Stopped working on locomotion controller
  //Set a timer to check the compass at 4Hz
  compassTimerID = timer.setInterval(200, readHeading);

  SerialUSB.println("Sensor Collator Online");
}

void loop()
{
  timer.run(); //Update any timers we are running
  checkGPS();

  //GPS and compass are checked regularly via timer firing

  if (newInfo == true)
  //if (0) //Testing
  {
    newInfo = false;

    SerialUSB.print("Degrees[");
    SerialUSB.print(compassHeading);
    SerialUSB.print("]Lat[");
    SerialUSB.print(gpsLat / (double)100000, 5);
    SerialUSB.print("]Long[");
    SerialUSB.print(gpsLong / (double)100000, 5);
    SerialUSB.print("]SIV[");
    SerialUSB.print(gpsSIV);
    SerialUSB.print("]HDOP[");
    SerialUSB.print(gpsHDOP);
    //SerialUSB.print("]Alt(ft)[");
    //SerialUSB.print(gpsAltitude * 3.28);
    SerialUSB.print("]Age[");
    SerialUSB.print(millis() - gpsAge);
    SerialUSB.print("]");

    SerialUSB.println();
  }
}

