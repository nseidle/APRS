/*
  Laser Control - Read from three laser tape measures and present data over I2C to maser controller
  Nathan Seidle
  SparkFun Electronics
  8/21/2016

  I'm going to use a SAMD21 mini breakout to connect to three laser modules at the same time

  Unit responds to I2C commands in the following way:
  0x00: Send all 6 datums in order, 4*6 = 24 bytes
  0x01: distance 1 in mm (long, four bytes)
  0x02: distance 2
  0x03: distance 3
  0x04: age of reading 1 in ms (long, four bytes)
  0x05: age of reading 2
  0x06: age of reading 3

  Todo:

  Minimum read distance is about 0.05m. Anything less than that and the laser will shine but
  the module will not output a reading (times outs).

  Variance between readings is +/-1mm. Amazing.

  Average read time is 360ms but varies widely based on surface reflectivity. It can be as much as 2s.
  If the surface is reflective, glossy, or opaque the laser will time out

  Connections:
  13: Status LED
  11: Software serial output to Laser RX (ylw wire)
  (not used because software serial can't reliably do 115200) 10: Software serial input from Laser TX (blu wire)
  0: Serial input from Laser TX (blu wire)
  5V to Laser module (red wire) (Laser module has a 3.3V regulator on board. Should be able to handle 5-10V)
  GND to Laser module (black wire)

  Laser1 on Serial1:
  Mini 0 = RX from laser, gray to blue to green to TX on laser
  Mini 1 = TX to laser, yellow to yellow to yellow to RX on laser

  Laser2 on Serial2:
  Mini 11 = RX from laser, gray to blue to green to TX on laser
  Mini 10 = TX to laser, yellow to yellow to yellow to RX on laser

  Laser3 on Serial3:
  Mini 5 = RX from laser, gray to blue to green to TX on laser
  Mini 2 = TX to laser, yellow to yellow to yellow to RX on laser

  I2C Connections:
  SDA: Gray to Blue on bus
  SCL: Yellow to Green on Bus
  
*/

#include <Arduino.h>   //Required before wiring_private.h
#include "wiring_private.h" //Needed for pinPeripheral() function
#include <Wire.h> //I2C Arduino Library

#include <LaserDistanceModule.h> //Library for all the laser communcation

#include <SimpleTimer.h> //https://github.com/jfturcot/SimpleTimer
SimpleTimer timer; //Used to blink LED and other time based things

#define STAT 13 //Status LED on most Arduino boards

//The SAMD21 Arduino implementation shifts addresses 
//0x0D is locomotion controller
//0x0E is Sensor Collator
//0x0F is Laser Collator
//0x1E is HMC5883L magneto

#define LOCOMOTION_ADDRESS 0x0D
#define SENSORCOLLATOR_ADDRESS 0x0E
#define LASERCOLLATOR_ADDRESS 0x0F

//The SERCOM_RX_PAD_0 and UART_TX_PAD_2 come
//from SERCOM table: https://learn.sparkfun.com/tutorials/samd21-minidev-breakout-hookup-guide/all#samd21-overview
//Example: Uart laserSerial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
//Find a pin you want as RX under sercom1. Now scan left and find the pad#. That becomes SERCOM_RX_PAD_blah
//Find a pin you want as TX under sercom1. Now scan left and find the pad#. That becomes UART_TX_PAD_blag

//RX on 11, TX on 10
Uart laserSerial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  laserSerial2.IrqHandler();
}

//RX on 5, TX on 2
Uart laserSerial3 (&sercom2, 5, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM2_Handler()
{
  laserSerial3.IrqHandler();
}

//Connect the lasers to their serial streams
laserRanger laser1(Serial1);
laserRanger laser2(laserSerial2);
laserRanger laser3(laserSerial3);

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
boolean statLEDon = false; //Keep track of the state of the status LED because there is MOSFET controlling it

byte whichSetting = 0; //This is the pseudo 'memory address' that the master will set to read/write I2C stuff

long secondTimerID;
long safetyMonitor1_ID, safetyMonitor2_ID, safetyMonitor3_ID; //These are the IDs for the safe monitor timers
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  pinMode(STAT, OUTPUT);
  secondTimerID = timer.setInterval(1000L, secondPassed); //Call secondPassed every Xms

  Wire.begin(LASERCOLLATOR_ADDRESS); //Join I2C bus as slave at this address
  Wire.onReceive(receiveEvent); //When we get I2C bytes, call this function
  Wire.onRequest(requestEvent); //When master is asking for something, call this function

  //This is a safety mechanism: If we don't get a reading after 3 seconds, re-issue multiRead command
  //The timer is reset every time we have a successful distance read
  safetyMonitor1_ID = timer.setInterval(3000L, issueMultiRead1); //We have individual issueMultiReads because the timer library can't pass variables
  safetyMonitor2_ID = timer.setInterval(3000L, issueMultiRead2);
  safetyMonitor3_ID = timer.setInterval(3000L, issueMultiRead3);

  SerialUSB.begin(115200);

  //The SparkFun laser distance module runs at 115200bps
  Serial1.begin(115200); //Connection to Laser module #1
  laserSerial2.begin(115200); //Connection to Laser module #2
  laserSerial3.begin(115200); //Connection to Laser module #3

  pinPeripheral(2, PIO_SERCOM);
  pinPeripheral(5, PIO_SERCOM);
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  startLaser(1);
  startLaser(2);
  startLaser(3);

  issueMultiRead1;
  issueMultiRead2;
  issueMultiRead3;
}

void loop()
{
  timer.run(); //Update any timers we are running
  laser1.check(); //Polls serial port connected to this laser for any new characters
  laser2.check();
  laser3.check();

  //Only print status if a measurement changed
  if (laser1.measurement.newMeasurement == true)// || measurement2.newMeasurement == true || measurement3.newMeasurement == true)
  {
    //TODO reset safety timer on a given channel
    //We could maybe get this inside the library but tricky
    if (laser1.measurement.newMeasurement == true) timer.restartTimer(safetyMonitor1_ID); //Reset the safety timer for this channel
    if (laser2.measurement.newMeasurement == true) timer.restartTimer(safetyMonitor2_ID); //Reset the safety timer for this channel
    if (laser3.measurement.newMeasurement == true) timer.restartTimer(safetyMonitor3_ID); //Reset the safety timer for this channel

    laser1.measurement.newMeasurement = false;
    laser2.measurement.newMeasurement = false;
    laser3.measurement.newMeasurement = false;

    long age1 = millis() - laser1.measurement.time;
    long age2 = millis() - laser2.measurement.time;
    long age3 = millis() - laser3.measurement.time;
    long avgAge = (age1 + age2 + age3) / 3;

    SerialUSB.print("1[");
    SerialUSB.print(laser1.measurement.currentDistance);
    SerialUSB.print("mm ");
    SerialUSB.print(age1);
    SerialUSB.print("ms]");

    SerialUSB.print(" 2[");
    SerialUSB.print(laser2.measurement.currentDistance);
    SerialUSB.print("mm ");
    SerialUSB.print(age2);
    SerialUSB.print("ms]");

    SerialUSB.print(" 3[");
    SerialUSB.print(laser3.measurement.currentDistance);
    SerialUSB.print("mm ");
    SerialUSB.print(age3);
    SerialUSB.print("ms]");

    SerialUSB.print(" - AvgAge[");
    SerialUSB.print(avgAge);
    SerialUSB.print("]");

    SerialUSB.println();

  }

  delay(1);
}

void startLaser(byte laserNumber)
{
  bool response;
  if (laserNumber == 1) response = laser1.begin();
  if (laserNumber == 2) response = laser2.begin();
  if (laserNumber == 3) response = laser3.begin();

  if (response == true)
  {
    SerialUSB.print("Laser ");
    SerialUSB.print(laserNumber);
    SerialUSB.println(" online");
  }
  else
  {
    SerialUSB.print("Laser ");
    SerialUSB.print(laserNumber);
    SerialUSB.println(" failed to respond");
  }
}

//Send command to lasers to take multiread
//We have individual issueMultiReads because the timer library can't pass variables
void issueMultiRead1()
{
  SerialUSB.println("Issuing multiread");
  laser1.startMultiRead(); //Tell laser to start reading again
}
void issueMultiRead2()
{
  laser2.startMultiRead(); //Tell laser to start reading again
}
void issueMultiRead3()
{
  laser3.startMultiRead(); //Tell laser to start reading again
}

//Blink the status LED to show we are alive
void secondPassed()
{
  if (statLEDon == false)
  {
    digitalWrite(STAT, HIGH);
    statLEDon = true;
  }
  else
  {
    digitalWrite(STAT, LOW);
    statLEDon = false;
  }
}

