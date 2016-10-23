/*
  The Brain - Master Controller for AVC/PRS Vehicle
  Nathan Seidle
  SparkFun Electronics
  3/27/2016

  Master controller sends bytes to the steering controller and read values from
  locomotion controller, sensor collator, and laser collator

  From all the sensor information we make the navigation decisions

  I2C Connections:
  SDA: Blue on bus
  SCL: Green on Bus
*/

#include <SimpleTimer.h> //https://github.com/jfturcot/SimpleTimer
#include <Wire.h> //Needed for I2C slave communication

#define STAT 13

SimpleTimer timer;
long secondTimerID;

//Number of milliseconds between
#define CHECKIN_PERIOD 100L

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
boolean statLEDon = false; //Keep track of the state of the status LED because there is MOSFET controlling it

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//0x0D is locomotion controller
//0x0E is Sensor Collator
//0x0F is Laser Collator
//0x1E is HMC5883L magneto

#define LOCOMOTION_ADDRESS 0x0D
#define SENSOR_COLLATOR_ADDRESS 0x0E
#define LASER_COLLATOR_ADDRESS 0x0F
#define DISPLAY_ADDRESS1 0x72 //This is the default address of the OpenLCD

int cycles = 0;

void setup()
{
  pinMode(STAT, OUTPUT);

  SerialUSB.begin(115200);
  //while(!SerialUSB);

  secondTimerID = timer.setInterval(1000L, secondPassed); //Call secondPassed every Xms

  SerialUSB.println("LCD Testing");

  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
}

void loop()
{
  timer.run(); //Update any timers we are running
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


