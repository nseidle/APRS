/*
  Nav Testing - Master Controller for AVC/PRS Vehicle
  Nathan Seidle
  SparkFun Electronics
  9/8/2016

  Let's test our angle math and turn left/right logic.

  GPS looks like this:
  4001.07977,N,10516.95624,W

  We only care about and store only the last 5 digits. These are the 1/60 fraction of the minute section.
  A float will not hold 5 decimal places but a double will. Because of this we use a SAMD21 instead of a proMini.

  Assume we will receive a string from the sensor collator.

  Main Bus I2C Connections:
  SDA: Blue on bus
  SCL: Green on Bus

 Connections:
  13: Status LED
  8: Pro mini output to GPS RX (ylw wire) NOT USED
  5: Pro Mini input to GPS TX (blu wire)
  13: Software SCL (red wire) to compass
  11: Software SDA (blk wire) to compass
  3.3V to GPS (red wire) (GPS module has a 3.3V regulator on board. Should be able to handle 5-10V)
  GND to GPS (black wire)
  
*/

#include <SimpleTimer.h> //https://github.com/jfturcot/SimpleTimer
#include <Wire.h> //Needed for I2C slave communication
#include <HMC5883L.h> //From https://github.com/jarzebski/Arduino-HMC5883L

#include "wiring_private.h" // pinPeripheral() function

HMC5883L compass;

//Uncomment the following line to show the incoming GPS serial stream
//Used for debugging
//#define SHOW_GPS

//#define STAT 13 //Bad - LED is on I2C line 13
#define STAT 10 //Temp, doesn't work

SimpleTimer timer;
long secondTimerID;
long gpsTimerID;
long compassTimerID;
long navigateTimerID; //Keeps track of when we should update heading and turning

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
boolean statLEDon = false; //Keep track of the state of the status LED because there is MOSFET controlling it

int steeringPosition;
int throttlePosition;

struct SOTW //State of the World
{
  double gpsLat;
  double gpsLong;
  long gpsAge;
  float gpsHDOP;
  byte gpsSIV;
  int heading;

  int settingAngleStraight;
  int settingAngleMax;
  int settingAngleMin;
  int settingPotDelta;
  int requestedSteeringPosition;
  int currentSteeringPosition;
  int motorSpeed;
  int throttlePosition;
  int systemMode;

  long leftDistance;
  long rightDistance;
  long centerDistance;
  long leftAge;
  long rightAge;
  long centerAge;
};
SOTW batmobile; //This is the total state of the world for the batmobile

struct Coordinates
{
  double gpsLong;
  double gpsLat;
};

struct Coordinates wayPoint[10]; //These are our stored contest waypoints
int currentWP = 0; //This is our waypoint token. Start at zero.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Settings
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int cruisingSpeedForAuto = 15; //15%? Maybe?

unsigned int left_wall_min_distance = 750; //750mm to wall before we start to steer away
unsigned int right_wall_min_distance = 750; //750mm to wall before we start to steer away

//We want a 2.5m halo. If we are within 8.2 of the spot, we're close enough.
double halo_long_delta = 8.2/101/(double)60; //1" = 101ft, 2.5m = 8.2ft
//8.2/101 / 60 = 0.0013531353135
double halo_lat_delta = 0.1/(double)60;  //1" = ~80ft, 2.5m = 8.2ft = 0.1"
//0.1/60 = 0.0016666666

//These were found by adjusting wheels and taking readings
unsigned int a_little_right = 561;
unsigned int a_lot_right = 551;
unsigned int a_little_left = 565;
unsigned int a_lot_left = 575;

long left_wall_time_before_a_lot_turn = 1000; //Amount of time (turning a little) before we give up and turn a lot
long right_wall_time_before_a_lot_turn = 1000; 

long wall_left_timestamp; //Record when we saw a wall so that we can go hard away if needed
long wall_right_timestamp;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  pinMode(STAT, OUTPUT);

  SerialUSB.begin(115200);

  pinPeripheral(11, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM_ALT);

  secondTimerID = timer.setInterval(1000, secondPassed); //Call secondPassed every Xms

  while (!SerialUSB); //Magic code that waits for serial port to be allowed to print

  SerialUSB.println("Nav Test!");

  setupCompass();
  setupGPS();

  SerialUSB.println("Sensors Online");

  //Load waypoints
  //We store only the 0.xxxxx digits from the GPS module
  wayPoint[0].gpsLat = 0.45358;
  wayPoint[0].gpsLong = 0.10902;
  //wayPoint[0].gpsLat = 28.00 / (double)60;
  //wayPoint[0].gpsLong = 5.64 / (double)60;
  wayPoint[1].gpsLat = 27.88 / (double)60;
  wayPoint[1].gpsLong = 5.39 / (double)60;
  wayPoint[2].gpsLat = 27.64 / (double)60;
  wayPoint[2].gpsLong = 5.68 / (double)60;
  wayPoint[3].gpsLat = 27.70 / (double)60;
  wayPoint[3].gpsLong = 5.95 / (double)60;

  SerialUSB.print("Waypoint: ");
  SerialUSB.print(wayPoint[0].gpsLat, 5);
  SerialUSB.print(" ");
  SerialUSB.print(wayPoint[0].gpsLong, 5);
  SerialUSB.println();

  //Float test
  /*String strTest1 = "0.1234567";
  String strTest2 = "4001.3465900";

  
  double test1 = (double)strTest1.toFloat();
  double test2 = (double)strTest2.toFloat();
  
  SerialUSB.print("Float test: ");
  SerialUSB.print(test1, 7);
  SerialUSB.print(" ");
  SerialUSB.print(test2, 7);
  SerialUSB.println();

  while(1);*/

  //Test current waypoints
  
  //Bat 276, left
  batmobile.gpsLat = 27.69 / (double)60;
  batmobile.gpsLong = 3.58 / (double)60;

  //Test 3 = 261, left
  //batmobile.gpsLat = 28.16 / (double)60;
  //batmobile.gpsLong = 3.57 / (double)60;

  //Test 2 = 146, right
  //batmobile.gpsLat = 28.53 / (double)60;
  //batmobile.gpsLong = 5.83 / (double)60;

  //Bat 2 = 202, left
  batmobile.gpsLat = 28.71 / (double)60;
  batmobile.gpsLong = 5.05 / (double)60;

  //Bat 3 = 99, right
  //batmobile.gpsLat = 28.16 / (double)60;
  //batmobile.gpsLong = 7.19 / (double)60;

  //Bat 4 = 80, right
  //batmobile.gpsLat = 27.65 / (double)60;
  //batmobile.gpsLong = 6.80 / (double)60;

  //Test 4 = 324, left
  //batmobile.gpsLat = 27.26 / (double)60;
  //batmobile.gpsLong = 4.93 / (double)60;

  //Test 1 = 34, right
  //batmobile.gpsLat = 27.24 / (double)60;
  //batmobile.gpsLong = 5.83 / (double)60;

  batmobile.heading = 22;

  //Let's start navigating!
  navigateTimerID = timer.setInterval(500, navigate);
}

void loop()
{
  timer.run(); //Update any timers we are running

  if (SerialUSB.available())
  {
    char incoming = SerialUSB.read();

    SerialUSB.println("1: Requesting steering to 35");

    if (incoming == '1')
    {
      SerialUSB.println("Requesting steering to 35");

    }
    else if (incoming == '2')
    {
      SerialUSB.println("Writing motor speed to 17");

    }
  }
}

void navigate()
{

  if(batmobile.gpsLat == -1)
  {
    SerialUSB.println("Waiting for GPS");
    return;
  }
  
  //Calculate our new desired heading
  SerialUSB.print("WP[");
  SerialUSB.print(currentWP);
  SerialUSB.print("]wpLL[");
  SerialUSB.print(wayPoint[currentWP].gpsLat, 5);
  SerialUSB.print("/");
  SerialUSB.print(wayPoint[currentWP].gpsLong, 5);
  SerialUSB.print("] curLL[");
  SerialUSB.print(batmobile.gpsLat, 5);
  SerialUSB.print("/");
  SerialUSB.print(batmobile.gpsLong, 5);
  SerialUSB.print("]SIV[");
  SerialUSB.print(batmobile.gpsSIV, DEC);
  SerialUSB.print("]HDOP[");
  SerialUSB.print(batmobile.gpsHDOP, 2);
  SerialUSB.print("]head[");

  //Fix heading
  //batmobile.heading += 40; //Correction value found from compass on phone
  //if(batmobile.heading > 360) batmobile.heading -= 360;
  
  SerialUSB.print(batmobile.heading, DEC);
  SerialUSB.print("]");

  double xDelta = batmobile.gpsLong - wayPoint[currentWP].gpsLong;
  double yDelta = batmobile.gpsLat - wayPoint[currentWP].gpsLat;

  //Calc our distance to the next WP
  double distanceToWP = (xDelta * xDelta + yDelta * yDelta);
  distanceToWP = sqrt(distanceToWP);
  //Convert distance to feet
  //1" = ~80ft long and 1" = 101ft lat so let's say 1" = 100ft
  //0.1" = 10ft.
  //0.00143 = 10 * 0.00143 / .1 = 0.143ft
  //0.00976 = ~8ft. = 0.976ft
  distanceToWP *= 1000;

  //Determine if we are in the halo of the wayPoint
  if(abs(xDelta) < halo_long_delta && abs(yDelta) < halo_lat_delta)
  {
    SerialUSB.print(" Waypoint! ");
  }
  /*while(abs(xDelta) < halo_long_delta && abs(yDelta) < halo_lat_delta)
  {
    SerialUSB.println();
    SerialUSB.println();
    SerialUSB.print(" Waypoint [");
    SerialUSB.print(currentWP);
    SerialUSB.print("] achieved! Going to next!");
    
    currentWP++;
    
    xDelta = batmobile.gpsLong - wayPoint[currentWP].gpsLong;
    yDelta = batmobile.gpsLat - wayPoint[currentWP].gpsLat;
  }*/

  /*SerialUSB.print(" xDelta: ");
  SerialUSB.print(xDelta, 5);
  SerialUSB.print(" yDelta: ");
  SerialUSB.print(yDelta, 5);
  SerialUSB.println();*/

  double ratio = xDelta / yDelta;

  /*SerialUSB.print(" ratio: ");
  SerialUSB.print(ratio);*/

  double alpha = atan(ratio); //Find the angle in the corner of the triangle
  
  /*SerialUSB.print(" alphaRad: ");
  SerialUSB.print(alpha);*/

  alpha *= (double)180 / (double)PI; //Convert to degrees

  /*SerialUSB.print(" alphaDegrees: ");
  SerialUSB.print(alpha);*/

  int desiredHeading; //This is the heading we want to achieve. Also called bearing.

  if (xDelta >= 0 && yDelta >= 0) desiredHeading = 180 - (int)alpha;
  else if (xDelta >= 0 && yDelta < 0) desiredHeading = (int)alpha * -1;
  else if (xDelta < 0 && yDelta < 0) desiredHeading = 360 - (int)alpha;
  else if (xDelta < 0 && yDelta >= 0) desiredHeading = 180 - (int)alpha;

  SerialUSB.print("headWP[");
  SerialUSB.print(desiredHeading);
  SerialUSB.print("]");

  //Decide which way to turn based on current heading and desired heading
  #define TURN_LEFT 1
  #define TURN_RIGHT 2
  #define TURN_NONE 3
  byte turnDirection;
  int headingDelta;

  if (batmobile.heading >= desiredHeading)
  {
    if (batmobile.heading - desiredHeading < 180)
    {
      turnDirection = TURN_LEFT;
      headingDelta = batmobile.heading - desiredHeading;
    }
    if (batmobile.heading - desiredHeading >= 180)
    {
      turnDirection = TURN_RIGHT;
      headingDelta = 360 - batmobile.heading + desiredHeading;
    }
  }
  else if (batmobile.heading < desiredHeading)
  {
    if(desiredHeading - batmobile.heading >= 180)
    {
      turnDirection = TURN_LEFT;
      headingDelta = 360 - desiredHeading + batmobile.heading;
    }
    if(desiredHeading - batmobile.heading < 180)
    {
      turnDirection = TURN_RIGHT;
      headingDelta = desiredHeading - batmobile.heading;
    }
  }

  SerialUSB.print("headDelta[");
  SerialUSB.print(headingDelta);
  SerialUSB.print("]");

  if(headingDelta < 5) turnDirection = TURN_NONE; //Drive straight!
  
  SerialUSB.print("drive[");
  if (turnDirection == TURN_RIGHT) SerialUSB.print("Right");
  if (turnDirection == TURN_LEFT) SerialUSB.print("Left");
  if (turnDirection == TURN_NONE) SerialUSB.print("Straight!");
  SerialUSB.print("]distance[");
  SerialUSB.print(distanceToWP, 2);
  SerialUSB.print("ft]");

  SerialUSB.println();

  //Convert heading to steering position
//  break here

  //We have a recommended heading from GPS/Compass but let's check that against walls

/*
  int newSteeringPosition = desiredHeading; //We will go with desiredHeading if wall logic doesn't change it
  
  //Decide if we have a wall in the way
  if(batmobile.leftDistance < left_wall_min_distance)
  {
    SerialUSB.println("Wall left!");
    if(millis() - wall_left_timestamp < left_wall_time_before_a_lot_turn)
    {
      newSteeringPosition = a_little_right;
    }
    else
    {
      //We've spent too much time near the wall, get more drastic and turn a lot away
      newSteeringPosition = a_lot_right;
    }
  }
  else //No left wall obstruction detected!
  {
    wall_left_timestamp = millis(); //Reset timestamp
  }

  if(batmobile.rightDistance < right_wall_min_distance)
  {
    SerialUSB.println("Wall right!");
    if(millis() - wall_right_timestamp < right_wall_time_before_a_lot_turn)
    {
      newSteeringPosition = a_little_left;
    }
    else
    {
      //We've spent too much time near the wall, get more drastic and turn a lot away
      newSteeringPosition = a_lot_left;
    }
  }
  else //No right wall obstruction detected!
  {
    wall_right_timestamp = millis(); //Reset timestamp
  }

  SerialUSB.print(" newSteeringPosition[");
  SerialUSB.print(newSteeringPosition);
  SerialUSB.print("]");
  SerialUSB.println();

  //Set the motor speed
  //Call the motor controller and set to setting
  */
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

/*
 * look up waypoints from eeprom
 * GPS coordinate entry
 * halo around that coordinate that is acceptable as 'acheived'
 * heading entry
 *
 */


