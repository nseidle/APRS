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

#include "pins.h"

#include "doubleEEPROM.h" //Needed for Waypoint recording on external EEPROM

#include "wd.h" //Comes from https://github.com/adafruit/Adafruit_SleepyDog/blob/master/utility/WatchdogSAMD.cpp

WatchdogSAMD dog;

#define COMPASS_LOCATION 1 //Compass offset is two bytes
#define UNDERWAY_LOCATION 8 //Location in EEPROM for underway variable. Helps us recover from WDT reset

#define UNDERWAY_STOPPED 255

SimpleTimer timer;
long secondTimerID;
long navigateTimerID; //Keeps track of when we should update heading and turning

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
boolean statLEDon = false; //Keep track of the state of the status LED because there is MOSFET controlling it

struct SOTW //State of the World
{
  double gpsLat;
  double gpsLong;
  float gpsAge;
  float gpsHDOP;
  int gpsSIV;
  int heading;

  byte Underway;

  int settingAngleStraight;
  int settingAngleMax;
  int settingAngleMin;
  int settingPotDelta;
  int requestedSteeringPosition;
  int currentSteeringPosition;
  int motorSpeed;
  int throttlePosition;
  int systemMode;
  int brakeOn;

  long leftDistance;
  long rightDistance;
  long centerDistance;
  long leftAge;
  long rightAge;
  long centerAge;
};

SOTW batmobile;

struct Coordinates
{
  double gpsLong;
  double gpsLat;
};

#define NUMBER_OF_WAYPOINTS 10 //Increase this if we want more waypoints
struct Coordinates wayPoint[NUMBER_OF_WAYPOINTS]; //These are our stored contest waypoints
int currentWP = 0; //This is our waypoint token. Start at zero.

String msg; //Generic string used for debug printing on LCD and serialUSB

boolean printedFirstTime = false; //This is just used to print the opening menu once
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Settings
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int cruisingSpeedForAuto = 15; //15%? Maybe?

unsigned int left_wall_min_distance = 750; //750mm to wall before we start to steer away
unsigned int right_wall_min_distance = 750; //750mm to wall before we start to steer away

//We want a 2.5m halo. If we are within 8.2 of the spot, we're close enough.
double halo_long_delta = 8.2 / 101 / (double)60; //1" = 101ft, 2.5m = 8.2ft
//8.2/101 / 60 = 0.0013531353135
double halo_lat_delta = 0.1 / (double)60; //1" = ~80ft, 2.5m = 8.2ft = 0.1"
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

//We're not sure how the compass will be stuck to car. Use cell phone to figure out offset from N.
int compassOffset; //61, Add a number that is the difference between cell phone N and what nav says is N
int rawHeading;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//0x0D is locomotion controller
//0x0E is Sensor Collator
//0x0F is Laser Collator
//0x1E is HMC5883L magneto
//0x72 is the OpenLCD

#define LOCOMOTION_ADDRESS 0x0D
#define SENSOR_COLLATOR_ADDRESS 0x0E
#define LASER_COLLATOR_ADDRESS 0x0F
#define DISPLAY_ADDRESS1 0x72 //This is the default address of the OpenLCD
#define EEPROM_ADDRESS 0x50 //Used for storing waypoints and settings

#define LOCO_REQUESTED_STEERING 4
#define LOCO_CURRENT_STEERING 5
#define LOCO_MOTOR_SPEED 6
#define LOCO_THROTTLE_POSITION 7
#define LOCO_SYSTEM_MODE 8
#define LOCO_BRAKE_ON 9
#define AUTO 0
#define HUMAN 2

#define SENSE_HEADING 0
#define SENSE_LAT 1
#define SENSE_LONG 2
#define SENSE_ALT 3
#define SENSE_HDOP 4
#define SENSE_SIV 5
#define SENSE_AGE 6

#define DISTANCE_ALL 0
#define DISTANCE_LEFT 1
#define DISTANCE_CENTER 2
#define DISTANCE_RIGHT 3
#define DISTANCE_LEFT_AGE 4
#define DISTANCE_CENTER_AGE 5
#define DISTANCE_RIGHT_AGE 6

void setup()
{
  dog.reset(); //pet the dog
  dog.disable();

  pinMode(STAT, OUTPUT);
  secondTimerID = timer.setInterval(1000L, secondPassed); //Call secondPassed every Xms

  SerialUSB.begin(115200);
  //We will get stuck here if USB cable is not attached.
  //Uncomment for debug/testing
  //while (!SerialUSB); //Magic code that waits for serial port to be allowed to print

  //dog.enable(512); //Hope to hit 512ms
  //dog.enable(5120); //Hope to hit 512ms
  dog.reset(); //pet the dog

  Wire.begin(); //Join I2C bus as master

  setupButton(); //Setup things to read the user button

  resetLCD();
  clearLCD();
  blastMessage("Hello");
  delay(250);
  dog.reset(); //pet the dog
  clearLCD();

  pingControllers(); //See who responds
}

void loop()
{
  dog.reset(); //pet the dog
  timer.run(); //Update any timers we are running

  if (SerialUSB.available() || printedFirstTime == false)
  {
    printedFirstTime = true; //Don't print opening menu a second time

    char incoming = SerialUSB.read();

    SerialUSB.println("t: Test menu");
    SerialUSB.println("p: Enter waypoints");
    SerialUSB.println("n: Navigate");
    SerialUSB.println("c: Compass");
    SerialUSB.println();

    if (incoming == 't')
      testMenu();
    else if (incoming == 'p')
      enterWayPoints();
    else if (incoming == 'n')
    {
      SerialUSB.println("Are you sure? y/n:");
      while (!SerialUSB.available()) dogDelay(1);
      if (SerialUSB.read() == 'y') navigate();

      //If we exit from navigate, return to stopped condition
      batmobile.Underway = UNDERWAY_STOPPED;
      writeUnderway();
    }
    else if(incoming == 'c')
    {
      compassCalibrate();
    }
  }

  //See if user is holding button
  if (digitalRead(DUIT_BUTTON) == LOW) buttonPressed();
}

//Attempts to talk to each controller
void pingControllers()
{
  int tries;

  if (loadWayPoints() == false) //Load the waypoints from EEPROM to local memory
  {
    blastMessage(" Waypoints bad"); //Sends to both LCD and SerialUSB with not clear
  }
  else
  {
    blastMessage(" Waypoints good");
  }

  //Moto
  for (tries = 0 ; tries < 5 ; tries++)
  {
    if (readLocomotion(LOCO_SYSTEM_MODE) != -1) break;
    dog.reset(); //pet the dog
    delay(100);
  }
  if (tries == 5) blastMessage(" Moto bad");
  else blastMessage(" Moto good");

  //GPS
  for (tries = 0 ; tries < 5 ; tries++)
  {
    if (readSensor(SENSE_LAT) != -1) break;
    dog.reset(); //pet the dog
    delay(100);
  }
  if (tries == 5) blastMessage(" GPS bad");
  else blastMessage(" GPS good");

  //Lasers
  for (tries = 0 ; tries < 5 ; tries++)
  {
    if (readDistance(DISTANCE_LEFT_AGE) != -1) break;
    dog.reset(); //pet the dog
    delay(100);
  }
  if (tries == 5) blastMessage(" Laser bad");
  else blastMessage(" Laser good");

  msg = " Let's roll!";
  SerialUSB.println(msg);
  sendStringLCD(msg);
}

void testMenu()
{
  while (1)
  {
    SerialUSB.println("1: Requesting steering to 35");
    SerialUSB.println("2: Writing motor speed to 10");
    SerialUSB.println("3: Read steering position");
    SerialUSB.println("4: Read motorSpeed");
    SerialUSB.println("5: Read throttle");
    SerialUSB.println("6: Read heading");
    SerialUSB.println("7: Read lat");
    SerialUSB.println("8: Read gpsAge");
    SerialUSB.println("9: LeftDistance");
    SerialUSB.println("a/s/d: Left/Center/Right");

    SerialUSB.println("w: State of the world");
    SerialUSB.println("x: Exit");
    SerialUSB.println();

    //Wait for user input
    while (!SerialUSB.available()) dogDelay(1);

    char incoming = SerialUSB.read();

    if (incoming == '1')
    {
      SerialUSB.println("Requesting steering to 35% left");
      writeLocomotion(LOCO_REQUESTED_STEERING, 35); //steering [-99 = 100% right, 99 = 100% left, 0 = straight]
    }
    else if (incoming == '2')
    {
      SerialUSB.println("Writing motor speed to 10");
      writeLocomotion(LOCO_MOTOR_SPEED, 10); //motorSpeed = 0 to 99%
    }
    else if (incoming == '3') //Read current steering position
    {
      int temp = readLocomotion(LOCO_CURRENT_STEERING);
      SerialUSB.print("Steering position: ");
      SerialUSB.println(temp);
    }
    else if (incoming == '4') //Read current motor speed
    {
      int temp = readLocomotion(LOCO_MOTOR_SPEED);
      SerialUSB.print("MotorSpeed: ");
      SerialUSB.println(temp);
    }
    else if (incoming == '5')
    {
      SerialUSB.print("Throttle: ");
      int temp = readLocomotion(LOCO_THROTTLE_POSITION);
      SerialUSB.println(temp);
    }
    else if (incoming == '6')
    {
      SerialUSB.print("Heading: ");
      float temp = 0;
      for (int x = 0 ; x < 5 ; x++) //Attempt 5 times to get good reading
      {
        temp = readSensor(SENSE_HEADING);
        if (isnan(temp) == false) break; //Break from loop if we have a good reading

        SerialUSB.println("Failed, trying again");
      }

      SerialUSB.println(temp, 2);
    }
    else if (incoming == '7')
    {
      SerialUSB.print("Lat: ");
      long temp = readSensor(SENSE_LAT);
      //batmobile.gpsLat = temp / 100000;
      SerialUSB.println(temp);
    }
    else if (incoming == '8')
    {
      SerialUSB.print("gpsAge: ");
      batmobile.gpsAge = (long)readSensor(SENSE_AGE);
      SerialUSB.println(batmobile.gpsAge);
    }
    else if (incoming == '9')
    {
      SerialUSB.print("leftDistance: ");
      batmobile.leftDistance = readDistance(DISTANCE_LEFT);
      SerialUSB.println(batmobile.leftDistance);
    }
    else if (incoming == 'p')
    {
      enterWayPoints();
    }
    else if (incoming == 'a')
    {
      SerialUSB.println("Requesting steering to 35% left");
      writeLocomotion(LOCO_REQUESTED_STEERING, 35); //steering [-99 = 100% right, 99 = 100% left, 0 = straight]
    }
    else if (incoming == 's')
    {
      SerialUSB.println("Requesting steering to 0%");
      writeLocomotion(LOCO_REQUESTED_STEERING, 0); //steering [-99 = 100% right, 99 = 100% left, 0 = straight]
    }
    else if (incoming == 'd')
    {
      SerialUSB.println("Requesting steering to 35% right");
      writeLocomotion(LOCO_REQUESTED_STEERING, -35); //steering [-99 = 100% right, 99 = 100% left, 0 = straight]
    }
    else if (incoming == 'w')
    {
      dog.reset(); //pet the dog
      //Get state of world

      //Get laser readings
      batmobile.leftDistance = readDistance(DISTANCE_LEFT);
      batmobile.centerDistance = readDistance(DISTANCE_CENTER);
      batmobile.rightDistance = readDistance(DISTANCE_RIGHT);
      batmobile.leftAge = readDistance(DISTANCE_LEFT_AGE);
      batmobile.centerAge = readDistance(DISTANCE_CENTER_AGE);
      batmobile.rightAge = readDistance(DISTANCE_RIGHT_AGE);

      //Get motor info
      //int settingAngleStraight; //Not yet implemented
      //int settingAngleMax;
      //int settingAngleMin;
      //int settingPotDelta;
      batmobile.requestedSteeringPosition = readLocomotion(LOCO_REQUESTED_STEERING);
      batmobile.currentSteeringPosition = readLocomotion(LOCO_CURRENT_STEERING);
      batmobile.motorSpeed = readLocomotion(LOCO_MOTOR_SPEED);
      batmobile.throttlePosition = readLocomotion(LOCO_THROTTLE_POSITION);
      batmobile.systemMode = readLocomotion(LOCO_SYSTEM_MODE);

      //TODO error check these values
      if (batmobile.leftDistance > 10000) batmobile.leftDistance = -1;

      //Get GPS data
      //Get heading
      batmobile.heading = (int)readSensor(SENSE_HEADING);
      batmobile.gpsLat = readSensor(SENSE_LAT) / (double)100000;
      batmobile.gpsLong = readSensor(SENSE_LONG) / (double)100000;
      batmobile.gpsAge = readSensor(SENSE_AGE);
      batmobile.gpsHDOP = readSensor(SENSE_HDOP);
      batmobile.gpsSIV = readSensor(SENSE_SIV);

      SerialUSB.print("Laser L[");
      SerialUSB.print(batmobile.leftDistance);
      SerialUSB.print(" ");
      SerialUSB.print(batmobile.leftAge);
      SerialUSB.print("]");

      SerialUSB.print(" C[");
      SerialUSB.print(batmobile.centerDistance);
      SerialUSB.print(" ");
      SerialUSB.print(batmobile.centerAge);
      SerialUSB.print("]");

      SerialUSB.print(" R[");
      SerialUSB.print(batmobile.rightDistance);
      SerialUSB.print(" ");
      SerialUSB.print(batmobile.rightAge);
      SerialUSB.print("]");

      SerialUSB.println();

      //Motor
      SerialUSB.print("Motor RSP[");
      SerialUSB.print(batmobile.requestedSteeringPosition);
      SerialUSB.print("]");

      SerialUSB.print(" CS[");
      SerialUSB.print(batmobile.currentSteeringPosition);
      SerialUSB.print("]");

      SerialUSB.print(" MS[");
      SerialUSB.print(batmobile.motorSpeed);
      SerialUSB.print("]");

      SerialUSB.print(" TR[");
      SerialUSB.print(batmobile.throttlePosition);
      SerialUSB.print("]");

      SerialUSB.print(" MD[");
      if (batmobile.systemMode == AUTO)
        SerialUSB.print("AUTO");
      else
        SerialUSB.print("HUMAN");
      SerialUSB.print("]");

      SerialUSB.println();

      //GPS
      SerialUSB.print("GPS LL[");
      SerialUSB.print(batmobile.gpsLong, 5);
      SerialUSB.print("/");
      SerialUSB.print(batmobile.gpsLat, 5);
      SerialUSB.print("]");

      SerialUSB.print(" H[");
      SerialUSB.print(batmobile.heading);
      SerialUSB.print("]");

      SerialUSB.print(" S[");
      SerialUSB.print(batmobile.gpsSIV);
      SerialUSB.print("]");

      SerialUSB.print(" DOP[");
      SerialUSB.print(batmobile.gpsHDOP);
      SerialUSB.print("]");

      SerialUSB.print(" A[");
      SerialUSB.print(batmobile.gpsAge);
      SerialUSB.print("]");

      SerialUSB.println();
    }
    else if (incoming == 'x')
    {
      break; //User wants to quit
    }
  }
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

//Used to keep track of whether or not the vehicle was driving
//So that we can resume quickly if it has
void readUnderway()
{
  //Read the variable from non-volatile memory
  eeRead(UNDERWAY_LOCATION, batmobile.Underway);
}

void writeUnderway()
{
  eeWrite(UNDERWAY_LOCATION, batmobile.Underway);
}

//Delay and pet dog
void dogDelay(long amount)
{
  long startTime = millis();
  while (millis() - startTime < amount)
  {
    delay(1);
    dog.reset(); //pet the dog
  }
}


