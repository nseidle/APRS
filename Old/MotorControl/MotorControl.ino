/*
  Motor Controller and Steering Controller
  Nathan Seidle
  SparkFun Electronics
  3/10/2016

  The motor controller receives an I2C value between 0 and 100 and outputs a PWM analog value to the 
  30A/1500W DC motor controller. Also reads the throttle (hall effect sensor from 1 to 4V) 
  and wheel steer pot position.
  
  To talk to the locomotionControl send two bytes of data to 0xCD
  Byte 1: motor speed, 0 to 99%
  Byte 2: steering position 0 to 99 (0 = far left, 99 = far right)

  If it doesn't receive a value every 1000ms the controller will turn off.

  If we use pin 9 and set the PWM freq to 31,250 Hz it is above hearing range.
  Pin 9 with next divisor of 8 makes a really high, annoying pitch.
*/

#include <SimpleTimer.h> //https://github.com/jfturcot/SimpleTimer
#include <avr/wdt.h> //We need watch dog for this program
#include <EEPROM.h> //Used to remember steer straight values
#include <Wire.h> //Needed for I2C slave communication

#include "pins.h" //Defines how all the hardware is connected

#define AUTO 0
#define HUMAN 2
byte systemMode = HUMAN; //Keeps track of which mode we are in

//Internal EEPROM locations for the straight/min/max values
//These are the min/max angles before we turn off linear actuators
//Thing of them as emergency stops
//Each value is an int, needs two bytes
#define LOCATION_STRAIGHT_ANGLE 0x02
#define LOCATION_MAX_ANGLE 0x04
#define LOCATION_MIN_ANGLE 0x06
#define LOCATION_POT_DELTA 0x08 //The + or - amount of pot reading before we believe we are at the requested steering angle

byte whichSetting = 0; //This is the pseudo 'memory address' that the master will set to read/write I2C stuff
//The following variables are 0, 1, 2, etc
int settingAngleStraight;
int settingAngleMax;
int settingAngleMin;
int settingPotDelta;
int requestedSteeringPosition;
int currentSteeringPosition;
int motorSpeed; //What the motor is at right now?
int throttlePosition; //Is the user pressing the throttle?

SimpleTimer timer;
long secondTimerID;
long motorUpdateID;
long timeOutID;
long steeringID;
long throttleID;
long brakeCheckID;
long serialUpdateID;

#define MAX_TIMEOUT 3000L //Number of ms between motor speed updates before system goes into safe mode
#define MOTORUPDATE_TIME 100L //Time between updating the motor
#define STEERING_CHECK_TIME 100L //Number of ms between checking the steering position
#define THROTTLE_CHECK_TIME 25L //Number of ms between checking the user's throttle
#define BRAKE_CHECK_TIME 25L //Number of ms between checking the user's brake

//0x0D is locomotion controller
//0x0E is Sensor Collator
//0x0F is Laser Collator
//0x1E is HMC5883L magneto

#define LOCOMOTION_ADDRESS 0x0D
#define SENSOR_COLLATOR_ADDRESS 0x0E
#define LASER_COLLATOR_ADDRESS 0x0F

//Define the various system states
char motorState;
#define MOTOR_STOP 'S'
#define RECEIVED_SETTING 'R'
#define RECEIVED_THROTTLE 'T'
#define MOTOR_USER_EMERGENCY_STOP 'U'
char steerState;
#define STEER_STOP 's'
#define STEER_ARRIVED 'a'
#define STEER_LEFT 'l'
#define STEER_RIGHT 'r'
#define STEER_USER_EMERGENCY_STOP 'u'

//Define the min/max thresholds of the hall effect sensor
//Analog value is 172 to 858
#define HALL_MIN 19 //Actual is 16 or 17
#define HALL_MAX 84
#define HALL_DELTA 2

//For my particular relay setup driving the pin high causes the relay to turn off
#define STEER_RELAY_OFF HIGH
#define STEER_RELAY_ON LOW

#define ERROR_THROTTLE_RANGE 2
#define ERROR_THROTTLE_MISSING 3

unsigned long timeSinceLastUpdate = 0; //Amount of time since last command from master
unsigned long throttleOffline = 0; //Amount of time since we last detected the throttle online

void setup()
{
  wdt_reset(); //Pet the dog
  wdt_disable(); //We don't want the watchdog during init

  //Turn off motor above all
  pinMode(THROTTLE_OUT, OUTPUT);
  digitalWrite(THROTTLE_OUT, LOW);

  Serial.begin(115200);

  pinMode(STAT, OUTPUT);

  setupSteering(); //Set the GPIOs
  setupMotor(); //Set the motor and throttle pins up
  setupBrake(); //Set up the brake switch
  setupModeSwitch(); //Setup weird GPIOs to check the auto switch

  //Wire.begin(LOCOMOTION_ADDRESS); //Join I2C bus as slave at this address
  //Wire.onRequest(requestEvent); //When master is asking for something, call this function

  serialUpdateID = timer.setInterval(100L, printUpdate); //Print a system update every Xms
  secondTimerID = timer.setInterval(1000L, secondPassed); //Call secondPassed every Xms
  timer.setInterval(1000L, checkAutoModeSwitch); //Check the switch every second to see what mode we're in

  Serial.println("Motor and Steering Controller Online");

  wdt_reset(); //Pet the dog
  //wdt_enable(WDTO_1S); //Unleash the beast
}

void loop()
{
  timer.run(); //Update any timers we are running
  wdt_reset(); //Pet the dog

  Serial.print("motorSpeed: ");
  Serial.print(motorSpeed, DEC);
  Serial.print(" motorState: ");
  Serial.print(motorState);

  Serial.print(" steerPot: ");
  Serial.print(readSteerPot());
  Serial.print(" steerState: ");
  Serial.print(steerState);
  Serial.println();

  if (Serial.available())
  {
    char incoming = Serial.read();

    if (incoming == 'S' || incoming == '0')
    {
      timeSinceLastUpdate = millis();
      motorState = MOTOR_STOP;
      motorSpeed = 0;
    }
    else if (incoming == '1')
    {
      timeSinceLastUpdate = millis();
      motorState = RECEIVED_SETTING;
      motorSpeed = 25;
    }
    else if (incoming == '2')
    {
      timeSinceLastUpdate = millis();
      motorState = RECEIVED_SETTING;
      motorSpeed = 75;
    }
    else if (incoming == 'a')
    {
      timeSinceLastUpdate = millis();
      motorState = RECEIVED_SETTING;
      motorSpeed += 5;
    }
    else if (incoming == 'z')
    {
      timeSinceLastUpdate = millis();
      motorState = RECEIVED_SETTING;
      motorSpeed -= 5;
    }
    else if(incoming == 'c') //Set the center point
    {
      calibrateCenter();
    }
    else if (incoming == '[')
    {
      
    }
  }

  //If we haven't received an I2C event in too long then check the throttle and respond to it
  /*if(motorSpeed == 0)
  {
    throttleValue = readThrottle();
    if(throttleValue > 0) 
      motorState = RECEIVED_THROTTLE;
    else
      motorState = MOTOR_STOP;
  }*/

  delay(25);
}

//Calibrate center steering position
//Allow user to very slightly adjust the 'drive straight' point of the trimpot
void calibrateCenter(void)
{
  requestedPosition = settingAngleStraight; //Tell steering to go to center

  int newCenter = settingAngleStraight;

  while(1)
  {
    Serial.println();
    Serial.print(F("Center: "));
    Serial.println(newCenter);
  
    Serial.println(F("L)eft 5"));
    Serial.println(F("R)ight 5"));
    Serial.println(F("l)eft 1"));
    Serial.println(F("r)ight 1"));
    Serial.print(F(">"));

    requestedPosition = newCenter; //Tell steering to go to this new center

    //Wait for user input
    while(Serial.available() == 0)
    {
      timer.run(); //This will drive/control the actuator
      wdt_reset(); //Pet the dog
    }
    
    char incoming = Serial.read();

    if(incoming == 'L')
      newCenter += 5;
    else if(incoming == 'R')
      newCenter -= 5;
    else if(incoming == 'l')
      newCenter += 1;
    else if(incoming == 'r')
      newCenter -= 1;
    else if(incoming == 'x')
    {
      settingAngleStraight = newCenter;
      writeEEPROMInt(LOCATION_STRAIGHT_ANGLE, settingAngleStraight); //Record to EEPROM
      
      Serial.println(F("Exiting"));
      return; //Look for escape character
    }

    //Error correct
    if(newCenter > 1023) newCenter = 1023;
    if(newCenter < 1) newCenter = 1;
  }
}

//Blink the status LED to show we are alive
void secondPassed()
{
  digitalWrite(STAT, !digitalRead(STAT));
}

//Every Xms print a system update and look for input from user
void printUpdate()
{
  if (motorState == MOTOR_USER_EMERGENCY_STOP)
  {
    Serial.println("Emergency Stop");
  }
  else
  {
    Serial.print(" Mode[");
    if (systemMode == HUMAN) Serial.print("HUMAN");
    if (systemMode == AUTO) Serial.print("AUTO");
    Serial.print("]");

    Serial.print(" brake[");
    if (userBraking()) Serial.print("ON");
    else Serial.print("OFF");
    Serial.print("]");

    Serial.print(" speed[");
    Serial.print(motorSpeed, DEC);
    Serial.print("]");
    //Serial.print(" motorState: ");
    //Serial.print(motorState);
    //Serial.print(" ThrottleIn: ");
    //Serial.print(throttlePosition, DEC);

    Serial.print(" potDelta[");
    Serial.print(settingPotDelta);
    Serial.print("]");

    Serial.print(" currentSteer[");
    Serial.print(currentSteeringPosition);
    Serial.print("]");

    Serial.print(" requestSteer[");
    Serial.print(requestedSteeringPosition);
    Serial.print("]");

    Serial.print(" Steer[");
    if (steerState == STEER_STOP) Serial.print("STOP");
    if (steerState == STEER_ARRIVED) Serial.print("Arrived");
    if (steerState == STEER_RIGHT) Serial.print("RIGHT");
    if (steerState == STEER_LEFT) Serial.print("LEFT");
    Serial.print("]");

    Serial.println();
  }

  if (Serial.available())
  {
    char incoming = Serial.read();
    Serial.print("!");

    if (incoming == 'S' || incoming == '0')
    {
      timeSinceLastUpdate = millis();
      if (motorState != MOTOR_USER_EMERGENCY_STOP) motorState = MOTOR_STOP;
      motorSpeed = 0;
    }
    else if (incoming == 'a')
    {
      timeSinceLastUpdate = millis();
      if (motorState != MOTOR_USER_EMERGENCY_STOP) motorState = RECEIVED_SETTING;
      motorSpeed += 5;
      if (motorSpeed > 99) motorSpeed = 99;
    }
    else if (incoming == 'z')
    {
      timeSinceLastUpdate = millis();
      if (motorState != MOTOR_USER_EMERGENCY_STOP) motorState = RECEIVED_SETTING;
      motorSpeed -= 5;
      if (motorSpeed < 0) motorSpeed = 0;
    }
    else if (incoming == 'c') //Set the center point
    {
      calibrateCenter();
    }
    else if (incoming == '[')
    {
      //Steer left a little bit
      requestedSteeringPosition += 5;
    }
    else if (incoming == ']')
    {
      //Steer right a little bit
      requestedSteeringPosition -= 5;
    }
  }
}

//The switch is wired oddly so we have to set a bunch of pins up for pull downs and things
void setupModeSwitch()
{

#define LED_OFF HIGH
#define LED_ON LOW

  pinMode(MODE_SWITCH_LOW, OUTPUT);
  digitalWrite(MODE_SWITCH_LOW, LOW); //Push pin low for GND connection
  pinMode(MODE_SWITCH_HIGH, OUTPUT);
  digitalWrite(MODE_SWITCH_HIGH, HIGH); //Push pin high for VCC

  pinMode(MODE_SWITCH_LED, OUTPUT);
  digitalWrite(MODE_SWITCH_LED, LED_OFF); //Turn off LED

  pinMode(MODE_SWITCH, INPUT);

}

//Checks to see if the Mode Switch is turned on
//Sets the global mode variable
void checkAutoModeSwitch()
{
#define LED_OFF HIGH
#define LED_ON LOW

#define MANUAL_MODE LOW
#define AUTO_MODE HIGH

  if (digitalRead(MODE_SWITCH) == MANUAL_MODE) //Switch is open, we are in manual mode
  {
    digitalWrite(MODE_SWITCH_LED, LED_OFF);
    systemMode = HUMAN;

    Wire.onReceive(receiveEvent_Offline); //When we get I2C bytes, call the offline dummy function

    throttleID = timer.setInterval(THROTTLE_CHECK_TIME, throttleUpdate); //Read the throttle position only if human is driving
  }
  else
  {
    timer.disable(throttleID); //Stop the throttle from being checked

    digitalWrite(MODE_SWITCH_LED, LED_ON);
    systemMode = AUTO;

    Wire.onReceive(receiveEvent); //When we get I2C bytes, call this function
  }
}

void blinkError(int errorType)
{
  wdt_reset(); //Pet the dog
  wdt_disable(); //We don't want the watchdog anymore

  motorSpeed = 0;
  analogWrite(THROTTLE_OUT, motorSpeed); //Shut it down

  Serial.println("Blinking Error. Shutting down.");

  //Disable all timers
  timer.disable(timeOutID); 
  timer.disable(secondTimerID);
  timer.disable(motorUpdateID);
  timer.disable(steeringID);
  timer.disable(throttleID);
  timer.disable(brakeCheckID);
  timer.disable(serialUpdateID);

  while (1)
  {
    delay(3000);

    for (int x = 0 ; x < errorType ; x++)
    {
      digitalWrite(STAT, HIGH);
      delay(250);
      digitalWrite(STAT, LOW);
      delay(250);
    }
  }

}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(byte pinToRead)
{
  byte numberOfReadings = 16;
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++)
  {
    runningValue += analogRead(pinToRead);
    delayMicroseconds(10);
  }
  runningValue /= numberOfReadings;

  return (runningValue);
}

