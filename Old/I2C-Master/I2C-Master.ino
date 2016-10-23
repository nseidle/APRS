/*
  Master Controller
  Nathan Seidle
  SparkFun Electronics
  3/27/2016

  Master controller sends bytes to the steering controller and read values from locomotion controller

  I2C Connections:
  SDA: Blue on bus
  SCL: Green on Bus

*/

#include <SimpleTimer.h> //https://github.com/jfturcot/SimpleTimer
#include <Wire.h> //Needed for I2C slave communication

//0x0D is locomotion controller
//0x0E is Sensor Collator
//0x0F is Laser Collator
//0x1E is HMC5883L magneto

#define LOCOMOTION_ADDRESS 0x0D
#define SENSOR_COLLATOR_ADDRESS 0x0E
#define LASER_COLLATOR_ADDRESS 0x0F

#define STAT 13

SimpleTimer timer;
long secondTimerID;

//Number of milliseconds between
#define CHECKIN_PERIOD 100L

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
boolean statLEDon = false; //Keep track of the state of the status LED because there is MOSFET controlling it

int steeringPosition;
int throttlePosition;

unsigned long leftDistance;
unsigned long centerDistance;
unsigned long rightDistance;
unsigned long leftAge;
unsigned long centerAge;
unsigned long rightAge;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define LOCO_REQUESTED_STEERING 4
#define LOCO_CURRENT_STEERING 5
#define LOCO_MOTOR_SPEED 6
#define LOCO_THROTTLE_POSITION 7

#define SENSE_HEADING 0
#define SENSE_LAT 1
#define SENSE_LONG 2
#define SENSE_ALT 3
#define SENSE_HDOP 4
#define SENSE_SIV 5

#define DISTANCE_ALL 0
#define DISTANCE_LEFT 1
#define DISTANCE_CENTER 2
#define DISTANCE_RIGHT 3
#define DISTANCE_LEFT_AGE 4
#define DISTANCE_CENTER_AGE 5
#define DISTANCE_RIGHT_AGE 6

void setup()
{
  pinMode(STAT, OUTPUT);

  SerialUSB.begin(115200);

  Wire.begin(); //Join I2C bus as master

  secondTimerID = timer.setInterval(1000L, secondPassed); //Call secondPassed every Xms

  SerialUSB.println("Master Controller Online");

}

void loop()
{
  timer.run(); //Update any timers we are running

  if (SerialUSB.available())
  {
    char incoming = SerialUSB.read();

    if (incoming == '1')
    {
      SerialUSB.println("Requesting steering to 35");
      writeLocomotion(LOCO_REQUESTED_STEERING, 35); //steering = -100 (far left) to 99 (far right), throttle = 0 to 99%
    }
    else if (incoming == '2')
    {
      SerialUSB.println("Writing motor speed to 17");
      writeLocomotion(LOCO_MOTOR_SPEED, 17); //motorSpeed = 0 to 99%
    }
    else if (incoming == '3') //Read current steering position
    {
      int temp = readLocomotion(LOCO_CURRENT_STEERING);
      SerialUSB.print("Reading steering position: ");
      SerialUSB.println(temp);
    }
    else if (incoming == '4') //Read current motor speed
    {
      int temp = readLocomotion(LOCO_MOTOR_SPEED);
      SerialUSB.print("Reading motorSpeed: ");
      SerialUSB.println(temp);
    }
    else if (incoming == '5')
    {
      SerialUSB.print("Reading throttle: ");
      int temp = readLocomotion(LOCO_THROTTLE_POSITION);
      SerialUSB.println(temp);
    }
    else if (incoming == '6')
    {
      SerialUSB.print("Reading heading: ");
      float temp = 0;
      for (int x = 0 ; x < 5 ; x++) //Attempt 5 times to get good reading
      {
        temp = readSensor(SENSE_HEADING);
        if (isnan(temp) == false) break; //Break from loop if we have a good reading
      }

      SerialUSB.println(temp, 2);
    }
    else if (incoming == '7')
    {
      SerialUSB.print("Reading lat: ");
      String temp = readSensorString(SENSE_LAT);
      SerialUSB.println(temp);
    }
    else if (incoming == '8')
    {
      leftDistance = readDistance(DISTANCE_LEFT);
      centerDistance = readDistance(DISTANCE_CENTER);
      rightDistance = readDistance(DISTANCE_RIGHT);
      leftAge = readDistance(DISTANCE_LEFT_AGE);
      centerAge = readDistance(DISTANCE_CENTER_AGE);
      rightAge = readDistance(DISTANCE_RIGHT_AGE);

      SerialUSB.print("Left: ");
      SerialUSB.print(leftDistance);
      SerialUSB.print("mm ");
      SerialUSB.print(leftAge);
      SerialUSB.print("ms ");

      SerialUSB.print(" Center: ");
      SerialUSB.print(centerDistance);
      SerialUSB.print("mm ");
      SerialUSB.print(centerAge);
      SerialUSB.print("ms ");

      SerialUSB.print(" Right: ");
      SerialUSB.print(rightDistance);
      SerialUSB.print("mm ");
      SerialUSB.print(rightAge);
      SerialUSB.print("ms ");

      SerialUSB.println();
    }
  }

  delay(25);
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
//Ask the locomotion controller what position the steering is at
void readLocomotion()
{
  Wire.beginTransmission(LOCOMOTION_ADDRESS);
  Wire.write(4); //Set 'memory address' to requestedPosition
  Wire.endTransmission();

  //Read the data at this address
  Wire.requestFrom(LOCOMOTION_ADDRESS, 2);
  //while(Wire.available() < 2) delay(1); //Wait

  byte chunk1 = Wire.read();
  byte chunk2 = Wire.read();

  steeringPosition = (int)chunk2 << 8 | chunk1;

  //throttlePosition = Wire.read();
}

//Send a value to a spot on the locomotion controller
void writeLocomotion(byte addr, int value)
{
  Wire.beginTransmission(LOCOMOTION_ADDRESS);
  Wire.write(addr); //Spot 4 is requestedPosition

  //This is really screwy. We have to split the values into an array and send the array
  byte data[2];
  data[0] = value;
  data[1] = value >> 8;
  Wire.write(data, 2);

  Wire.endTransmission();
}

//Read a given addr from the locomotion controller
int readLocomotion(byte addr)
{
  Wire.beginTransmission(LOCOMOTION_ADDRESS);
  Wire.write(addr); //Set 'memory address' to whatever the caller tells us
  Wire.endTransmission();

  //Read the data at this address
  Wire.requestFrom(LOCOMOTION_ADDRESS, 2);
  byte chunk1 = Wire.read();
  byte chunk2 = Wire.read();
  return ((int)chunk2 << 8 | chunk1);
}

//Read a given addr from the sensor controller
float readSensor(byte addr)
{
  Wire.beginTransmission(SENSOR_COLLATOR_ADDRESS);
  Wire.write(addr); //Set 'memory address' to whatever the caller tells us
  Wire.endTransmission();

  //Read data from this device
  //Floats from sensor collator are 4 bytes
  Wire.requestFrom(SENSOR_COLLATOR_ADDRESS, 4);
  byte data[sizeof(float)];
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  data[3] = Wire.read();

  float value;
  memcpy(&value, data, sizeof value);
  return (value);
}

//Read a given addr from the sensor controller
//To maintain precision (5 decimal digits) Lat and Long values come in as strings instead of float
String readSensorString(byte addr)
{
  Wire.beginTransmission(SENSOR_COLLATOR_ADDRESS);
  Wire.write(addr); //Set 'memory address' to whatever the caller tells us
  Wire.endTransmission();

  //Lat is 10 byte string
  Wire.requestFrom(SENSOR_COLLATOR_ADDRESS, 10);
  String temp = "";
  for (byte x = 0 ; x < 10 ; x++)
  {
    char incoming = Wire.read();
    temp += incoming;
  }

  return (temp);
}

//Read a given addr from the distance controller
unsigned long readDistance(byte addr)
{
  Wire.beginTransmission(LASER_COLLATOR_ADDRESS);
  Wire.write(addr); //Set 'memory address' to whatever the caller tells us
  Wire.endTransmission();

  //Read data from this device
  //Data from distance collator are 4 bytes
  Wire.requestFrom(LASER_COLLATOR_ADDRESS, 4);
  byte data[sizeof(unsigned long)];
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  data[3] = Wire.read();

  unsigned long value;
  memcpy(&value, data, sizeof value);
  return (value);
}
