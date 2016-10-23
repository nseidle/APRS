/*
  Calibrate HMC5883L. Output for HMC5883L_calibrate_processing.pde
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
  GIT: https://github.com/jarzebski/Arduino-HMC5883L
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;

int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

void setup()
{
  SerialUSB.begin(115200);

  // Initialize Initialize HMC5883L
  while (!compass.begin())
  {
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
}

void loop()
{
  pingCompass();
  
  Vector mag = compass.readRaw();

  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;

  // Calculate offsets
  offX = (maxX + minX)/2;
  offY = (maxY + minY)/2;

  SerialUSB.print(mag.XAxis);
  SerialUSB.print(":");
  SerialUSB.print(mag.YAxis);
  SerialUSB.print(":");
  SerialUSB.print(minX);
  SerialUSB.print(":");
  SerialUSB.print(maxX);
  SerialUSB.print(":");
  SerialUSB.print(minY);
  SerialUSB.print(":");
  SerialUSB.print(maxY);
  SerialUSB.print(":");
  SerialUSB.print(offX);
  SerialUSB.print(":");
  SerialUSB.print(offY);
  SerialUSB.print("\n");
}

//Used to unlock I2C comm when it freezes
void pingCompass()
{

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

  TwoWire testI2C(&sercom1, 11, 13); //SDA = 11, SCL = 13

  testI2C.begin();

  //Put the HMC5883 IC into the correct operating mode
  testI2C.beginTransmission(address); //open communication with HMC5883
  testI2C.write(0x02); //select mode register
  testI2C.write((uint8_t)(0x00)); //continuous measurement mode
  testI2C.endTransmission();

  int x, y, z; //triple axis data

  //Tell the HMC5883L where to begin reading data
  testI2C.beginTransmission(address);
  testI2C.write(0x03); //select register 3, X MSB register
  testI2C.endTransmission();

  //Read data from each axis, 2 registers per axis
  testI2C.requestFrom(address, 6);
  if (6 <= testI2C.available())
  {
    x = testI2C.read() << 8; //X msb
    x |= testI2C.read(); //X lsb
    z = testI2C.read() << 8; //Z msb
    z |= testI2C.read(); //Z lsb
    y = testI2C.read() << 8; //Y msb
    y |= testI2C.read(); //Y lsb
  }

}


