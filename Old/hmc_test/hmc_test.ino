#include <Wire.h> //I2C Arduino Library
#include <HMC5883L.h> //From https://github.com/jarzebski/Arduino-HMC5883L

#include "wiring_private.h" // pinPeripheral() function

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

TwoWire testI2C(&sercom1, 11, 13); //SDA = 11, SCL = 13

HMC5883L compass;

void setup() {
  //Initialize Serial and I2C communications
  SerialUSB.begin(115200);
  while (!SerialUSB); //Magic code that waits for serial port to be allowed to print

  SerialUSB.println("Test hmc");

  pinPeripheral(11, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM_ALT);

  /*testI2C.begin();

  //Put the HMC5883 IC into the correct operating mode
  testI2C.beginTransmission(address); //open communication with HMC5883
  testI2C.write(0x02); //select mode register
  testI2C.write((uint8_t)0); //continuous measurement mode
  testI2C.endTransmission();*/

  setupCompass();
}

void loop() {

  int x, y, z; //triple axis data

  //Tell the HMC5883L where to begin reading data
  testI2C.beginTransmission(address);
  testI2C.write(0x03); //select register 3, X MSB register
  testI2C.endTransmission();


  //Read data from each axis, 2 registers per axis
  testI2C.requestFrom(address, 6);
  if (6 <= testI2C.available()) {
    x = testI2C.read() << 8; //X msb
    x |= testI2C.read(); //X lsb
    z = testI2C.read() << 8; //Z msb
    z |= testI2C.read(); //Z lsb
    y = testI2C.read() << 8; //Y msb
    y |= testI2C.read(); //Y lsb
  }

  //Print out values of each axis
  SerialUSB.print("x: ");
  SerialUSB.print(x);
  SerialUSB.print("  y: ");
  SerialUSB.print(y);
  SerialUSB.print("  z: ");
  SerialUSB.println(z);

  delay(250);
}

void setupCompass()
{
  SerialUSB.println("Initialize HMC5883L");

  byte tries = 0;
  while (!compass.begin())
  {
    SerialUSB.println("Could not find a valid HMC5883L sensor, check wiring!");
    if (tries++ > 3) return; //Give up
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
}
