#include <Wire.h> //I2C Arduino Library

#define DISPLAY_ADDRESS1 0x72 //This is the default address of the OpenLCD

void setup()
{
  //Initialize Serial and I2C communications
  SerialUSB.begin(115200);
  while (!SerialUSB); //Magic code that waits for serial port to be allowed to print

  SerialUSB.println("Test LCD");

  Wire.begin(); //Join I2C bus as master

}

int counter = 0;
int gpsSIV = 4;
int gpsAge = 57;
int gpsLat = 56733;
String msg;

void loop()
{
  counter++;

  SerialUSB.print("test: ");
  SerialUSB.println(counter);

  clearLCD();
  //sendValueLCD(counter);

  //Doesn't show numbers less than 10?
  //sendStringLCD("Test ");
  //sendStringLCD((String)counter);

  //Works great
  //sendStringLCD("Test " + (String)counter);

  gpsLat++;
  msg = "";
  msg += "No GPS SIV[";
  msg += String(gpsSIV, DEC);
  msg += "]";
  msg += "AGE[";
  msg += String(gpsAge, DEC);
  msg += "]";
  msg += "LAT[";
  msg += String(gpsLat, DEC);
  msg += "]";

  clearLCD();
  blastMessage(msg);

  delay(250);

  //Really long strings may be a problem
}

void clearLCD()
{
  //Send the reset command to the display - this forces the cursor to return to the beginning of the display
  Wire.beginTransmission(DISPLAY_ADDRESS1);
  Wire.write('|'); //Put LCD into setting mode
  Wire.write('-'); //Send clear display command
  Wire.endTransmission();
}

void resetLCD()
{
  for (int x = 0 ; x < 5 ; x++)
  {
    Wire.beginTransmission(DISPLAY_ADDRESS1);
    Wire.endTransmission();
  }
}

//Send the message both to LCD and SerialUSB
void blastMessage(String msg)
{
  SerialUSB.println(msg);
  sendStringLCD(msg);
}

//Given a number, i2cSendValue chops up an integer into four values and sends them out over I2C
void sendValueLCD(int value)
{
  Wire.beginTransmission(DISPLAY_ADDRESS1); // transmit to device #1

  Wire.write('|'); //Put LCD into setting mode
  Wire.write('-'); //Send clear display command

  Wire.print("Cycles: ");
  Wire.print(value);

  Wire.endTransmission();
}

//Clears LCD and sends string
/*void sendStringLCD(String thing)
{
  Wire.beginTransmission(DISPLAY_ADDRESS1);

  Wire.print(thing);

  Wire.endTransmission();
}*/

//Clears LCD and sends string
void sendStringLCD(String thing)
{
  #define MAX_LENGTH  8
  int head = 0;
  int tail = 0;
  String temp;

  //Cut the long string into shorter strings of 8
  //I don't the I2C bus can handle large streams of data
  if (thing.length() > MAX_LENGTH)
  {
    while (1)
    {
      tail = head + MAX_LENGTH;
      if (tail > thing.length()) tail = thing.length();
      temp = thing.substring(head, tail);

      Wire.beginTransmission(DISPLAY_ADDRESS1);
      Wire.print(temp);
      Wire.endTransmission();

      head += MAX_LENGTH;
      if(head >= thing.length()) break;
    }
  }
  else
  {
    Wire.beginTransmission(DISPLAY_ADDRESS1);
    Wire.print(thing);
    Wire.endTransmission();
  }
}

