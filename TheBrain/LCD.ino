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
  dog.reset(); //pet the dog
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
void sendStringLCD(String thing)
{
#define MAX_LENGTH  8
  int head = 0;
  int tail = 0;
  String temp;

  //Cut the long string into shorter strings of MAX_LENGTH
  //I don't the I2C bus can handle large streams of data
  if (thing.length() > MAX_LENGTH)
  {
    while (1)
    {
      dog.reset(); //pet the dog

      tail = head + MAX_LENGTH;
      if (tail > thing.length()) tail = thing.length();
      temp = thing.substring(head, tail);

      Wire.beginTransmission(DISPLAY_ADDRESS1);
      Wire.print(temp);
      Wire.endTransmission();

      head += MAX_LENGTH;
      if (head >= thing.length()) break;
    }
  }
  else
  {
    Wire.beginTransmission(DISPLAY_ADDRESS1);
    Wire.print(thing);
    Wire.endTransmission();
  }
}

