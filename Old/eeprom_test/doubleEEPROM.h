//From Apex Logic. Really cool.
//http://apexlogic.net/code-bank/arduino/eeprom-i2c-write-anything/

#define DEVICE 0x50 //this is the device ID from the datasheet of the 24LC256

//in the normal write anything the eeaddress is incrimented after the writing of each byte. The Wire library does this behind the scenes.

template <class T> int eeWrite(int ee_address, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  Wire.beginTransmission(DEVICE);
  Wire.write((int)(ee_address >> 8)); // MSB
  Wire.write((int)(ee_address & 0xFF)); // LSB
  for (i = 0; i < sizeof(value); i++)
    Wire.write(*p++);
  Wire.endTransmission();
  return i;
}

template <class T> int eeRead(int ee_address, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  Wire.beginTransmission(DEVICE);
  Wire.write((int)(ee_address >> 8)); // MSB
  Wire.write((int)(ee_address & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(DEVICE, sizeof(value));
  for (i = 0; i < sizeof(value); i++)
    if (Wire.available())
      *p++ = Wire.read();
  return i;
}
