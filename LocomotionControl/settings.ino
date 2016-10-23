#include <EEPROM.h> //Used to remember steer straight values

//Reads the current system settings from EEPROM
//If anything looks weird, reset setting to default value
void readSystemSettings(void)
{
  //Read the current straight setting, default is 512
  settingAngleStraight = readEEPROMInt(LOCATION_STRAIGHT_ANGLE);
  if (settingAngleStraight == 0xFFFF)
  {
    settingAngleStraight = 512; //Reset straight angle
    writeEEPROMInt(LOCATION_STRAIGHT_ANGLE, settingAngleStraight); //Record to EEPROM
  }

  //Read the current max setting, default is 800
  settingAngleMax = readEEPROMInt(LOCATION_MAX_ANGLE);
  if (settingAngleMax == 0xFFFF)
  {
    settingAngleMax = 516; //Reset max angle
    writeEEPROMInt(LOCATION_MAX_ANGLE, settingAngleMax); //Record to EEPROM
  }

  //Read the current min setting, default is 200
  settingAngleMin = readEEPROMInt(LOCATION_MIN_ANGLE);
  if (settingAngleMin == 0xFFFF)
  {
    settingAngleMin = 200; //Reset min angle
    writeEEPROMInt(LOCATION_MIN_ANGLE, settingAngleMin); //Record to EEPROM
  }

  //Read the allowed delta, default is 5
  settingPotDelta = readEEPROMInt(LOCATION_POT_DELTA);
  if (settingPotDelta == 0xFFFF)
  {
    settingPotDelta = 5; //Reset allowed delta
    writeEEPROMInt(LOCATION_POT_DELTA, settingPotDelta); //Record to EEPROM
  }

//LATE NIGHT FIX
  
  settingAngleStraight = 283; //LATE NIGHT HARD CODE
  settingAngleMax = settingAngleStraight + 150;
  settingAngleMin = settingAngleStraight - 150;

}

//Read an int from EEPROM
int readEEPROMInt(int location)
{
  byte byte1 = EEPROM.read(location);
  byte byte2 = EEPROM.read((int)(location + 1));

  int reading = (int)byte1 << 8 | (int)byte2;
  
  return(reading);
}

//Write an int to EEPROM
void writeEEPROMInt(int location, int data)
{
  EEPROM.write(location, (byte)(data >> 8));
  EEPROM.write(location + 1, (byte)data);
}

