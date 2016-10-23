/*
  All the GPS control functions

  Example: 4021.08432, 10351.95963
*/

//Vars for GPS 

#define MAX_GPS_WAIT  500 //Number of ms before we give up and error out

char incomingGPSString[100]; //Strings should only be 81 long
//End GPS Vars

//Check the GPS every 200ms
void gpsCheck()
{
  updateGPS();
  //if (updateGPS())
    //SerialUSB.print('+');
//  else
    //SerialUSB.print('-');
}

void setupGPS()
{
  Serial1.begin(57600);
  if (updateGPS() == false)
  {
    SerialUSB.println("GPS failed to respond");
  }

  gpsTimerID = timer.setInterval(200, gpsCheck); //Check for new GPS readings every 200ms
}

//When called this listens to the serial port for a GPGGA sentence
//Then splits and loads lat, long, siv, altitude, hdop, and checksum
//Returns false if failed to return
boolean updateGPS()
{
  boolean gpsChecksum = false; //Assume we have bad checksum

  //Clear any previous chars in buffer
  //Serial1.flush(); //Causes SAMD21 to lock for some reason

  //Spin until $
  long startTime = millis();
  while (1)
  {
    if (millis() - startTime > MAX_GPS_WAIT) return false;

    if (Serial1.available())
    {
      if (Serial1.read() == '$')
      {
        //This is the start
        //Now record until we have *
        byte endLength = 90; //This is max length
        byte sentenceLen = 0;
        for ( ; sentenceLen < endLength ; sentenceLen++)
        {
          while (!Serial1.available()) ; //Wait for incoming chars
          incomingGPSString[sentenceLen] = Serial1.read();

          //If we see the * we have two more chars to capture
          if (incomingGPSString[sentenceLen] == '*') endLength = sentenceLen + 3;

          //Check to see if we have the right sentence
          if (sentenceLen == 2 && incomingGPSString[2] == 'R') break;
        }

        #ifdef SHOW_GPS
          SerialUSB.println(incomingGPSString);
        #endif

        //Make sure this is a GNGGA sentence
        if (incomingGPSString[2] == 'G' && incomingGPSString[3] == 'G')
        {
          //Break it up!
          if(crackGPSSentence(sentenceLen) == true) //True means we have good checksum
          {
            batmobile.gpsAge = (float)millis(); //Time in float because I2C is setup to transmit floats
          
            return (true);
          }
          else
            return(false); //Cracking failed, probably bad checksum
        }

      } //End listening for $
    } //End available
  } //End while 1
}

//Given a string, check the checksum
//Returns true if checksum is valid
boolean verifyChecksum(String incomingString)
{

  //Checksum works like this:
  //Xor all the characters between $ and * together and store into one byte
  //Take the last two characters (after *), convert string to HEX value
  //Compare Xor'd byte to HEX byte

  /*int sentenceLength = incomingString.length();

    String strChecksum = incomingString.substring(sentenceLength - 2, sentenceLength);

    SerialUSB.print("strChecksum");
    SerialUSB.println(strChecksum);

    char arrayChecksum[2];
    strChecksum.toCharArray(arrayChecksum, 2); //Move these two chars into their own array

    byte incomingChecksum = (byte)strtoul(arrayChecksum, NULL, 0);

    SerialUSB.print("incomingChecksum");
    SerialUSB.println(incomingChecksum);

    //Peel off two characters, turn them into a 2 digit int, add all them together excluding
    //the checksum value doublet at the end of the string
    int sentenceChecksum = 0;
    for (int spot = 0 ; spot < sentenceLength - 3 ; spot += 2)
    sentenceChecksum += incomingString.substring(spot, spot + 2).toInt();

    sentenceChecksum %= 100; //Get to a two digit number

    //Convert the 2 digits at the end of the sentence to a checksum value
    int incomingChecksum = incomingString.substring(sentenceLength - 2, sentenceLength).toInt();

    if (incomingChecksum == sentenceChecksum) return (true);
    else return (false);

    return (false);*/

}

//Takes the received GPS sentence and cracks it into the various sub bits
//We assume the incomingGPSString variable has been filled with a proper gpgga sentence
boolean crackGPSSentence(byte sentenceLen)
{
  boolean gpsChecksum = false;
  
  //Calc the checksum
  byte calcChecksum = 0; //Clear and load checksum with this first character
  for (byte spot = 0 ; spot < sentenceLen - 3 ; spot++)
    calcChecksum ^= incomingGPSString[spot];

  //Convert the 2 digit hex value at the end of the sentence to a checksum value
  int incomingChecksum = 0;
  char readChar = incomingGPSString[sentenceLen - 2];
  for (byte i = 0; i < 2 ; i++)
  {
    incomingChecksum <<= 4; //Shift HEX by 4 bits

    readChar = incomingGPSString[sentenceLen - 2 + i]; //Pull 2nd from last, then 1st from last
    if (readChar >= 'A' && readChar <= 'F') incomingChecksum += readChar - 'A' + 10;
    else if (readChar >= '0' && readChar <= '9') incomingChecksum += readChar - '0';
  }
  if (incomingChecksum == calcChecksum)
  {
    //SerialUSB.println("Good Checksum");
    gpsChecksum = true;
  }
  else 
  {
    //SerialUSB.println("Bad Checksum");
    return (false);
  }

  //So we have a good sentence!

  //Find the significant comma positions
  byte startLat, startLong, startSIV, startHDOP, startAlt;
  byte commaCount = 0;
  for (byte spot = 0 ; spot < sentenceLen - 2 ; spot++)
  {
    if (incomingGPSString[spot] == ',')
    {
      commaCount++;
      if (commaCount == 2) startLat = spot + 1;
      if (commaCount == 4) startLong = spot + 1;
      if (commaCount == 7) startSIV = spot + 1;
      if (commaCount == 8) startHDOP = spot + 1;
      if (commaCount == 9) startAlt = spot + 1;
    }
  }

  String tempString;

  //Convert lat to float. ATmega328 float can handle 6-7 decimal digits of precision
  //so 4001.07935 should be ok
  String gpsLatitude = "";
  for (byte spot = startLat ; incomingGPSString[spot] != ',' ; spot++)
    gpsLatitude += incomingGPSString[spot];

  //Convert Latitude to double
  //4010.12345 = 10 chars
  //1234567890
  if(gpsLatitude.length() < 10)
  { 
    batmobile.gpsLat = -1; //None available
  }
  else
  {
    //Throw away the leader chars, 4010.
    int localLat = gpsLatitude.substring(5, 10).toInt();
    
    batmobile.gpsLat = localLat / (double)100000;
  }
  
  //Convert long to float. ATmega328 float can handle 6-7 decimal digits of precision
  //so 10316.95411 should be ok
  String gpsLongitude = "";
  for (byte spot = startLong ; incomingGPSString[spot] != ',' ; spot++)
    gpsLongitude += incomingGPSString[spot];

  //Convert Latitude to double
  //10316.95411 = 11 chars
  //12345678901
  if(gpsLongitude.length() < 11)
  { 
    batmobile.gpsLong = -1; //None available
  }
  else
  {
    //Throw away the leader chars, 10316.
    int localLong = gpsLongitude.substring(6, 11).toInt();
    
    batmobile.gpsLong = localLong / (double)100000;
  }

  //Longitude should be negative but I don't care for nav purposes

  //Now go get Sats in view
  tempString = "";
  for (byte spot = startSIV ; incomingGPSString[spot] != ',' ; spot++)
    tempString += incomingGPSString[spot];
  if(tempString.length() < 1)
  {
    batmobile.gpsSIV = 0;
  }
  else
  {
    batmobile.gpsSIV = tempString.toInt();
  }

  //Now go get HDOP
  tempString = "";
  for (byte spot = startHDOP ; incomingGPSString[spot] != ',' ; spot++)
    tempString += incomingGPSString[spot];
  if(tempString.length() == 0)
  {
    batmobile.gpsHDOP = -1;
  }
  else
  {
    batmobile.gpsHDOP = tempString.toFloat();
  }

  //Now go get Altitude
  /*tempString = "";
  for (byte spot = startAlt ; incomingGPSString[spot] != ',' ; spot++)
    tempString += incomingGPSString[spot];
  gpsAltitude = tempString.toFloat();*/

  //SerialUSB.print("Lat: ");
  //SerialUSB.println(gpsLatitude);

  return(true); //We have good checksum
}

