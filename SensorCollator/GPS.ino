/*
  All the GPS control functions

  Example: 4021.08432, 10351.95963
*/

//Vars for GPS

String incomingResponse = "";
boolean sentenceComplete = false;
int checksumCounter = 0;

//End GPS Vars

void setupGPS()
{
  Serial1.begin(57600);
}

//Takes the received GPS sentence and cracks it into the various sub bits
//We assume the incomingGPSString variable has been filled with a proper gpgga sentence
boolean crackGPSSentence(String incomingGPSString)
{
  int sentenceLen = incomingGPSString.length();

  //SerialUSB.print("GPS: ");
  //SerialUSB.println(incomingGPSString);

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
    gpsChecksum = false;
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
  tempString = "";
  for (byte spot = startLat ; incomingGPSString[spot] != ',' ; spot++)
    tempString += incomingGPSString[spot];

  //Convert Latitude to double
  //4010.12345 = 10 chars
  //1234567890
  if (tempString.length() < 10)
  {
    gpsLat = -1; //None available
  }
  else
  {
    //Throw away the leader chars, 4010.
    gpsLat = tempString.substring(5, 10).toInt();
  }

  //Convert long to float. ATmega328 float can handle 6-7 decimal digits of precision
  //so 10316.95411 should be ok
  tempString = "";
  for (byte spot = startLong ; incomingGPSString[spot] != ',' ; spot++)
    tempString += incomingGPSString[spot];

  //Convert Latitude to double
  //10316.95411 = 11 chars
  //12345678901
  if (tempString.length() < 11)
  {
    gpsLong = -1; //None available
  }
  else
  {
    //Throw away the leader chars, 10316.
    gpsLong = tempString.substring(6, 11).toInt();
  }

  //Longitude should be negative but I don't care for nav purposes

  //Now go get Sats in view
  tempString = "";
  for (byte spot = startSIV ; incomingGPSString[spot] != ',' ; spot++)
    tempString += incomingGPSString[spot];
  if (tempString.length() < 1)
  {
    gpsSIV = 0;
  }
  else
  {
    gpsSIV = tempString.toInt();
  }

  //Now go get HDOP
  tempString = "";
  for (byte spot = startHDOP ; incomingGPSString[spot] != ',' ; spot++)
    tempString += incomingGPSString[spot];
  if (tempString.length() == 0)
  {
    gpsHDOP = -1;
  }
  else
  {
    gpsHDOP = tempString.toFloat();
  }

  //Now go get Altitude
  /*tempString = "";
  for (byte spot = startAlt ; incomingGPSString[spot] != ',' ; spot++)
    tempString += incomingGPSString[spot];
  gpsAltitude = tempString.toFloat();*/

  //SerialUSB.print("Lat: ");
  //SerialUSB.println(gpsLatitude);

  return (true);
}

//Checks incoming buffer for the start/stop characters
//Cracks any completed sentences as it becomes available
//Returns true if any serial traffic was received
bool checkGPS()
{
  //SerialUSB.print("GPS Check");
  
  bool serialWasReceived = false;

  while (Serial1.available())
  {
    serialWasReceived = true;

    char incomingChar = Serial1.read();

    if (incomingChar == '*')
    {
      sentenceComplete = true;
      checksumCounter = 2;

      //We have to get two more characters that are the checksum
      incomingResponse += incomingChar;
    }
    else if (sentenceComplete == true && checksumCounter > 0)
    {
      //Grab checksum chars
      incomingResponse += incomingChar;
      checksumCounter--;

      if (checksumCounter == 0)
      {
        sentenceComplete = false; //Reset variable

        //We are done! Crack it apart!
        if (crackGPSSentence(incomingResponse) == true) //True means we have good checksum
        {
          gpsAge = (float)millis(); //Time in float because I2C is setup to transmit floats
          
          //Consider turning off I2C events around this line
          //currentVector = temp; //We've got a good record. Load it.
          //..
          
          newInfo = true;

          SerialUSB.print("+");

          return (true);
        }
        else
        {
          SerialUSB.print("-");
          return (false); //Cracking failed, probably bad checksum
        }
      }
    }
    else if (incomingChar == '$')
    {
      SerialUSB.print("$");
      
      //Restart everything!
      incomingResponse = "";
      sentenceComplete = false; //Reset variable
    }
    else
    {
      //Just add it to the que
      incomingResponse += incomingChar;
    }
  } //End laseravailable()

  return (serialWasReceived);
}

