
#define WAYPOINT_START_POINT 256 //This gives us 100 spots in eeprom for other settings, like the number of waypoints

boolean loadWayPoints()
{
  //Get waypoints from EEPROM
  for (int x = 0 ; x < NUMBER_OF_WAYPOINTS ; x++)
  {
    readWP(x);

    //Show waypoints
    SerialUSB.print("WP[");
    SerialUSB.print(x);
    SerialUSB.print("]Lon[");
    SerialUSB.print(wayPoint[x].gpsLong, 5);
    SerialUSB.print("]Lat[");
    SerialUSB.print(wayPoint[x].gpsLat, 5);
    SerialUSB.print("]");
    SerialUSB.println();
    dogDelay(50);
  }

  return (true);
}

//When user tells us to, record the current WP to EEPROM
void enterWayPoints()
{

  while (1)
  {
    loadWayPoints(); //Print all waypoints

    dog.reset(); //Pet the dog

    batmobile.heading = (int)readSensor(SENSE_HEADING);
    batmobile.gpsLat = readSensor(SENSE_LAT) / (double)100000;
    batmobile.gpsLong = readSensor(SENSE_LONG) / (double)100000;
    batmobile.gpsAge = readSensor(SENSE_AGE);
    batmobile.gpsHDOP = readSensor(SENSE_HDOP);
    batmobile.gpsSIV = readSensor(SENSE_SIV);

    SerialUSB.print("WayPoint[");
    SerialUSB.print(currentWP);
    SerialUSB.print("] LongLat[");
    SerialUSB.print(wayPoint[currentWP].gpsLong, 5);
    SerialUSB.print("/");
    SerialUSB.print(wayPoint[currentWP].gpsLat, 5);
    SerialUSB.println("]");

    //GPS
    SerialUSB.print("Current LL[");
    SerialUSB.print(batmobile.gpsLong, 5);
    SerialUSB.print("/");
    SerialUSB.print(batmobile.gpsLat, 5);
    SerialUSB.print("]");

    SerialUSB.print(" H[");
    SerialUSB.print(batmobile.heading);
    SerialUSB.print("]");

    SerialUSB.print(" S[");
    SerialUSB.print(batmobile.gpsSIV);
    SerialUSB.print("]");

    SerialUSB.print(" DOP[");
    SerialUSB.print(batmobile.gpsHDOP);
    SerialUSB.print("]");

    SerialUSB.print(" A[");
    SerialUSB.print(batmobile.gpsAge);
    SerialUSB.print("]");
    SerialUSB.println();

    SerialUSB.println("r: Record current waypoint");
    SerialUSB.println("+/-: Increase/Decrease Waypoint #");
    SerialUSB.println("x: Exit");
    SerialUSB.println(">");
    SerialUSB.println();

    while (!SerialUSB.available()) dogDelay(1); //Wait for user input

    char incoming = SerialUSB.read();

    if (incoming == '+' || incoming == 'a')
    {
      currentWP++;
      if (currentWP > NUMBER_OF_WAYPOINTS) currentWP = NUMBER_OF_WAYPOINTS;
    }
    else if (incoming == '-' || incoming == 'z')
    {
      currentWP--;
      if (currentWP < 0) currentWP = 0;
    }
    else if (incoming == 'r')
    {
      //Record waypoint to EEPROM
      writeWP(currentWP, batmobile.gpsLat, batmobile.gpsLong);

      //Set current Waypoint in the array
      wayPoint[currentWP].gpsLat = batmobile.gpsLat;
      wayPoint[currentWP].gpsLong = batmobile.gpsLong;
    }
    else if (incoming == 'E')
    {
      SerialUSB.println("WPs Erased");
      for (int x = 0 ; x < NUMBER_OF_WAYPOINTS ; x++)
        writeWP(x, (double)0.9999, (double)0.9999);
    }
    else if (incoming == 'T')
    {
      SerialUSB.println("Test write");
      for (int x = 0 ; x < NUMBER_OF_WAYPOINTS * 2 ; x += 2)
        writeWP(x / 2, (double)x / 10000, (double)(x+1) / 10000);
    }
    else if (incoming == 'x')
    {
      break;
    }
  }
}

//Given a wapoint number, record to a spot in EEPROM
void writeWP(int waypointNumber, double lat, double lon)
{
  //int location = (int)WAYPOINT_START_POINT + (int)(waypointNumber * (sizeof(double) * 2)); //Should be 8 bytes per double, and we need to store two things
  int location = (int)WAYPOINT_START_POINT + (int)(waypointNumber * (8 * 2)); //Should be 8 bytes per double, and we need to store two things

  delay(10);
  eeWrite(location, lat);
  delay(10);
  eeWrite(location + 8, lon);
}

//Given a waypoint, read from EEPROM and record it to the waypoint array
void readWP(int waypointNumber)
{
  //int location = (int)WAYPOINT_START_POINT + (int)(waypointNumber * (sizeof(double) * 2)); //Should be 8 bytes per double, and we store two things in each location
  int location = (int)WAYPOINT_START_POINT + (int)(waypointNumber * (8 * 2)); //Should be 8 bytes per double, and we store two things in each location

  //Lat comes first
  delay(10);
  double lat;
  eeRead(location, lat);
  wayPoint[waypointNumber].gpsLat = lat;
  delay(10);
  double lon;
  eeRead(location + 8, lon);
  wayPoint[waypointNumber].gpsLong = lon;
}

