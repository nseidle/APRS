
//This is the main logic and control for the batmobile


#define TURN_LEFT 1
#define TURN_RIGHT 2
#define TURN_NONE 3


void navigate()
{
  clearLCD();

  //Check to see if we are in auto or manual mode
  batmobile.systemMode = readLocomotion(LOCO_SYSTEM_MODE);

  if (batmobile.systemMode == HUMAN)
  {
    blastMessage(" Motor in human mode. Please switch to auto. Aborting.");
    return;
  }
  else if (batmobile.systemMode == -1)
  {
    blastMessage(" Motor comm failed. Aborting.");
    return;
  }

  blastMessage(" Go!");
  dog.reset(); //pet the dog

  //Begin nav
  if (batmobile.Underway != 255)
  {
    currentWP = batmobile.Underway; //Start from the last known Waypoint if we have ont
  }
  else
  {
    currentWP = 0; //Start from beginning
  }

  while (currentWP < NUMBER_OF_WAYPOINTS) //Navigate until we get all wayPoints
  {
    dogDelay(250);
    clearLCD();

    //Look for emergency exits
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    //Human brake
    //Not yet implemented on the motor controller
    /*batmobile.brakeOn = readLocomotion(LOCO_BRAKE_ON);
      if (batmobile.brakeOn == true)
      {
      blastMessage(" Human pressed the brake! Aborting.");
      return;
      }*/

    //Human turns off auto switch
    //Check to see if we are in auto or manual mode
    batmobile.systemMode = readLocomotion(LOCO_SYSTEM_MODE);
    if (batmobile.systemMode == HUMAN)
    {
      blastMessage(" Motor in human mode. Please switch to auto. Aborting.");
      return;
    }

    //Serial
    if (SerialUSB.available())
    {
      char incoming = SerialUSB.read();
      if (incoming == 'X') //Stop navigating
      {
        break;
      }
    }

    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    //Get GPS data
    batmobile.gpsLat = readSensor(SENSE_LAT) / (double)100000;
    batmobile.gpsAge = readSensor(SENSE_AGE);
    batmobile.gpsSIV = readSensor(SENSE_SIV);

    //Check and wait for good GPS data
    int counter = 0;
    while (batmobile.gpsLat < 0 || batmobile.gpsLong < 0 || batmobile.gpsAge > 5000 || batmobile.gpsAge < 0)
    {
      batmobile.systemMode = readLocomotion(LOCO_SYSTEM_MODE);
      if (batmobile.systemMode == HUMAN)
      {
        blastMessage(" Motor in human mode. Exit nav.");
        return;
      }

      msg = "";
      msg += "No GPS SIV[";
      msg += String(batmobile.gpsSIV);
      msg += "]AGE[";
      msg += String(batmobile.gpsAge);
      msg += "]LAT[";
      msg += String(batmobile.gpsLat, 5);
      msg += "]LONG[";
      msg += String(batmobile.gpsLong, 5);
      msg += "]";

      SerialUSB.println(msg);

      clearLCD();
      sendStringLCD("No GPS " + String(counter++));

      dogDelay(250);

      //Attempt to read GPS again
      batmobile.gpsLat = readSensor(SENSE_LAT) / (double)100000;
      batmobile.gpsAge = readSensor(SENSE_AGE);
      batmobile.gpsSIV = readSensor(SENSE_SIV);
    }

    //Get the GPS data we haven't already
    batmobile.heading = readSensor(SENSE_HEADING);
    batmobile.gpsLong = readSensor(SENSE_LONG) / (double)100000;
    batmobile.gpsHDOP = readSensor(SENSE_HDOP);

    //Get laser readings
    batmobile.leftDistance = readDistance(DISTANCE_LEFT);
    batmobile.centerDistance = readDistance(DISTANCE_CENTER);
    batmobile.rightDistance = readDistance(DISTANCE_RIGHT);
    batmobile.leftAge = readDistance(DISTANCE_LEFT_AGE);
    batmobile.centerAge = readDistance(DISTANCE_CENTER_AGE);
    batmobile.rightAge = readDistance(DISTANCE_RIGHT_AGE);

    //Get motor info
    //int settingAngleStraight; //Not yet implemented
    //int settingAngleMax;
    //int settingAngleMin;
    //int settingPotDelta;
    batmobile.requestedSteeringPosition = readLocomotion(LOCO_REQUESTED_STEERING);
    batmobile.currentSteeringPosition = readLocomotion(LOCO_CURRENT_STEERING);
    batmobile.motorSpeed = readLocomotion(LOCO_MOTOR_SPEED);
    batmobile.throttlePosition = readLocomotion(LOCO_THROTTLE_POSITION);
    batmobile.systemMode = readLocomotion(LOCO_SYSTEM_MODE);

    //Calculate our new desired heading
    msg = "";
    msg += "WP[";
    msg += String(currentWP);
    //msg += "]To[";
    //msg += String(wayPoint[currentWP].gpsLat, 5);
    //msg += "/";
    //msg += String(wayPoint[currentWP].gpsLong, 5);
    //msg += "]Nw[";
    //msg += String(batmobile.gpsLat, 5);
    //msg += "/";
    //msg += String(batmobile.gpsLong, 5);
    //msg += "]SIV[";
    //msg += String(batmobile.gpsSIV);
    //msg += "]HDOP[";
    //msg += String(batmobile.gpsHDOP, 2);
    //msg += "]Age[";
    //msg += String(batmobile.gpsAge, 0); //Is this a problem?
    msg += "]head[";
    msg += String(batmobile.heading);
    msg += "]";

    clearLCD();
    blastMessage(msg);

    double xDelta = batmobile.gpsLong - wayPoint[currentWP].gpsLong;
    double yDelta = batmobile.gpsLat - wayPoint[currentWP].gpsLat;

    //Determine if we are in the halo of the wayPoint
    while (abs(xDelta) < halo_long_delta && abs(yDelta) < halo_lat_delta)
    {
      dogDelay(250);

      msg = "";
      msg += " WP[";
      msg += String(currentWP);
      msg += "] achieved! ";
      blastMessage(msg);

      currentWP++;

      xDelta = batmobile.gpsLong - wayPoint[currentWP].gpsLong;
      yDelta = batmobile.gpsLat - wayPoint[currentWP].gpsLat;
    }

    //Calc our desired heading
    double ratio = xDelta / yDelta;
    double alpha = atan(ratio); //Find the angle in the corner of the triangle
    alpha *= (double)180 / (double)PI; //Convert to degrees
    int desiredHeading; //This is the heading we want to achieve. Also called bearing.

    if (xDelta >= 0 && yDelta >= 0) desiredHeading = 180 - (int)alpha;
    else if (xDelta >= 0 && yDelta < 0) desiredHeading = (int)alpha * -1;
    else if (xDelta < 0 && yDelta < 0) desiredHeading = 360 - (int)alpha;
    else if (xDelta < 0 && yDelta >= 0) desiredHeading = 180 - (int)alpha;

    msg = "";
    msg += "wantHeading[";
    msg += String(desiredHeading);
    msg += "]";
    blastMessage(msg);

    //Decide which way to turn based on current heading and desired heading
    byte turnDirection;
    int headingDelta;

    if (batmobile.heading >= desiredHeading)
    {
      if (batmobile.heading - desiredHeading < 180)
      {
        turnDirection = TURN_LEFT;
        headingDelta = batmobile.heading - desiredHeading;
      }
      if (batmobile.heading - desiredHeading >= 180)
      {
        turnDirection = TURN_RIGHT;
        headingDelta = 360 - batmobile.heading + desiredHeading;
      }
    }
    else if (batmobile.heading < desiredHeading)
    {
      if (desiredHeading - batmobile.heading >= 180)
      {
        turnDirection = TURN_LEFT;
        headingDelta = 360 - desiredHeading + batmobile.heading;
      }
      if (desiredHeading - batmobile.heading < 180)
      {
        turnDirection = TURN_RIGHT;
        headingDelta = desiredHeading - batmobile.heading;
      }
    }


    if (headingDelta < 5) turnDirection = TURN_NONE; //Drive straight!

    /*msg = "";
      msg += "headDelta[";
      msg += String(headingDelta);
      msg += "]";
      blastMessage(msg);*/

    msg = "";
    msg += "drive[";
    if (turnDirection == TURN_RIGHT) msg += "R";
    if (turnDirection == TURN_LEFT) msg += "L";
    if (turnDirection == TURN_NONE) msg += "S";
    msg += "]";
    blastMessage(msg);

    //Calc our distance to the next WP
    double distanceToWP = (xDelta * xDelta + yDelta * yDelta);
    distanceToWP = sqrt(distanceToWP);
    //Convert distance to feet
    //1" = ~80ft long and 1" = 101ft lat so let's say 1" = 100ft
    //0.1" = 10ft.
    //0.00143 = 10 * 0.00143 / .1 = 0.143ft
    //0.00976 = ~8ft. = 0.976ft
    distanceToWP *= 1000;

    msg = "";
    msg += "dist[";
    msg += String(distanceToWP, 1);
    msg += "]";
    blastMessage(msg);

    //Convert heading to steering position
    //  break here

    //We have a recommended heading from GPS/Compass but let's check that against walls

    /*
      int newSteeringPosition = desiredHeading; //We will go with desiredHeading if wall logic doesn't change it

      //Decide if we have a wall in the way
      if(batmobile.leftDistance < left_wall_min_distance)
      {
        SerialUSB.println("Wall left!");
        if(millis() - wall_left_timestamp < left_wall_time_before_a_lot_turn)
        {
          newSteeringPosition = a_little_right;
        }
        else
        {
          //We've spent too much time near the wall, get more drastic and turn a lot away
          newSteeringPosition = a_lot_right;
        }
      }
      else //No left wall obstruction detected!
      {
        wall_left_timestamp = millis(); //Reset timestamp
      }

      if(batmobile.rightDistance < right_wall_min_distance)
      {
        SerialUSB.println("Wall right!");
        if(millis() - wall_right_timestamp < right_wall_time_before_a_lot_turn)
        {
          newSteeringPosition = a_little_left;
        }
        else
        {
          //We've spent too much time near the wall, get more drastic and turn a lot away
          newSteeringPosition = a_lot_left;
        }
      }
      else //No right wall obstruction detected!
      {
        wall_right_timestamp = millis(); //Reset timestamp
      }

      SerialUSB.print(" newSteeringPosition[");
      SerialUSB.print(newSteeringPosition);
      SerialUSB.print("]");
      SerialUSB.println();

    */

    //Start moving at 10% throttle
    //writeLocomotion(LOCO_MOTOR_SPEED, 10); //motorSpeed = 0 to 99%

    //During testing we will read the throttle and then echo it
    batmobile.throttlePosition = readLocomotion(LOCO_THROTTLE_POSITION);

    //During testing, limit to 20%
    if (batmobile.throttlePosition > 50) batmobile.throttlePosition = 50;

    writeLocomotion(LOCO_MOTOR_SPEED, batmobile.throttlePosition); //motorSpeed = 0 to 99%
  }
}

void drive(long lengthTime)
{
  long startTime = millis();
  while(millis() - startTime < lengthTime)
  {
    writeLocomotion(LOCO_MOTOR_SPEED, 35); //motorSpeed = 0 to 99%
    delay(10);    
  }
}

