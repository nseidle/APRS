//Comes from https://forum.arduino.cc/index.php?topic=366836.0

#include "wd.h" //Comes from https://github.com/adafruit/Adafruit_SleepyDog/blob/master/utility/WatchdogSAMD.cpp

WatchdogSAMD testWD;

void setup()
{
  testWD.reset(); //pet the dog
  testWD.disable();
  
  SerialUSB.begin(115200);
  while(!SerialUSB) testWD.reset(); //pet the dog
  
  delay(200);
  SerialUSB.println("WDT demo");

  int actualTimeout = testWD.enable(2000);
  SerialUSB.print("actualTM: ");
  SerialUSB.println(actualTimeout);

  long startTime = millis(); // keep track of time since WDT enabled

  while (millis() - startTime <= 10000)
  {
    delay(10);

    //testWD.reset(); //pet the dog

    SerialUSB.print("Elapsed: "); // print the elapsed time in milliSeconds
    SerialUSB.println(millis() - startTime);
  }

  SerialUSB.println("NO WDT reset occurred");
  testWD.disable();
}

void loop()
{
  // no loop code required for the WDT test
}

