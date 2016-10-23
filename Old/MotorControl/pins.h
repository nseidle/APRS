//Hardware definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define THROTTLE_IN A3
#define THROTTLE_OUT 9 //PWM base frequency is 31250 Hz, above hearing range
#define THROTTLE_OUT_VCC 8

#define STEER_POT A0
#define STEER_POT_LOW A1
#define STEER_POT_HIGH A2

#define MODE_SWITCH_LED 2 //Yellow
#define MODE_SWITCH 3 //Black
#define MODE_SWITCH_HIGH 4 //Red
#define MODE_SWITCH_LOW 12

//The rocket switch is wired super weird. The pin on side is LED common. And LED gets power from either
//of the lower lugs (+). Lug on left is wired 5V, lug on right is wired switch with a pull-down.
//So when rocket is thrown, 5V goes to switch and also to LED. When LED goes low, LED turns on.



#define RELAY1 6
#define RELAY2 5

#define BRAKE_SWITCH_LOW 10
#define BRAKE_SWITCH 11

#define STAT 13
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

