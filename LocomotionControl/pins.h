//Hardware definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define THROTTLE_IN A0

//#define THROTTLE_OUT 9 //PWM base frequency is 31250 Hz, above hearing range

#define STEER_POT A1

//The rocket switch is wired super weird. The pin on side is LED common. And LED gets power from either
//of the lower lugs (+). Lug on left is wired 5V, lug on right is wired switch with a pull-down.
//So when rocket is thrown, 5V goes to switch and also to LED. When LED goes low, LED turns on.
#define MODE_SWITCH_LED 4 //Yellow
#define MODE_SWITCH 3 //Black

#define RELAY1 8
#define RELAY2 9

#define BRAKE_SWITCH 2

#define STAT 13
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

