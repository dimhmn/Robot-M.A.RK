/*
Grove LED Bar - Basic Control Example
This example will show you how to use the setBits() function of this library.
Set any combination of LEDs using 10 bits.
Least significant bit controls the first LED.

The setBits() function sets the current state, one bit for each LED.
First 10 bits from the right control the 10 LEDs.

eg. 0b00000jihgfedcba
a = LED 1, b = LED 2, c = LED 3, etc.

dec    hex     binary
0    = 0x0   = 0b000000000000000 = all LEDs off
5    = 0x05  = 0b000000000000101 = LEDs 1 and 3 on, all others off
341  = 0x155 = 0b000000101010101 = LEDs 1,3,5,7,9 on, 2,4,6,8,10 off
1023 = 0x3ff = 0b000001111111111 = all LEDs on
                      |        |
                      10       1

The bits >10 are ignored, shown here as x: 0bxxxxx0000000000
*/

#include <Grove_LED_Bar.h>

Grove_LED_Bar bar(5, 4, 0);  // Clock pin, Data pin, Orientation

void setup()
{
  // nothing to initialize
  bar.begin();
  Serial.begin(115200);
}

void loop()
{
    long  cptTension=0;
    int affichageTension;
    long led;
    
    cptTension= analogRead(A0);
    cptTension = (3*cptTension*4980/(1023));
 
    affichageTension = (((78*cptTension)-493)/1000);
    affichageTension = (affichageTension/100);
    led=0;
    for(int i = 0; i<affichageTension;i++){
      led |= 1<<i;
    }
    bar.setBits(led);  
    
}
