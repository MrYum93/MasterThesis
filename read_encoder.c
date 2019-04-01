 /*
 *      link.c:
 *      blinks the first LED
 *      Gordon Henderson, projects@drogon.net
 */

#include <stdio.h>
#include <wiringPi.h>

int main(void)
{
  printf("Raspberry Pi blink\n");

  if (wiringPiSetup() == -1)
    return 1 ;

  pinMode(25, OUTPUT) ;         // aka BCM_GPIO pin 17

  for (;;)
  {
    digitalWrite(25, 1);       // On
    delay (100);               // mS
    printf("turned on\n");    // print on
    digitalWrite (25, 0);       // Off
    delay (100);
  }
  return 0 ;
}
