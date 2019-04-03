// event.c
//
// Example program for bcm2835 library
// Event detection of an input pin
//
// After installing bcm2835, you can build this 
// with something like:
// gcc -o event event.c -l bcm2835 
// sudo ./event
//
// Or you can test it before installing with:
// gcc -o event -I ../../src ../../src/bcm2835.c event.c
// sudo ./event
//
// Links to make life easier!
// https://www.airspayce.com/mikem/bcm2835/group__gpio.html
// https://www.airspayce.com/mikem/bcm2835/group__constants.html
//
// Author: Mike McCauley
// Copyright (C) 2011 Mike McCauley
// $Id: RF22.h,v 1.21 2012/05/30 01:51:25 mikem Exp $

#include <bcm2835.h>
#include <stdio.h>

// Input on RPi pin GPIO 15. Change this to the correct pin
// pin25 is an led for refference
#define A RPI_GPIO_P1_15
#define B RPI_GPIO_P1_15
#define Z RPI_GPIO_P1_15

void aEvent(void)
{
    // Read some data
    uint8_t value = bcm2835_gpio_lev(A);
    printf("read from pin A: %d\n", value);

}

int main(int argc, char **argv)
{
  printf("Raspberry Pi blink\n");

  if (!bcm2835_init())
        return 1;
  
   // Set RPI pin P1-15 to be an input
   bcm2835_gpio_fsel(A, BCM2835_GPIO_FSEL_INPT);
   bcm2835_gpio_fsel(B, BCM2835_GPIO_FSEL_INPT);
   bcm2835_gpio_fsel(Z, BCM2835_GPIO_FSEL_INPT);
   //  with a pulldown
   bcm2835_gpio_set_pud(A, BCM2835_GPIO_PUD_DOWN);
   bcm2835_gpio_set_pud(B, BCM2835_GPIO_PUD_DOWN);
   bcm2835_gpio_set_pud(Z, BCM2835_GPIO_PUD_DOWN);
   // And a Asynchronous Rising and Falling Detect enable
   bcm2835_gpio_aren(A);
   bcm2835_gpio_afen(A);   
   bcm2835_gpio_aren(B);
   bcm2835_gpio_afen(B);   
   bcm2835_gpio_aren(C);
   bcm2835_gpio_afen(C);

   
   while (1)
   {
       if (bcm2835_gpio_eds(PIN))
       {
           // Now clear the eds flag by setting it to 1
           bcm2835_gpio_set_eds(PIN);
           printf("low event detect for pin 15\n");
       }
       // wait a bit
        delay(500);
   }
   
   bcm2835_close();
   return 0;
}