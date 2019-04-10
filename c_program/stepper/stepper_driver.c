

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <time.h>
#include <math.h>
/***************************************************************************/
/* Pin defines */
    #define PIN_DIR 0//The green wire is direction (pin 11) (wpi 0)
    #define PIN_EN 16//The purple wire is Enable (pin 10) (wpi 16)
    #define PIN_STEP 1//The grey wire is step (pin 12) (wpi 1)
                        //The black wire is ground (pin 9)
/***************************************************************************/



/***************************************************************************/
/* shared memory includes */

/***************************************************************************/
/* global variables */


/****************************************************************************/



int main (void)
{
    pinMode (PIN_DIR, OUTPUT) ;
    pinMode (PIN_EN, OUTPUT) ;
    pinMode (PIN_STEP, OUTPUT) ;
    struct timespec time_now;
    clock_gettime(CLOCK_REALTIME, &time_now);
    ms = round(time_now.tv_nsec / 1000000);
    t = 1000 * time_now.tv_sec + ms;//Multiply with 1000 to go from nano secs to milli secs
    Boolean step_high = 1
    digitalWrite(PIN_DIR, HIGH)
    digitalWrite(PIN_EN, HIGH)
    while(1)
    {
        clock_gettime(CLOCK_REALTIME, &time_now);
        ms = round(time_now.tv_nsec / 1000000);
        t = 1000 * time_now.tv_sec + ms;//Multiply with 1000 to go from nano secs to milli secs
        
        delta_time = t - last_time 
        if (delta_time > 2.5 ){ 
            last_time = t
            if (!step_high){
                digitalWrite(PIN_STEP, HIGH)
                step_high = 1
            }
            else {
                digitalWrite(PIN_STEP, LOW)
                step_high = 0
            }


            }
            
        }
}
/***************************************************************************/
