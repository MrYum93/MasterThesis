

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <time.h>
#include <math.h>

/***************************************************************************/
/* shared memory includes */

/***************************************************************************/
/* global variables */


/****************************************************************************/



int main (void)
{
    struct timespec time_now;
    clock_gettime(CLOCK_REALTIME, &time_now);
    ms = round(time_now.tv_nsec / 1000000);
    t = 1000 * time_now.tv_sec + ms;//Multiply with 1000 to go from nano secs to milli secs

    while(1):
        if 
}
/***************************************************************************/
