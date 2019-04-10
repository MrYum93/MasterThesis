#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <time.h>
#include <math.h>

#include "stepper_driver.h"
/***************************************************************************/
/* Pin defines */

#define PIN_EN 16//The purple wire is Enable (pin 10) (wpi 16)
#define PIN_DIR 1//The green wire is direction (pin 11) (wpi 0)
#define PIN_STEP 0//The grey wire is step (pin 12) (wpi 1)
                        //The black wire is ground (pin 9)
#define BILLION  1000000000L;
/***************************************************************************/
/* methods  */
int stepper_init();

/***************************************************************************/
/* shared memory includes */

/***************************************************************************/
/* variables */
volatile long long signed t = 0;
volatile long signed ns = 0;
int step_high = 0;
long int delta_time = 0;
long long signed last_time = 0;
long long int freq = 1000; /*Lets step a thousan times a second*/
long long int half_period = 0;
int status = 1;
/****************************************************************************/

int stepper_init(void){
  printf("*****stepper main*****\n");
  //int tmp = stepper_init();
  /** Setup **/
  printf("***stepper init done***\n");
  // sets up the wiringPi library
  if (wiringPiSetup () < 0) {
    fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
    return 1;
  }

  pinMode (PIN_DIR, OUTPUT) ;
  pinMode (PIN_EN, OUTPUT) ;
  pinMode (PIN_STEP, OUTPUT) ;
  /****/


  struct timespec time_now;
  clock_gettime(CLOCK_REALTIME, &time_now);
  ns = time_now.tv_nsec;
  t = ns + time_now.tv_sec*BILLION;
  
  digitalWrite(PIN_DIR, HIGH);
  digitalWrite(PIN_EN, HIGH);
  printf("*enter while*\n");
  status = 1;
  return status;
}

int stepper_update(void){
  clock_gettime(CLOCK_REALTIME, &time_now);

  ns = time_now.tv_nsec;
  t = ns + time_now.tv_sec*BILLION;
  delta_time = t - last_time;
  half_period = (1/freq)*0.5
  if (delta_time >= half_period) {
    printf("delta time %li\n", delta_time);
    last_time = t;
    if (!step_high){
      digitalWrite(PIN_STEP, HIGH);
      step_high = 1;
    }
    else {
      digitalWrite(PIN_STEP, LOW);
      step_high = 0;
    }
  }
return 0;

}
/***************************************************************************/

void stepper_quit(void)
{
	printf ("app_quit() in stepper_driver.c called\n");
}


