#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <time.h>
#include <math.h>

/***************************************************************************/
/* application includes */

#include "stepper_driver.h"
#include "read_encoder_wiringPi.h"

/***************************************************************************/
/* methods  */

/***************************************************************************/
/* shared memory includes */

/***************************************************************************/
/* variables */
volatile long signed t_stp = 0;
volatile long signed ns = 0;
unsigned long update_cnt_stp;
unsigned long update_target;
unsigned long last_target_freq;
unsigned long last_duty_cycle;
unsigned long duty_cnt = 0;
unsigned long end_of_duty;
unsigned long pulses_cnt = 0;
int step_high = 0;
int last_step_high;
long int delta_time = 0;
long signed last_time = 0;
long int freq = 1000; /*Lets step a thousan times a second*/
long int half_period = 0;
int status = 1;
struct timespec time_now;

/****************************************************************************/

int stepper_init(void){
  printf("*****stepper init begin*****\n");
  
  /* sets up the wiringPi library */
  /*if (wiringPiSetup () < 0) {
    fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
    return 1;
  }*/

  pinMode (PIN_DIR, OUTPUT) ;
  pinMode (PIN_EN, OUTPUT) ;
  pinMode (PIN_STEP, OUTPUT) ;
  /****/

  clock_gettime(CLOCK_REALTIME, &time_now);
  ns = time_now.tv_nsec;
  t_stp = ns + time_now.tv_sec*BILLION;
  
  digitalWrite(PIN_DIR, HIGH);
  digitalWrite(PIN_EN, HIGH);
  
  /** Setup **/
  printf("***stepper init done***\n");
  
  status = STP_INIT_OK;

  return status;
}

int stepper_update(int target_freq){
  
  /* The freq is 10000Hz*//*and 1/100 is 100Hz */
  /* Since it sleeps half the time, it has only half freq*/
  /*  10000/update_cnt = target_freq <=> update_cnt = (10000/target_freq)/2         */
  //Maybe only do the calculation if the target freq has changed to a new value
  
  printf("step high %d\n",step_high);

  if (last_target_freq != target_freq) {
    update_target = (10000/target_freq);
    last_target_freq = target_freq;  
  }
  
  if (step_high){
    step_high = 0;
    digitalWrite(PIN_STEP, LOW);
    pulses_cnt++; 
  }
  
  if (update_cnt_stp == 0) {
    digitalWrite(PIN_STEP, HIGH);
    step_high = 1;
  }
 
  update_cnt_stp+=1;
  if(update_cnt_stp >= update_target){
    update_cnt_stp = 0;
  }
    
    /*clock_gettime(CLOCK_REALTIME, &time_now);
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
    if (!step_high){
        digitalWrite(PIN_STEP, HIGH);
        step_high = 1;
    }
    else {
        digitalWrite(PIN_STEP, LOW);
        step_high = 0;
    }
    */
   
  
  return pulses_cnt;
}
/***************************************************************************/

void stepper_quit(void)
{
	printf ("app_quit() in stepper_driver.c called\n");
}


