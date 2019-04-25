#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <time.h>
#include <math.h>

/***************************************************************************/
/* application includes */

#include "controller.h"


/***************************************************************************/
/* methods  */



/***************************************************************************/
/* shared memory includes */

/***************************************************************************/
/* variables */

struct timespec time_now;
double theta_enc = 0;
double theta_stp = 0;
int state_controller = 1;
int stepper_freq = 100;
int delta_vel = 10;
unsigned long accelerate_cnt = 0;

/****************************************************************************/

int controller_init(void){
  printf("*****controller init begin*****\n");
  
  /* sets up the wiringPi library */
  /*if (wiringPiSetup () < 0) {
    fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
    return 1;
  }*/

  /****/

/*  clock_gettime(CLOCK_REALTIME, &time_now);
  ns = time_now.tv_nsec;
  t_stp = ns + time_now.tv_sec*BILLION;
*/
  /** Setup **/
  printf("***controller init done***\n");

  return CONTROLLER_INIT_OK;
}

int detect_slip(unsigned long enc_tics, unsigned long stp_tics){
  //First transfer both encoder and stepper tics to theta
  theta_enc = enc_tics/524;
  theta_stp = stp_tics/70; //this value is to be determined
  //Then return the difference between theta maybe with some error thresholding


}

int controller_update(signed long enc_tics, unsigned long stp_tics){
  
  switch (state_controller)
  {
    case 0: /*Standby state*/
      stepper_freq = 0;
      break;
    
    case 1: /*accelerate state*/
      accelerate_cnt++;
      if (accelerate_cnt % 20000 == 0){
        accelerate_cnt = 0;
        stepper_freq = stepper_freq;//delta_vel;
        state_controller = 2;
      }
      break;
    
    case 2: /*braking state*/
      accelerate_cnt++;
      if (accelerate_cnt % 1000 == 0) {
        stepper_freq -= delta_vel;
        accelerate_cnt = 0;
      }
      break;
  
    default:
      break;
  }
  
  printf("Stepper vel %d", stepper_freq);
  return stepper_freq;
}
/***************************************************************************/

void controller_quit(void)
{
	printf ("app_quit() in controller.c called\n");
}


