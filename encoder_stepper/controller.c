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
  
  detect_slip(enc_tics, stp_tics);
  printf("Encoder pulses %il, stepper pulses %ul\n", enc_tics, stp_tics);

  return 0;
}
/***************************************************************************/

void controller_quit(void)
{
	printf ("app_quit() in controller.c called\n");
}


