#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <time.h>
#include <math.h>
/***************************************************************************/
/* Pin defines */

#define PIN_EN 16//The purple wire is Enable (pin 10) (wpi 16)
#define PIN_DIR 1//The green wire is direction (pin 11) (wpi 0)
#define PIN_STEP 0//The grey wire is step (pin 12) (wpi 1)
                        //The black wire is ground (pin 9)
/***************************************************************************/
/* methods  */
int stepper_init();

/***************************************************************************/
/* shared memory includes */

/***************************************************************************/
/* variables */
volatile long long signed t = 0;
volatile long signed ms = 0;

/****************************************************************************/

int main(void){
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
  ms = round(time_now.tv_nsec / 1000000);
  t = 1000 * time_now.tv_sec + ms;//Multiply with 1000 to go from nano secs to milli secs
  int step_high = 0;
  long int delta_time = 0;
  long long signed last_time = 0;
  digitalWrite(PIN_DIR, HIGH);
  digitalWrite(PIN_EN, HIGH);
  printf("*enter while*\n");
  while(1) {
    clock_gettime(CLOCK_REALTIME, &time_now);
    ms = round(time_now.tv_nsec / 1000000);
    t = 1000 * time_now.tv_sec + ms;//Multiply with 1000 to go from nano secs to milli secs
    delta_time = t - last_time;
    if (delta_time > 2) {
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
  }
  return 0;
}
/***************************************************************************/

int stepper_init(){
  printf("***stepper init done***");
  // sets up the wiringPi library
  if (wiringPiSetup () < 0) {
    fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
    return 1;
  }

  pinMode (PIN_DIR, OUTPUT) ;
  pinMode (PIN_EN, OUTPUT) ;
  pinMode (PIN_STEP, OUTPUT) ;


  return 0;
}

