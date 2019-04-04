/*
isr4pi.c
D. Thiebaut
based on isr.c from the WiringPi library, authored by Gordon Henderson
https://github.com/WiringPi/WiringPi/blob/master/examples/isr.c

Compile as follows:

    gcc -o read_encoder read_encoder_wiringPi.c -lwiringPi -lrt -lm

Run as follows:

    sudo ./isr4pi

 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <time.h>
#include <math.h>

// Use physical GPIO Pin 36, 35 and 33, which is Pin 27, 24 and 23 for wiringPi library
#define PIN_A 27
#define PIN_B 24
#define PIN_Z 23
#define BILLION 1000000000L


// the phases high=1, low=0
volatile int A = 0;
volatile int B = 0;
volatile int Z = 1; // is always hihg unless it is at home pos
volatile int seq = 0;
volatile int old_seq = 0;
volatile int delta = 0;
volatile int revolutions = 0;
volatile char rev_flag = 0;
volatile signed int dir = 0;  // either 1 [CW] or -1 [CCW]
volatile signed long int tics = 0;
volatile long ms = 0;
volatile long unsigned int t = 0;

//struct timespec {
//        time_t   tv_sec;        /* seconds */
//        long     tv_nsec;       /* nanoseconds */
//};


// -------------------------------------------------------------------------
// myInterrupt:  called every time an event occurs
void aEvent(void) {
  A = digitalRead(PIN_A);
}

void bEvent(void) {
  B = digitalRead(PIN_B);
}

void zEvent(void) {
  // is high all the time except in home pos
  Z = digitalRead(PIN_Z);
}


// -------------------------------------------------------------------------
// main
int main(void) {
  
  // sets up the wiringPi library
  if (wiringPiSetup () < 0) {
    fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
    return 1;
  }
  
  // set all pins as input
  pinMode (PIN_A, INPUT) ;
  pinMode (PIN_B, INPUT) ;
  pinMode (PIN_Z, INPUT) ;

  // set Pin 17/0 generate an interrupt on high-to-low transitions
  // and attach myInterrupt() to the interrupt
  if ( wiringPiISR (PIN_A, INT_EDGE_BOTH, &aEvent) < 0 ) {
    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
    return 1;
  }
  if ( wiringPiISR (PIN_B, INT_EDGE_BOTH, &bEvent) < 0 ) {
    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
    return 1;
  }
  if ( wiringPiISR (PIN_Z, INT_EDGE_BOTH, &zEvent) < 0 ) {
    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
    return 1;
  }
  
  FILE *f = fopen("data_for_git/encoder_data.txt", "w");
  if (f == NULL)
  {
    printf("Error opening txt file!\n");
    exit(1);
  }
  
  /* print header to file */
  const char *header = "time,tics,rev";
  fprintf(f, "%s\n", header);
  
  struct timespec time_now;

  while ( 1 ) {
    seq = (A ^ B) | B << 1;  // get sequence according to documentation in drive
    delta = (seq - old_seq) % 4;
    if (delta == 0){
      //No change
      tics = tics;
    }
    else if (delta == 1){
      //CW
      tics += 1;
      dir = 1;
    }
    else if (delta == 2){
      //skipped a single read so assume it goes same dir as previously two times
      tics += dir*2;
    }
    else if (delta == 3){
      //CCW
      tics -= 1;
      dir = -1;
    }
    // save last seq to compare
    old_seq = seq;
    
    // Check for home pos
    if (Z == 0 & rev_flag == 0){
      printf("I am home now");
      revolutions += dir;
      rev_flag = 1;
    }
    else{
      rev_flag = 0;
    }
    
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    ms = round(time_now.tv_nsec / 1000000);
    t = 1000 * time_now.tv_sec + ms;//time_now.tv_nsec;
    //printf("time in EPOCH = %lu nanoseconds\n", (long unsigned int) t);

    
    // print the tics and revolutions
    fprintf(f, "%u,%d,%d\n", t, tics, revolutions);

    
   /* printf( "%d\n", A );
    printf( "%d\n", B );
    printf( "%d\n", Z );*/
    printf("tics: %d\n", tics);
    //delay( 200 ); // wait 0.2 second
  }
  
  fclose(f);

  return 0;
}
