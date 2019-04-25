/***************************************************************************
# Copyright (c) 2018, Mathias W. Madsen <matam14@student.sdu.dk> <mwittenm@gmail.com>
#                     Mark Buch         <mabuc13@student.sdu.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ****************************************************************************
#
# isr4pi.c
# D. Thiebaut
# based on isr.c from the WiringPi library, authored by Gordon Henderson
# https://github.com/WiringPi/WiringPi/blob/master/examples/isr.c
#
# Compile as follows:
#
#     gcc -o read_encoder read_encoder_wiringPi.c -lwiringPi -lrt -lm
#
# Run as follows:
#
#     sudo ./isr4pi
#
# This is a script containing all functions needed to pass RoVi1
#
# Revision
# YYYY-MM-DD
# 2019-03-28 MW First version
# 2019-04-11 MW Made a header and incoorporating into a schedular
# ****************************************************************************/
/* system includes */

/***************************************************************************/
/* application includes */

#include "read_encoder_wiringPi.h"

/***************************************************************************/
/* defines */


/***************************************************************************/
/* variables */
/* the phases high=1, low=0 */
FILE *f;
unsigned long update_cnt_enc;
struct timespec time_now;
struct timespec time_last;
volatile char AB[] = "00"
volatile char A = "0";
volatile char B = "0";
volatile int Z = 1; /* is always hihg unless it is at home pos*/
volatile int seq = 0;
volatile int old_seq = 0;
volatile int delta = 0;
volatile int revolutions = 0;
volatile char rev_flag = 0;
volatile signed int dir = 0;  /* either 1[CW], 0[NOTHING] or -1[CCW]*/
volatile signed long tics = 0;
volatile signed long old_tics = 0;
volatile float speed = 0;
volatile long ms = 0;
volatile long ms_last = 0;
volatile long signed t = 0;
volatile long signed t_last = 0;
int state = 0;

/* -------------------------------------------------------------------------
   myInterrupt:  called every time an event occurs */
void aEvent(void) {
  if (digitalRead(PIN_A)){
    A = "1";
  }
  else
  {
    A = "0";
  }
  
}

void bEvent(void) {
  if (digitalRead(PIN_B)){
    B = "1";
  }
  else
  {
    B = "0";
  }
}

/* is high all the time except in home pos */
void zEvent(void) {
  Z = digitalRead(PIN_Z);
}

int init_wiring(void){
  /* sets up the wiringPi library */
  if (wiringPiSetup () < 0) {
    fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
    return 1;
  }

  /* set all pins as input */
  pinMode (PIN_A, INPUT) ;
  pinMode (PIN_B, INPUT) ;
  pinMode (PIN_Z, INPUT) ;

  /* set PINs to events generate an interrupt on high-to-low transitions
     and attach () to the interrupt */
  wiringPiISR (PIN_A, INT_EDGE_BOTH, &aEvent);
  wiringPiISR (PIN_B, INT_EDGE_BOTH, &bEvent);
  wiringPiISR (PIN_Z, INT_EDGE_BOTH, &zEvent);
  
  if(wiringPiISR(PIN_B, INT_EDGE_BOTH, &bEvent) < 0 ) {
    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
    return 1;
  }
  printf("here1\n");
  if(wiringPiISR(PIN_A, INT_EDGE_BOTH, &aEvent) < 0 ) {
    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
    return 1;
  }
  printf("here2\n");

  if(wiringPiISR(PIN_Z, INT_EDGE_BOTH, &zEvent) < 0 ) {
    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
    return 1;
  }
  printf("here3\n");

}

int enc_init(void) {
  /* init main */
  printf("******init read_encoder*******\n");

  /*init_wiring();
  sets up the wiringPi library */
  /*if (wiringPiSetup () < 0) {
    fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
    return 1;
  }*/

  /* set all pins as input */
  /*pinMode (PIN_A, INPUT) ;
  pinMode (PIN_B, INPUT) ;
  pinMode (PIN_Z, INPUT) ;*/

  /* set PINs to events generate an interrupt on high-to-low transitions
     and attach () to the interrupt */
  /*wiringPiISR (PIN_A, INT_EDGE_BOTH, &aEvent);
  wiringPiISR (PIN_B, INT_EDGE_BOTH, &bEvent);
  wiringPiISR (PIN_Z, INT_EDGE_BOTH, &zEvent);*/

  if (wiringPiISR(PIN_A, INT_EDGE_BOTH, &aEvent) < 0 ) {
    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
    return 1;
  }

  if (wiringPiISR(PIN_B, INT_EDGE_BOTH, &bEvent) < 0 ) {
    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
    return 1;
  }

  if (wiringPiISR(PIN_Z, INT_EDGE_BOTH, &zEvent) < 0 ) {
    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
    return 1;
  }

  AB = A + B;

/*
  f = fopen("../data_for_git/encoder_data.txt", "w");
  if (f == NULL)
  {
    printf("Error opening txt file!\n");
    exit(1);
  }
  */
  /* print header to file 
  const char *header = "time,tics,rev,speed";
  fprintf(f, "%s\n", header);

  clock_gettime(CLOCK_MONOTONIC, &time_last);
  ms_last = round(time_last.tv_nsec / 1000000);
  t_last = 1000 * time_last.tv_sec + ms_last;/*time_now.tv_nsec;

  int status = ENC_INIT_OK;

  printf ("enc_init() in read_encoder_wiringPi.c called\n");
  */
  return ENC_INIT_OK;
}

void enc_quit(void) {
  fclose(f);
  printf ("enc_quit() in read_encoder_wiringPi.c called\n");
}

// -------------------------------------------------------------------------
// main
int enc_update(void) {

  switch (AB)
  {
    case "00":
      if (B == "1") {
        tics++;
      }
      else if (A == "1") {
        tics--;
      }
      break;
    case "01":
      if (A == "1") {
        tics++;
      }
     else if (B == "0") {
        tics--;
      }
      break;
    case "11":
      if (B == "0") {
        tics++;
      }
      if (A == "0") {
        tics--;
      }
      break;
    case "10":
       if (A == "0") {
        tics++;
        }
      if (B == "1") {
        tics--;
        }
      break;
    default:
      break;
  }
  AB = A + B;

  printf("state %d", state);

  /*
  prev_A = A;
  A = digitalRead(PIN_A);
  prev_B = B;
  B = digitalRead(PIN_B);
  if ((prev_B == 0) && (A == 1) && (B == 1) {
    tics++;
  }

  if ((prev_A == 1) && (A == 0) && (B == 1) {
    tics++;
  }

  if ((prev_B == 1) && (A == 0) && (B == 0) {
    tics++;
  }

  if ((prev_A == 0) && (A == 1) && (B == 0) {
    tics++;


  if ((prev_A == 0) && (B == 1) && (A == 1) {
    tics--;
  }

  if ((prev_B == 1) && (B == 0) && (A == 1) {
    tics--;
  }

  if ((prev_A == 1) && (B == 0) && (A == 0) {
    tics--;
  }

  if ((prev_B == 0) && (B == 1) && (A == 0) {
    tics--;
  }
  
  
  update_cnt_enc++;
  /*printf("update_cnt\n");*/
  /*The freq is 2000Hz*//*and 1/50 of 2000Hz is */
/*
  if(update_cnt_enc % (1) == 0){
    seq = (A ^ B) | B << 1;  // get sequence according to documentation in drive
  /*if(update_cnt_enc % (1) == 0){
    seq = (A ^ B) | B << 1;  // get sequence according to documentation in drive
    delta = (seq - old_seq) % 4;
    if (delta == 0){
      //No change
      tics = tics;
      dir = 0;
    }
    if (delta == 1){
      //CW
      printf("one CW, %d\n", tics);
      tics += 1;
      dir = 1;
      printf("after one CW, %d\n", tics);
    }
    if (delta == 2){
      //skipped a single read so assume it goes same dir as previously two times
      tics += dir*2;
    }
    if (delta == 3){
      //CCW
      printf("one CCW, %d", tics);
      tics -= 1;
      dir = -1;
      printf("after one CCW, %d\n", tics);
    }
    // save last seq to compare
    old_seq = seq;

    seq = (A ^ B) | B << 1;  // get sequence according to documentation in drive
    delta = (seq - old_seq) % 4;
    switch(delta){
    case 0:
      //No change
      tics = tics;
      dir = 0;
    case 1:
      //CW
      //printf("one CW, %d\n", tics);
      tics += 1;
      dir = 1;
      //printf("after one CW, %d\n", tics);
    case 2:
      //skipped a single read so assume it goes same dir as previously two times
      tics += dir*2;
    case 3:
      //CCW
      //printf("one CCW, %d", tics);
      tics -= 1;
      dir = -1;
      //printf("after one CCW, %d\n", tics);
    }
    // save last seq to compare
    old_seq = seq;
*/

    /*seq = (A << 1) | B;
    /*printf("seq, %d\n", seq);*/
    /*
    if (seq != old_seq){
      switch(seq){
        case 0:
        if (old_seq == 2){
          tics -= 1;
          dir = -1;
        }
        else if (old_seq == 1){
          tics += 1;
          dir = 1;
        }
        case 1:
          if (old_seq == 3){
            tics += 1;
            dir = 1;
          }
          else if (old_seq == 0){
            tics -= 1;
            dir = -1;
          }
        case 3:
          if (old_seq == 2){
            tics += 1;
            dir = 1;
          }
          else if (old_seq == 1){
            tics -= 1;
            dir = -1;
          }
        case 2:
          if (old_seq == 3){
            tics -= 1;
            dir = -1;
          }
          else if (old_seq == 0){
            tics += 1;
            dir = +1;
          }
          default:
            tics = tics;
	    dir = 0;
      }
    }
    old_seq = seq;
  
    // Check for home pos
    if ((Z == 0) & (rev_flag == 0)){
      printf("I am home now, %d\n", revolutions);
      revolutions += dir;
      rev_flag = 1;
    }
    else if (Z == 1){
      rev_flag = 0;
    }

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    ms = round(time_now.tv_nsec / 1000000);
    t = 1000 * time_now.tv_sec + ms;//time_now.tv_nsec;
    t = time_now.tv_sec;
      //printf("time in EPOCH = %lu nanoseconds\n", (long unsigned int) t);
      printf( "time: %d\n", t );
      printf( "t_ol: %d\n", old_t );
      printf( "dir : %d\n", dir );

    printf( "t: %ld\n", t );
    printf( "t_last: %ld\n", t_last );

    /*speed over 100ms
    if (t >= t_last+100){
      speed = (tics-old_tics) / (t-t_last); // t is in ms
      //printf( "Speed: %f\n", speed );
      printf( "tics: %ld\n", tics );
      printf("rev: %d\n", revolutions);
      old_tics = tics;
      //update the new comparison
      clock_gettime(CLOCK_MONOTONIC, &time_last);
      ms_last = round(time_last.tv_nsec / 1000000);
      t_last = 1000 * time_last.tv_sec + ms_last;//time_now.tv_nsec;
    }
    else {
      speed = speed;
    }
    */

    /* print the tics and revolutions
    fprintf(f, "%li,%ld,%d,%f\n", t, tics, revolutions, speed);

    /* Update variables */
    /*
    clock_gettime(CLOCK_MONOTONIC, &time_last);
    ms_last = round(time_last.tv_nsec / 1000000);
    t_last = 1000 * time_last.tv_sec + ms_last;//time_now.tv_nsec;
    
 
  */
  return tics;
}
