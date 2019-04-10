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
FILE *f;
unsigned long update_cnt;

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

int enc_init(int argc, char **argv) {
  // init main
  printf("******init read_encoder*******\n");

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
  
  f = fopen("data_for_git/encoder_data.txt", "w");
  if (f == NULL)
  {
    printf("Error opening txt file!\n");
    exit(1);
  }
  
  /* print header to file */
  const char *header = "time,tics,rev,speed";
  fprintf(f, "%s\n", header);
  
  struct timespec time_now;
  struct timespec time_last;

  clock_gettime(CLOCK_MONOTONIC, &time_last);
  ms_last = round(time_last.tv_nsec / 1000000);
  t_last = 1000 * time_last.tv_sec + ms_last;//time_now.tv_nsec;


  int status = ENC_INIT_OK;

	printf ("enc_init() in read_encoder_wiringPi.c called\n");

	return status;
}

void enc_quit(void) {
  fclose(f);
	printf ("enc_quit() in read_encoder_wiringPi.c called\n");
}

// -------------------------------------------------------------------------
// main
int enc_main(void) {
  update_cnt++;
  if(update_cnt % 1000 == 0){
    
    seq = A << 1 | B;
    if (seq != old_seq){

      switch(seq){
        case 0:
        if (old_seq == 2){
          tics += 1;
          dir = 1;
        }
        else if (old_seq == 1){
          tics -= 1;
          dir = -1;
        }
        case 1:
          if (old_seq == 3){
            tics -= 1;
            dir = -1;
          }
          else if (old_seq == 0){
            tics += 1;
            dir = 1;
          }
        case 3:
          if (old_seq == 2){
            tics -= 1;
            dir = -1;
          }
          else if (old_seq == 1){
            tics += 1;
            dir = 1;
          }
        case 0b10:
          if (old_seq == 3){
            tics += 1;
            dir = 1;
          }
          else if (old_seq == 0){
            tics -= 1;
            dir = -1;
          }
          default:
            tics = tics;
      }
    }
    old_seq = seq;

    // Check for home pos
    if (Z == 0 & rev_flag == 0){
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
  //  t = time_now.tv_sec;
  /*
      //printf("time in EPOCH = %lu nanoseconds\n", (long unsigned int) t);
      printf( "time: %d\n", t );
      printf( "t_ol: %d\n", old_t );
      printf( "dir : %d\n", dir );
  */

  //    printf( "t: %d\n", t );
  //   printf( "t_last: %d\n", t_last );

    //speed over 100ms
    if (t >= t_last+100){
      speed = (tics-old_tics) / (t-t_last); // t is in ms
      printf( "Speed: %f\n", speed );
      //printf( "tics: %d\n", tics );
      old_tics = tics;
      //update the new comparison
      clock_gettime(CLOCK_MONOTONIC, &time_last);
      ms_last = round(time_last.tv_nsec / 1000000);
      t_last = 1000 * time_last.tv_sec + ms_last;//time_now.tv_nsec;
    }
    else {
      speed = speed;
    }

    // print the tics and revolutions
    fprintf(f, "%u,%d,%d,%f\n", t, tics, revolutions, speed);

    // Update variables
  /*
      clock_gettime(CLOCK_MONOTONIC, &time_last);
      ms_last = round(time_last.tv_nsec / 1000000);
      t_last = 1000 * time_last.tv_sec + ms_last;//time_now.tv_nsec;
  */
 
  }
  return 0;
}
