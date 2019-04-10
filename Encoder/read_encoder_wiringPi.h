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
# This is a script containing all functions needed to pass RoVi1

# NOT YET: Rest of lectures...

# Revision
# YYYY-MM-DD
# 2018-11-07 MW First version
# *****************************************************************************/
/* system includes */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <time.h>
#include <math.h>

/***************************************************************************/
/* application includes */

/***************************************************************************/
/* defines */

#define true					1
#define false					0

#define SCHED_INTERVAL 			1e3; /* in microsec */

#define ENC_INIT_OK				0

#define PIN_A                   27
#define PIN_B                   24
#define PIN_Z                   23
#define BILLION                 1000000000L

/***************************************************************************/
/* global variables */

// the phases high=1, low=0
volatile int A = 0;
volatile int B = 0;
volatile int Z = 1; // is always hihg unless it is at home pos
volatile int seq = 0;
volatile int old_seq = 0;
volatile int delta = 0;
volatile int revolutions = 0;
volatile char rev_flag = 0;
volatile signed int dir = 0;  // either 1 [CW], 0[NOTHING] or -1 [CCW]
volatile signed long int tics = 0;
volatile signed long int old_tics = 0;
volatile float speed = 0;
volatile long ms = 0;
volatile long ms_last = 0;
volatile long signed int t = 0;
volatile long signed int t_last = 0;


/***************************************************************************/
/* function prototypes */

int enc_init(int argc, char **argv);
int enc_update(void);
void enc_quit(void);
void aEvent(void);
void bEvent(void);
void zEvent(void);


/***************************************************************************/
