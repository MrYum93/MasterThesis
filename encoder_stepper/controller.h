#ifndef CONTROLLER_H
#define CONTROLLER_H
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
# This is a script containing all functions needed to pass RoVi1*/
/* system includes */

#include <stdlib.h>

/***************************************************************************/
/* application includes */

/***************************************************************************/
/* defines */

#define true					1
#define false					0

#define CONTROLLER_INIT_OK		0

/***************************************************************************/
/* Pin defines */

#define PIN_EN      16  /*The purple wire is Enable (pin 10) (wpi 16)*/
#define PIN_DIR     1   /*The green wire is direction (pin 11) (wpi 0)*/
#define PIN_STEP    0   /*The grey wire is step (pin 12) (wpi 1)*/
                        /*The black wire is ground (pin 9)*/

/***************************************************************************/
/* global variables */

/***************************************************************************/
/* function prototypes */

int controller_init(void);
int controller_update(unsigned long enc_tics, unsigned long stp_tics);
void controller_quit(void);

/***************************************************************************/
#endif
