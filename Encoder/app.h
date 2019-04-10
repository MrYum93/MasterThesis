/****************************************************************************
# Linux Scheduler
# Copyright (c) 2008-2011 Kjeld Jensen <kjeld@cetus.dk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
*****************************************************************************
# File: app.h
# Purpose: Scheduled application template header file
# Project: Linux Scheduler
# Author: Kjeld Jensen <kjeld@cetus.dk>
# Created:  2008-10-12 Kjeld Jensen,
# Modified: 2011-02-16 Kjeld Jensen, Released under MIT license
****************************************************************************/
/* system includes */

#include <stdlib.h>

/***************************************************************************/
/* application includes */

/***************************************************************************/
/* defines */

#define true					1
#define false					0

#define SCHED_INTERVAL 			2e4; /* 50 Hz */

#define APP_INIT_OK				0

/***************************************************************************/
/* global variables */

/***************************************************************************/
/* function prototypes */

int app_init(int argc, char **argv);
int app_update(void);
void app_quit(void);

/***************************************************************************/
