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
# File: main.c
# Purpose: Linux scheduler main file
# Project: Linux Scheduler
# Author: Kjeld Jensen <kjeld@cetus.dk>
# Created:  2008-10-12 Kjeld Jensen,
# Modified: 2011-02-16 Kjeld Jensen, Released under MIT license
# Modified: 2011-03-28 Kjeld Jensen, Added missing include for stdlib.h
****************************************************************************/
/* system includes */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/timeb.h>

/***************************************************************************/
/* application includes */

/*#include "app.h"*/

#include "vectornav.h"

/***************************************************************************/
/* global variables */

static sigset_t wait_mask;


int imu_response;
/***************************************************************************/
void nullhandler(int signo)
{
}
/***************************************************************************/
/* handle CTRL-C gracefully */
static void quit () /* do not add void here */
{
	/* perform application cleanup */
	imu_quit();
	/* exit */
	exit(EXIT_SUCCESS);
}
/***************************************************************************/
static int sched_init (void)
{
	int err = false;
	struct sched_param my_sched_params;
	struct itimerval interval;

	/* Setup high priority task */
	my_sched_params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	printf("sched, %d\n", sched_setscheduler(0, SCHED_FIFO, &my_sched_params));
	if (! (sched_setscheduler(0, SCHED_FIFO, &my_sched_params)))
	{
		/* lock all pages in memory */
		mlockall(MCL_CURRENT | MCL_FUTURE);

		/* implement the signal handler to handle CTRL-C gracefully */
		signal(SIGINT, quit);
		signal(SIGTERM, quit);

		/* set nullhandler on SIGALRM */
		signal(SIGALRM, nullhandler);

		/* setup timer for periodic signal */
		sigemptyset(&wait_mask);
		interval.it_value.tv_sec = 0;
		interval.it_interval.tv_sec = 0;
		interval.it_value.tv_usec = SCHED_INTERVAL;
		interval.it_interval.tv_usec = SCHED_INTERVAL;
		setitimer( ITIMER_REAL, &interval, NULL);
	}
	else
		err = true;
	return err;
}
/***************************************************************************/
int main (int argc, char **argv)
{
	
	printf("*****MAIN*****\n");
	/* initialize scheduler */
	if (! sched_init())
	{
		printf("***SCHED***\n");
		/* initialize application */
		if (imu_init() == IMU_INIT_OK)
       		    

		{
			


			while (1) //(!enc_stop) & (!stp_stop)
			{
				/* update application */
				
			
			  imu_response = imu_update();
				
				/* suspend until next event */
				sigsuspend(&wait_mask);
			}
		}
		quit();
	}

	return 0;
}
/***************************************************************************/
