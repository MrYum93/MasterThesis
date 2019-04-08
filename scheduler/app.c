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
# File: app.c
# Purpose: Scheduled application template
# Project: Linux Scheduler
# Platform: Linux
# Author: Kjeld Jensen <kjeld@cetus.dk>
# Created:  2008-10-12 Kjeld Jensen
# Modified: 2011-02-16 Kjeld Jensen, Released under MIT license
# Modified: 2011-08-04 Kjeld Jensen, Removed reference to libusb.h
****************************************************************************/
/* system includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/***************************************************************************/
/* application includes */

#include "app.h"

/***************************************************************************/
/* defines */

/***************************************************************************/
/* variables */
unsigned long update_cnt;

/***************************************************************************/
int app_init(int argc, char **argv)
{
	int status = APP_INIT_OK;

	printf ("app_init() in app.c called\n");

	return status;
}
/***************************************************************************/
int app_update(void)
{
	update_cnt++;

	if (update_cnt % 50 == 0) /* 1/50 of 50Hz (app.h) means 1/second */
		printf ("app_update() in app.c called once per second, press CTRL-C to quit...\n");

	return false;
}
/***************************************************************************/
void app_quit(void)
{
	printf ("app_quit() in app.c called\n");
}
/***************************************************************************/
