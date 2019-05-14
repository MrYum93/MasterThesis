#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

/***************************************************************************/
/* application includes */

#include "velest.h"

/***************************************************************************/
/* methods  */

/***************************************************************************/
/* shared memory includes */

#define MAXITEMS 3

/***************************************************************************/
/* variables */
volatile long signed t_stp = 0;
volatile long signed ns = 0;
unsigned long update_target;
unsigned long pulses_cnt = 0;
int update_cnt_vel = 0;
int status = 1;
struct timespec time_now;
double vel = 0;

/*************************beginning of queue stuff***************************/
/* from https://www.tutorialspoint.com/data_structures_algorithms/queue_program_in_c.htm*/

double pos_l[MAXITEMS];
int front = 0;
int rear = -1;
int itemCount = 0;

double peek(void) {
   return pos_l[front];
}

int isEmpty(void) {
   return itemCount == 0;
}

int isFull(void) {
   return itemCount == MAXITEMS;
}

int size(void) {
   return itemCount;
}  

void insert(double data) {

   if(!isFull()) {
	
      if(rear == MAXITEMS-1) {
         rear = -1;            
      }       

      pos_l[++rear] = data;
      itemCount++;
   }
}

double removeData() {
   int data = pos_l[front++];
	
   if(front == MAXITEMS) {
      front = 0;
   }
	
   itemCount--;
   return data;  
}
/***************** end of queue stuff ********************/


int est_vel_init(void)
{
  printf("*****est_vel init begin*****\n");
  
  status = VEL_EST_INIT_OK;

  return status;
}

double est_vel_update(double pos)
{
  update_cnt_vel += 1;

  /* Running with 10.000 Hz run code with 10.000/100=100Hz */
  if(update_cnt_vel % 100 == 0){
    insert(pos);
    if(size() >= 3){
      vel = 1;
    }


  }

  return vel;
}

void est_vel_quit(void)
{
	printf ("est_vel_quit() in est_vel.c called\n");
}


