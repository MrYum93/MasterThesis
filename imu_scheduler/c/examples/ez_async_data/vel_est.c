#include <stdio.h>
#include <time.h>
#include "vel_est.h"

#define MAXITEMS 3

int est_vel_cnt = 0;
int i, tmp, array_pos;
int n = MAXITEMS;
double vel = 0.0;
double pos_l[MAXITEMS];
struct timespec time_now;
struct timespec time_last;
volatile long ms_now = 0;
volatile long ms_last = 0;
volatile long signed t_now = 0;
volatile long signed t_last = 0;


/* This method is used to shift the pos list, and then insert a 
*  double element to a list and then */ 
double insert(double pos){  
  for(i=0;i<n-1;i++)
  {
    pos_l[i]=pos_l[i+1];
  }
  pos_l[n-1]=pos;
}

int estVelInit(void)
{
  return 0;
}

double est_vel_update(double pos)
{
  est_vel_cnt += 1;

  if((est_vel_cnt % 10) == 0)
  {
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    ms_now = round(time_now.tv_nsec / 1000000); /* is nano sec with 1 billion and mili with 1 mil*/
    t_now = 1000 * time_now.tv_sec + ms_now;//time_now.tv_nsec;

    insert(pos);
    /* this makes it so elem 0 is the newest and elem 2 is oldest */
    printf("pos 0 %f\n", pos_l[0]);
    printf("pos 2 %f\n\n", pos_l[2]);
    vel = (pos_l[2]-pos_l[0])/(2*(t_now-t_last));
    printf("h %lu\n", (t_now-t_last));
    printf("vel 0 %f\n\n\n", vel);

    ms_last = ms_now;
    t_last = t_now;
  }
  return vel;
}

void est_vel_quit(void)
{
  printf("quit est_vel!");
}

/*************************beginning of queue stuff***************************/
/* from https://www.tutorialspoint.com/data_structures_algorithms/queue_program_in_c.htm*/
/* 
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
} */
/***************** end of queue stuff ********************/
