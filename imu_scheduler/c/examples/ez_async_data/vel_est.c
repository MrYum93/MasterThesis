#include <stdio.h>
#include <time.h>
#include "vel_est.h"

#define MAXITEMS 3


double arm_ang = 90; /* measure this from arm in deg*/
double arm_length = 0.39; /* measured in m */
double half_dist_poles = 2.215; /* measured in m */
double neutral_rope_length = 2.215; /* measured in m */

double pos_l[MAXITEMS];

int est_vel_cnt = 0;
double a_y, a_x, r_y, y_pos_est;
double start_ang, yaw_offset, neutral_yaw;
double pos;
double vel = 0.0;
double return_vel;
struct timespec time_now;
struct timespec time_last;
volatile long ms_now = 0;
volatile long ms_last = 0;
volatile long signed t_now = 0;
volatile long signed t_last = 0;


/* This method is used to shift the pos list, and then insert a 
*  double element to a list and then */ 
double * insert(double * list_, double elem_){
  int n = sizeof(list_)/sizeof(list_[0]);
  int i;
  int tmp;
  for(i=0;i<n-1;i++)
  {
    list_[i]=list_[i+1];
  }
  list_[n-1]=elem_;

  return pos_l;
}

double est_pos(double yaw_, double neutral_yaw_)
{
  double rad_yaw = yaw_;
  double rad_neutral_yaw = neutral_yaw_;
  rad_yaw *= M_PI/180;
  rad_neutral_yaw  *= M_PI/180;

  yaw_offset = rad_neutral_yaw - rad_yaw;

//  printf("offset, %f\n", yaw_offset);

  a_y = arm_length * sin(rad_yaw);
  a_x = arm_length * cos(rad_yaw);
  /* rope in y dir */
  r_y = sqrt(fabs((neutral_rope_length * neutral_rope_length - (half_dist_poles - a_x ) * (half_dist_poles - a_x))));

  y_pos_est = arm_length - (a_y - r_y);
  
//  printf("y_pos_est, %f\n", y_pos_est);

  return y_pos_est;
}

double neutral_yaw_calc(double yaw)
{
   /*todo here:
   * calc the neutra yaw based on a sliding window */
  neutral_yaw = yaw - (M_PI/2);
  return neutral_yaw;
}

double vel_est_update(double yaw_)
{
  est_vel_cnt += 1;

  if((est_vel_cnt % 100) == 0)
  {
    // /* State machine that goes from monitor where the neutral yaw is updated, 
    // * and goes to vel estimate of FW */
    // neutral_yaw = neutral_yaw_calc(yaw_);
    // pos = est_pos(yaw_, neutral_yaw);
    vel = vel_from_pos(yaw_);
    // return_vel = vel;//*1000.0;
  }
  // printf("vel in submain, %f\n", pos);

  return return_vel;
}


int vel_est_init(void)
{
  return 0;
}

double vel_from_pos(double pos)
{
  clock_gettime(CLOCK_MONOTONIC, &time_now);
  ms_now = round(time_now.tv_nsec / 1000000); /* is nano sec with 1 billion and mili with 1 mil*/
  t_now = 1000 * time_now.tv_sec + ms_now;//time_now.tv_nsec;  
  
  double sub_vel;

  insert(pos_l, pos);
  /* this makes it so elem 0 is the newest and elem 2 is oldest */
  printf("pos 0 %f\n", pos_l[0]);
  printf("pos 2 %f\n\n", pos_l[2]);
  sub_vel = (pos_l[2]-pos_l[0])/(2*(t_now-t_last));
  /* printf("h %lu\n", (t_now-t_last));
  printf("vel 0 %f\n\n\n", vel);*/
  ms_last = ms_now;
  t_last = t_now;
  return sub_vel;
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
