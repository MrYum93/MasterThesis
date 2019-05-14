#ifndef VEL_EST_H
#define VEL_EST_H

#define VEL_EST_INIT_OK 0

int estVelInit(void);
double vel_from_pos(double pos);
double vel_est_update(double pos);
double neutral_yaw_calc(double yaw);
void est_vel_quit(void);
double vel_from_pos(double pos);
double est_pos(double yaw, double neutral_yaw);



#endif