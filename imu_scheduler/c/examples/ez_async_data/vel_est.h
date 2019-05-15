#ifndef VEL_EST_H
#define VEL_EST_H

#define VEL_EST_INIT_OK 0
#define M_PI            3.14159265358979323846

int vel_est_init(void);
double vel_est_update(double pos);
void est_vel_quit(void);

double vel_from_pos(double pos);
double neutral_yaw_calc(double yaw);
double vel_from_pos(double pos);
double est_pos(double yaw, double neutral_yaw);



#endif