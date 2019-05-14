#ifndef VEL_EST_H
#define VEL_EST_H

#define VEL_EST_INIT_OK 0

int estVelInit(void);
double est_vel_update(double pos);
void est_vel_quit(void);


#endif