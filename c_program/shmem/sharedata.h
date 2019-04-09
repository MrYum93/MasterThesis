/***************************************************************************/

#define SEMKEY ((key_t) 7895)		/* semaphore ID key */

/* we use a semaphore for each module to make sure that only one process
at a time writes to the shared memory area for this module */

#define NUMBER_OF_SEMAPHORES	1 /* must correspond to the defines below */

#define SEM_STEPPER_VEL             0       /* Speed control semaphore */
			    	     	 

/***************************************************************************/
/* shared memory */

#define SHMKEY ((key_t) 7890)		/* shared memory ID key */

/***************************************************************************/
/* Vehicle struct */

typedef struct
{
	double velocity

} stepper_t;

/***************************************************************************/
/* GPS struct */

typedef struct
{
	char upd;						/* incremented on updates */
	double lat;						/* position latitude */
	double lon;						/* position longitude */
} gps_t;

/***************************************************************************/
/** shared memory struct */

typedef struct
{
	stepper_t		stepper;		/* stepper struct */
} shared_memory_t;

/***************************************************************************/
