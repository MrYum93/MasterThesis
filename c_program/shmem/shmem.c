/***************************************************************************/
/* system includes */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>		/* signal(), SIGINT */
#include <time.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sched.h>

/***************************************************************************/
/* application includes */

#include "sharedata.h"
#include "sharedmem.h"

/***************************************************************************/
/* defines */

#define false				0
#define true				1

#define SCHED_INTERVAL  	1e4 /**< schedule interval set to 100 Hz */

/***************************************************************************/
/* variables */

shared_memory_t *shared;
static sigset_t wait_mask;

/***************************************************************************/
static void shared_data_reset (void)
{
	/* fill entire struct with zero's */
	memset (shared, 0, sizeof (shared_memory_t));
}
/***************************************************************************/
void nullhandler(int signo)
{
}
/***************************************************************************/
/* handle CTRL-C gracefully */

static void quit (int s)
{
	int err;

	/* release shared memory and semaphores */
	err = shared_memory_remove (&shared);

	switch (err)
	{
		case ERR_SHMEM_DETACH:
			printf("shmem: Cannot detach shared memory segment\n");
			break;

		case ERR_SHMEM_REMOVE:
			printf("shmem: Cannot remove shared memory segment\n");
			break;
	}

	/* exit */
	exit (EXIT_SUCCESS);
}
/***************************************************************************/
static int sched_init (void)
{
	int err = false;
	struct sched_param my_sched_params;
	struct itimerval interval;

	/* Setup high priority task */
	my_sched_params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	if (! sched_setscheduler(0, SCHED_FIFO, &my_sched_params))
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
	{
		printf ("shmem: Scheduling error\n");
		err = true;
	}
	return err;
}
/***************************************************************************/
int main(int argc, char *argv[])
{
	int err;

	/* create shared memory structure */
	err = shared_memory_init (&shared);

	if (! err)
	{
		printf ("shmem: Started, press CTRL-C to quit\n");

		/* initialize the memory */
		shared_data_reset ();

		/* initialize scheduler */
		if (! sched_init())
		{
			/* Wait untill user press CTRL-C */
			while(true)
			{
				/* suspend until next event */
				sigsuspend(&wait_mask);
			}
		}
	}
	else
	{
		/* print error message */
		switch (err)
		{
			case ERR_SHMEM_SEGMENT:
				printf ("shmem: Cannot create shared memory segment\n");
				break;

			case ERR_SHMEM_ATTACH:
				printf("shmem: Cannot attach shared memory segment\n");
				break;

			case ERR_SEMAPHORE:
				printf("shmem: Cannot open semaphore\n");
				break;
		}
	}

	return 0;
}
/***************************************************************************/
