/***************************************************************************/
/* system includes */

#include <sys/shm.h>			/* shared memory */

/***************************************************************************/
/* application includes */

#include "sharedata.h" 			/* must be first */
#include "sharedmem.h"
#include "semaphore.h"

/***************************************************************************/
/* defines */

#define	PERMS 	(0666) 	/* everybody can read and write the shared memory */

/***************************************************************************/
/* static variables */

static int semaphore[NUMBER_OF_SEMAPHORES];

/***************************************************************************/
/* create a shared memory structure                                        */
/* do not call shared_memory_connect() after this one                      */
/***************************************************************************/
int shared_memory_init (shared_memory_t **shared)
{
	int status = 0;
	int shmid;

	/* try to get shared memory segment */
	if ((shmid=shmget(SHMKEY, sizeof(shared_memory_t), PERMS | IPC_CREAT))!= -1)
	{
		/* try to attach to the shared memory */
	  	if ((*shared = (shared_memory_t *)
    		shmat(shmid, (char *) 0, 0)) != (shared_memory_t *) -1)
   		{
   			int i;

			/* create semaphores */
			for (i=0; i<NUMBER_OF_SEMAPHORES; i++)
			{
				if ( (semaphore[i] = sem_create(SEMKEY + i, 0)) < 0)
					status = ERR_SEMAPHORE;
				sem_signal(semaphore[i]);
			}
  		}
		else
			status = ERR_SHMEM_ATTACH;
	}
	else
		status = ERR_SHMEM_SEGMENT;

	return status;
}
/***************************************************************************/
/* dispose the shared memory structure                                     */
/* call only if you previously called shared_memory_init()	               */
/***************************************************************************/
int shared_memory_remove (shared_memory_t **shared)
{
	int status = 0;
	int shmid;
	int i;

	/* remove semaphores */
	for (i=0; i<NUMBER_OF_SEMAPHORES; i++)
    	sem_rm(semaphore[i]);

	/* try to get shared memory area identifier */
	if ((shmid = shmget(SHMKEY, sizeof(shared_memory_t), 0)) >= 0)
	{

		/* try to detach shared memory associated with the shared pointer */
		if (shmdt(*shared) == 0)
		{
			/* try to remove shared memory */
			if ( shmctl(shmid, IPC_RMID, (struct shmid_ds *) 0) < 0 )
				status = ERR_SHMEM_REMOVE;
		}
		else
			status = ERR_SHMEM_DETACH;
	}
	return (status);
}
/***************************************************************************/
/* connect to an existing shared memory structure                          */
/* do not call this if you called shared_memory_init()                     */
/***************************************************************************/
int shared_memory_connect (shared_memory_t **shared)
{
	int status = 0;
	int shmid;

	/* try to get shared memory area identifier */
	if ((shmid = shmget(SHMKEY, sizeof(shared_memory_t), 0)) >= 0)
	{

		/* try to attach to the shared memory */
	  	if ((*shared = (shared_memory_t *)
    		shmat(shmid, (char *) 0, 0)) != (shared_memory_t *) -1)
   		{
			int i;

			/* try to open semaphores */
			for (i=0; i<NUMBER_OF_SEMAPHORES; i++)
			{
				if ((semaphore[i] = sem_open(SEMKEY + i)) == -1)
					status = ERR_SEMAPHORE;
  			}
  		}
		else
			status = ERR_SHMEM_ATTACH;
	}
	else
		status = ERR_SHMEM_SEGMENT;

	return (status);
}

/***************************************************************************/
/* detach shared memory structure associated with the shared pointer       */
/***************************************************************************/
int shared_memory_disconnect (shared_memory_t **shared)
{
	if (shmdt(*shared) == 0)
		return 0;
	else
		return ERR_SHMEM_DETACH;
}
/***************************************************************************/
void semaphore_acquire (int sem_id)
{
	sem_wait (semaphore[sem_id]);
}
/***************************************************************************/
void semaphore_release (int sem_id)
{
	sem_signal (semaphore[sem_id]);
}
/***************************************************************************/
