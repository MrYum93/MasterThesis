/***************************************************************************/
/* function prototypes */

/* only for shared memory server */
int shared_memory_init (shared_memory_t **shared);
int shared_memory_remove (shared_memory_t **shared);

/* only for shared memory clients */
int shared_memory_connect (shared_memory_t **shared);
int shared_memory_disconnect (shared_memory_t **shared);

/* semaphore functions for shared memory clients */
void semaphore_acquire (int sem_id);
void semaphore_release (int sem_id);

/***************************************************************************/
/* shared_memory_init() return codes */

#define ERR_SHMEM_SEGMENT	-1
#define ERR_SHMEM_ATTACH	-2
#define ERR_SEMAPHORE		-3

/***************************************************************************/
/* shared_memory_remove() return codes */

#define ERR_SHMEM_DETACH	-1
#define ERR_SHMEM_REMOVE	-2

/***************************************************************************/
/* shared_memory_connect() return codes */

#define ERR_SHMEM_SEGMENT	-1
#define ERR_SHMEM_ATTACH	-2
#define ERR_SEMAPHORE		-3

/***************************************************************************/
