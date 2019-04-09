/***************************************************************************/
/** function prototypes */

int  sem_create(key_t key, int initval);
int  sem_open(key_t key);
int  sem_rm(int id);
int  sem_close(int id);
void sem_wait(int id);
void sem_signal(int id);
void sem_op(int id, int value);

/***************************************************************************/
