//---------------- PARAMETERS.H ----------------------- 
#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <rtai_msg.h>
#include <sys/io.h>
#include <signal.h>
#include <rtai_mbx.h>

//lib2share.h and parameters.h must be on the same Directory
#include "lib2share.h"


#define TICK_TIME 100000000
#define CNTRL_TIME 50000000

#define TASK_PRIORITY 1

#define STACK_SIZE 10000

#define SEN_SHM 121111
#define ACT_SHM 112112
#define REFSENS 111213
#define BRKSENS 111214

#define SPACE_SEM 1324444
#define MEAS_SEM 1234445

#endif