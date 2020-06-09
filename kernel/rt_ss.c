#include <linux/module.h>
#include <asm/io.h>
#include <asm/rtai.h>
#include <rtai_shm.h>
#include <rtai_mbx.h>
#include <rtai_sem.h>
#include <rtai_sched.h>

#include "headers.h"

static RT_TASK ssTask;			//Sporadic Server Task
static ServerInfo ssInfo;		//Struct for SS info

static RTIME startTime,period;		//Variable to make task periodic

static SharedMemory* shared;		//used by aperiodic tasks for dialogue with the server

static SEM* mutex;			//Sync sem
static SEM* request_sem;		//Check for request

static Diag_shm* sharedDiag;		//used to save diagnostic data

static MBX* mbx_diag;			//Mailbox for tasks synch 
static MBX* mbx_time_amount;		//Mailbox for Ra replay
static MBX* mbx_airbag;			//Mailbox for Airbag

static int* breakdown;			//Flag acquire --> SS
static int* simulation_ended;		//Flag SS --> rt_async
static int activate_airbag;		//Flag SS --> airbag

unsigned int ptr;			//Index to Rt-Ra Array

void initInfos(void){
	ssInfo.Cs = 300*PERIOD_NS; 
	ssInfo.Ts = nano2count(PERIOD_NS) * 3000;
}

void aperiodic_fun(RTIME now){
	
	unsigned int i;
	
	/*		START CRITIC SECTION		*/
	rt_sem_wait(sharedDiag->mutex_diag);
	
	for(i = 0; i < NUM_OF_SLOT; ++i){
		//Saving next data
		if(ssInfo.Rt[i] >= now){
			sharedDiag->SS_nextRa = ssInfo.Ra[i];
			sharedDiag->SS_nextRt = ssInfo.Rt[i];
			break;
		}
	}
	//Sharing the current Cs
	sharedDiag->SS_Cs = ssInfo.Cs;
	
	rt_sem_signal(sharedDiag->mutex_diag);
	/*		END CRITIC SECTION		*/
	
	for( i = 0; i < NUM_OF_SLOT; ++i){
	rt_printk("LOG_SERVER: Rt[%d] = %lld Ra[%d] = %lld",i,ssInfo.Rt[i],i, ssInfo.Ra[i]);
	}
	
	//Sending diagnostic data to the DiagTask
	rt_mbx_send(mbx_diag,sharedDiag,sizeof(Diag_shm));
	rt_printk("\n\n\n\n");
	
}

void diag_func(void){

	unsigned int i;
	RTIME Ra_time;
	RTIME now;
	RTIME time2sleep;
	
	now = rt_get_time();			
	
	rt_printk("\n\nVALORE DI NOW: %lld\n", now);	
	
	for(i = 0; i < NUM_OF_SLOT; ++i){
		//Checks if one or more Rt have expired and in case refills the Cs
		if(ssInfo.Rt[i] < now){
			ssInfo.Cs += ssInfo.Ra[i];	
			ssInfo.Ra[i] = 0;
			ssInfo.Rt[i] = 0;
		}
	}
	
	ssInfo.Rt[ptr] = now + ssInfo.Ts;
	
	/*		START CRITIC SECTION		*/
	
	//Sem for the mutually exclusive access to the shared struct
	rt_sem_wait(mutex);
	

	while(ssInfo.Cs < shared->WCET){
		//The SS will sleep until the next Rt
		for(i = 0;i < NUM_OF_SLOT; ++i){
			if(ssInfo.Rt[i] >= now){
				time2sleep = ssInfo.Rt[i];
				rt_sleep_until(time2sleep);
				break;
			}
		}
	}
	
	//Serve the Task
	aperiodic_fun(now);

	//Unlock the mutex
	rt_sem_signal(mutex);
	
	/*		END CRITIC SECTION		*/
	
	
	//The SS waits for the task execution time
	rt_mbx_receive(mbx_time_amount, &Ra_time, sizeof(RTIME));

	ssInfo.Ra[ptr] = Ra_time;
	ssInfo.Cs -= Ra_time;	
	
	ptr = (ptr + 1) % NUM_OF_SLOT;
	
}

int activateAirbag(void){
	if(*breakdown){
		activate_airbag=1;
		rt_mbx_send(mbx_airbag, &activate_airbag, sizeof(int));

		//Flag to stop SS activity
		*simulation_ended = 1;
		return 1;
	}
	else
		return 0;
}

void ss(long t){
	ptr = 0;
	
	while(1){
	
	//Checking if there are pending request
	rt_sem_wait(request_sem);
	
	//If acquire task detects a hard braking the SS activates the airbag task and stops working
	if(activateAirbag() != 0)
		break;
	else
		diag_func();
	}
}

int init_module(void){

	printk("RT kernel module started!\n");
	initInfos();
	
	//Init shared
	shared = (SharedMemory*)rtai_kmalloc(SHARED_ID,sizeof(SharedMemory));
	shared->WCET = 0;
	
	if(!shared){
		printk("Shared memory segment creation failed!\n");
		return -1;
	}
	
	//Init sem
	mutex = rt_typed_named_sem_init(MUTEX_ID,1,BIN_SEM | FIFO_Q);
	request_sem = rt_typed_named_sem_init(REQUEST_SEM_ID, 0,BIN_SEM | FIFO_Q);
	
	//Init mailbox
	mbx_diag = rt_typed_named_mbx_init(DIAG_MBX_ID,MAILBOX_SIZE,FIFO_Q);
	mbx_time_amount = rt_typed_named_mbx_init(TIME_AMOUNT_MBX_ID, MAILBOX_SIZE, FIFO_Q);	
	mbx_airbag = rt_typed_named_mbx_init(AIRBAG_MBX_ID, MAILBOX_SIZE, FIFO_Q);	

	//Init sharedDiag
	sharedDiag = (Diag_shm*)rtai_kmalloc(SHAREDDIAG_ID,sizeof(Diag_shm));

	//Init sharedDiag mutex
	sharedDiag->mutex_diag = rt_typed_named_sem_init(DIAG_SEM_ID,1,BIN_SEM | FIFO_Q);

	//Init breakdown
	breakdown = rtai_kmalloc(BRKDOWNSENS, sizeof(int));
	*breakdown = 0;

	//Init simulation flag
	simulation_ended = rtai_kmalloc(SIMENDED_ID, sizeof(int));
	*simulation_ended = 0;

	period = nano2count(PERIOD_NS);
	
	activate_airbag = 0;
	
	rt_task_init_cpuid(&ssTask,ss,0,STACK_SIZE,1,0,0,0x00);

	startTime = rt_get_time() + (NTASKS+1) *period;

	rt_task_make_periodic(&ssTask,startTime,ssInfo.Ts);
	
	rt_spv_RMS(0x00);
	
	return 0;
}

void cleanup_module(){

	printk("End of RT kernel module\n");
	stop_rt_timer();
	
	//Remove task
	rt_task_delete(&ssTask);

	//Remove Sem
	rt_named_sem_delete(mutex);
	rt_named_sem_delete(request_sem);

	//Remove Mailbox
	rt_named_mbx_delete(mbx_diag);
	rt_named_mbx_delete(mbx_time_amount);
	rt_named_mbx_delete(mbx_airbag);			
	
	//Remove shared
	if(shared){
		rtai_kfree(SHARED_ID);
	}

	//Remove breakdown flag
	rtai_kfree(BRKDOWNSENS);
	
	//Remove sharedDiag
	rtai_kfree(SHAREDDIAG_ID);
	
	//Remove sharedDiag mutex
	rt_sem_delete(sharedDiag->mutex_diag);
	
}



