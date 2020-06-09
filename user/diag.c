#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <sys/io.h>
#include "headers.h"

#include <sys/time.h>

static RT_TASK* asyncTask;		
static SharedMemory* shared;		

static SEM* mutex;			//Sync sem
static SEM* request_sem;		//Check for request

static MBX* mbx_diag;			//Mailbox for tasks synch 
static MBX* mbx_time_amount;		//Mailbox for Ra replay

static RTIME WCET;

static int* simulation_ended;		//Flag to stop the SS activity

void async_request(void){
	printf("\n\n");
	
	RTIME startingTime = rt_get_time();
	RTIME endingTime;
	RTIME currentTime;
	
	pid_t PID = getpid();
	
	unsigned int i;
	unsigned int j;
	
	//Use for share the task's WCET
	Diag_shm receivedLog;
	
	
	
	
		/*		STARTING CRITICAL SECTION		*/
		rt_sem_wait(mutex);
	
		/*
		NOTE:
			Valutazione del WCET avvenuta mediante
			misuarazioni ripetute. (Passaggio facolatativo
			ma utile alla comprensione del docente)	
		*/
	
	
		//2 time the WCET mesured
		shared->WCET = 31435800 * 2;
	
		printf("[ASYNCH_PID %d]: Diag send a request to SS \n", PID);
	
	
		rt_sem_signal(mutex);
		/*		ENDING CRITICAL SECTION		*/
	
	
	
	
		//Signal to the SS
		rt_sem_signal(request_sem);
	

		//Checks for the simulation ending
		if(*simulation_ended) {
			printf("Simulation ended. Please reinsert server module.\n");
			return;
		}

		
		//Waits for the SS reply
		if(!rt_mbx_receive(mbx_diag, (void *)&receivedLog,sizeof(Diag_shm))){
			
			printf("Sporadic server processed an aperiodic request\n");
			for(i = 0; i < NUM_OF_WHEELS; ++i){
				printf("[DIAG 1/2]: AVG %d BLOCK %d CNTR %d\n", receivedLog.filter_avg[i], receivedLog.control_block[i], receivedLog.actuator_cntr[i]);
				printf("BUFFER: ");
				for(j = 0; j < BUF_SIZE; ++j){
					printf("%d ", receivedLog.acquire_buffer[i].buffer[j]);
				}
				printf("\n");
				printf("[DIAG 2/2]: ACQUIRE_WCET %lld FILTER_WCET %lld CONTROL_WCET %lld ACTUATOR_WCET %lld\n", receivedLog.acquire_WCET[i], receivedLog.filter_WCET[i], receivedLog.control_WCET[i], receivedLog.actuator_WCET[i]);			}
				printf("[SS_STATE]: Current Cs %lld NEXT_RT %lld NEXT_RA %lld\n",receivedLog.SS_Cs, receivedLog.SS_nextRt, receivedLog.SS_nextRa);
		}
		else{
			printf("failed retrieving message from mailbox\n");
			
		}
			
		endingTime = rt_get_time();
	
		currentTime = endingTime - startingTime;
	
		if( currentTime > WCET){
			WCET = currentTime;
		}
		
		//Sends for the Ra 
		rt_mbx_send(mbx_time_amount, (void *)&currentTime, sizeof(RTIME));
		printf("WCET %lld execTime %lld\n", WCET, currentTime);
		printf("\n\n");
}


int main(void){

	unsigned int i = 0;

	if(!(asyncTask = rt_task_init_schmod(nam2num("RT_ASYNC"),1,0,0, SCHED_FIFO, 0x1))){
		printf("failed creating rt task\n");
		exit(-1);
	}

	//Init sem
	mutex = rt_typed_named_sem_init(MUTEX_ID,1,BIN_SEM | FIFO_Q);
	request_sem = rt_typed_named_sem_init(REQUEST_SEM_ID, 0, FIFO_Q);
	
	//Init shared
	shared = (SharedMemory*)rtai_malloc(SHARED_ID,sizeof(SharedMemory));
	simulation_ended = (int *) rtai_malloc(SIMENDED_ID, sizeof(int));	

	//Init mailbox
	mbx_diag = rt_typed_named_mbx_init(DIAG_MBX_ID,MAILBOX_SIZE,FIFO_Q);
	mbx_time_amount = rt_typed_named_mbx_init(TIME_AMOUNT_MBX_ID, MAILBOX_SIZE, FIFO_Q);	
	
	for( i = 0; i < 10; ++i){
		async_request();
		usleep(600 * 1000);
	}

	printf("-------------------------SEPARATOR--------------------------\n");

	//Remove sem
	rt_named_sem_delete(mutex);
	rt_named_sem_delete(request_sem);
	
	//Remove shared
	rtai_free(SHARED_ID,&shared);
	rtai_free(SIMENDED_ID, &simulation_ended);	

	//Remove task
	rt_task_delete(asyncTask);
	
	//Remove Mailbox
	rt_named_mbx_delete(mbx_diag);
	rt_named_mbx_delete(mbx_time_amount);

	return 0;
}