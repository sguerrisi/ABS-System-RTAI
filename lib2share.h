#ifndef _LIB2SHARE_H_
#define _LIB2SHARE_H_





/*		NUM		*/
#define NUM_OF_WHEELS 2
#define NUM_OF_SLOT 5

/*		SIZE		*/
#define BUF_SIZE 10
#define MAILBOX_SIZE 256

/*		ID		*/
#define DIAG_SEM_ID "DiagSem"
#define SHARED_ID 12345
#define BRKDOWNSENS 111215
#define AIRSENS 111216
#define SIMENDED_ID 111217
#define MUTEX_ID "myMutex"
#define REQUEST_SEM_ID "Semmy"
#define AIRBAG_MUTEX_ID "myAirbagMutex"
#define SHAREDDIAG_ID 4041999
#define DIAG_MBX_ID "MyDiagMailbox"
#define TIME_AMOUNT_MBX_ID "MyTimeAmountMailbox"
#define AIRBAG_MBX_ID "MyAirbagMailbox"





//used by aperiodic tasks for dialogue with the server
typedef struct{

	int WCET;					//each task sends its own WCET
	
	//space left for future implementations
	//...	
	
}SharedMemory;


//used to adapt the system to the number of wheels
typedef struct{

	int buffer[BUF_SIZE];				//data buffer between acquire and filter 
	int head,tail;					//used for the management of the queue
	
	SEM* space_avail;				//Prod-Cons sem
	SEM* meas_avail;				//Prod-Cons sem
	
}buf_struct;


//used for shared diagnostics
typedef struct{

	SEM* mutex_diag;				//Sem for mutually exclusive access								

	buf_struct acquire_buffer[NUM_OF_WHEELS];	//Current buffer value
	RTIME acquire_WCET[NUM_OF_WHEELS];		//WCET acquireTask
	
	int filter_avg[NUM_OF_WHEELS];			//Current average value
	RTIME filter_WCET[NUM_OF_WHEELS];		//WCET filterTask
			
	int control_block[NUM_OF_WHEELS];		//Current blocking value
	RTIME control_WCET[NUM_OF_WHEELS];		//WCET controlTask
	
	int actuator_cntr[NUM_OF_WHEELS];		//Current value of the actuation signal
	RTIME actuator_WCET[NUM_OF_WHEELS];		//WCET actuatorTask	
	
	RTIME SS_Cs;					//Current Cs value of the SS 
	RTIME SS_nextRa;				//Next Ra value of the SS
	RTIME SS_nextRt;				//Next Rt value of the SS

}Diag_shm;

#endif