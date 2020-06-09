//------------------- CONTROLLER.C ---------------------- 

#include "parameters.h"

#define CPUMAP 0x1
#define MAILBOX_ID "SynchMBX"
#define MAILBOX_SIZE 256

static RT_TASK *main_Task;
static RT_TASK *read_Task;
static RT_TASK *filter_Task[NUM_OF_WHEELS];
static RT_TASK *control_Task[NUM_OF_WHEELS];
static RT_TASK *write_Task[NUM_OF_WHEELS];
static int keep_on_running = 1;

static pthread_t read_thread;
static pthread_t filter_thread;
static pthread_t control_thread;
static pthread_t write_thread;
static RTIME sampl_interv;

static MBX *mailBox;					//Mailbox
static SEM* request_sem;				//Check for airbag request

static Diag_shm* sharedDiag;				//Used for shared current values

static void endme(int dummy) {keep_on_running = 0;}

int* sensor;
int* actuator;
int* reference;
int* handbrake;						//Flag handbrake --> controllers
int* breakdown;						//Flag acquire --> SS
int* airbag_stop;					//Flag airbag --> controllers

buf_struct bufferArray[NUM_OF_WHEELS];


static void * acquire_loop(void * par) {
	
	//The index passed
	int index = (int) par;
	
	//For WCET calculation
	RTIME WCET;
	WCET = 0;
	
	int prev_sensor = 0;	

	//Inizializzo coda e testa a zero.
	bufferArray[index].head = bufferArray[index].tail = 0;
	

	if (!(read_Task = rt_task_init_schmod(nam2num("READER")+index, 1, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SENSOR TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(read_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	while (keep_on_running)
	{		
	
		/*	STARTING CURRENT TIME CALCULATION	*/
		RTIME Timer[3];
		RTIME startingTime;
		RTIME endingTime;
		RTIME currentTime;
		
		rt_get_exectime(read_Task, Timer);
		
		startingTime = Timer[0];
		/*	ENDING CURRENT TIME CALCULATION	*/
		
		
		
		// DATA ACQUISITION FROM PLANT
		rt_sem_wait(bufferArray[index].space_avail);
		
		bufferArray[index].buffer[bufferArray[index].head] = (sensor[index]);
		
		//If hard braking is detected acquire notifies the SS
		if(prev_sensor-sensor[index]>=50){			
			*breakdown=1;

			rt_sem_signal(request_sem);
		}
		prev_sensor=sensor[index];

		
		bufferArray[index].head = (bufferArray[index].head + 1) % BUF_SIZE;
		
		rt_printk("[ACQUIRE %d]: BUFFER[%d] = %d\n", index, index, sensor[index]);
		
		rt_sem_signal(bufferArray[index].meas_avail);
		
		rt_task_wait_period();
		
		rt_get_exectime(read_Task, Timer);
		
		endingTime = Timer[0];
		
		currentTime = endingTime - startingTime;
		
		if(currentTime > WCET)
			WCET = currentTime;

		//Saving content on sharedDiag
		rt_sem_wait(sharedDiag->mutex_diag);

		sharedDiag->acquire_buffer[index] = bufferArray[index];
		sharedDiag->acquire_WCET[index] = WCET;

		rt_sem_signal(sharedDiag->mutex_diag);
		
	}
	
	rt_task_delete(read_Task);
	return 0;
}

static void * filter_loop(void * par) {

	int index = (int) par;
	unsigned i = 0;
	
	RTIME WCET;
	WCET = 0;

	if (!(filter_Task[index] = rt_task_init_schmod(nam2num("FILTER")+index, 3, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT FILTER TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(filter_Task[index], expected, sampl_interv);
	rt_make_hard_real_time();

	int cnt = BUF_SIZE;
	unsigned int sum = 0;
	unsigned int avg = 0;
	while (keep_on_running)
	{
	
		/*	STARTING CURRENT TIME CALCULATION	*/
		RTIME Timer[3];
		RTIME startingTime;
		RTIME endingTime;
		RTIME currentTime;
		
		rt_get_exectime(filter_Task[index], Timer);
		
		startingTime = Timer[0];
		/*	ENDING CURRENT TIME CALCULATION		*/
	
	
	
		// FILTERING (average)
		rt_printk("[FILTER %d]: waiting for mutex",index);
		rt_sem_wait(bufferArray[index].meas_avail);

		sum += bufferArray[index].buffer[bufferArray[index].tail];
		bufferArray[index].tail = (bufferArray[index].tail + 1) % BUF_SIZE;
		
		rt_sem_signal(bufferArray[index].space_avail);
		
		cnt--;

		if (cnt == 0) {
			cnt = BUF_SIZE;
			avg = sum/BUF_SIZE;
			sum = 0;
			rt_printk("[FILTER %d]: signal mutex (avg = %d)",index, avg);

			
			// sends the average measure to the controller
			rt_send(control_Task[index], avg);		
		}
		rt_task_wait_period();
		
		rt_get_exectime(filter_Task[index], Timer);
		
		endingTime = Timer[0];
		
		currentTime = endingTime - startingTime;
		
		if(currentTime > WCET)
			WCET = currentTime;

		//Saving data to the shared structure.
		rt_sem_wait(sharedDiag->mutex_diag);
		
		sharedDiag->filter_avg[index] = avg;
		sharedDiag->filter_WCET[index] = WCET;
		
		
		rt_sem_signal(sharedDiag->mutex_diag);
	}
	
	for(i = 0; i < NUM_OF_WHEELS; ++i){
		rt_task_delete(filter_Task[i]);
	}
	
	return 0;
}

static void * control_loop(void * par) {
	unsigned int prev_sensor=0;             	//to store previous sensor readings to detect skids
	unsigned int plant_state = 0;           	//speed received from the plant
	int error = 0;                          	//error to use to calculate the control action
	unsigned int control_action = 0;        	//control action to be sent to the actuator
	unsigned int ANTI_SKID_ON = 1;			//to activate the ANTI SKID
	unsigned int CONTROL_PERIOD_MULTIPLIER = 1;	//to configure the control period
	
	
	int block = 0; 					// to check if the wheel is blocked

	int index = (int) par;
	unsigned int i = 0;
	
	RTIME WCET;
	WCET = 0;

	if (!(control_Task[index] = rt_task_init_schmod(nam2num("CNTRL")+index, 5, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT CONTROL TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(control_Task[index], expected, 
		CONTROL_PERIOD_MULTIPLIER*BUF_SIZE*sampl_interv);
	rt_make_hard_real_time();
      
	while (keep_on_running)
	{         
		/*	STARTING CURRENT TIME CALCULATION	*/
		RTIME Timer[3];
		RTIME startingTime;
		RTIME endingTime;
		RTIME currentTime;
		
		rt_get_exectime(control_Task[index], Timer);
		
		startingTime = Timer[0];
		/*	ENDING CURRENT TIME CALCULATION		*/
	
	
	
		// receiving the average plant state from the filter
		rt_receive(filter_Task[index], &plant_state);
		rt_printk("[CONTROL %d] my plant state %d", index, plant_state);
		// evaluating if the wheel is blocked
                if(prev_sensor==(sensor[index])){
                	block = 1;
                	rt_printk("[CONTROL %d]: Black control %d", index, (index + 1) % NUM_OF_WHEELS);
                	rt_mbx_send(mailBox, (void *)&block, sizeof(block));
                }
                else {
        	        block = 0; 
                //get blocked state.
	                rt_mbx_receive_if(mailBox, (void *)&block, sizeof(block));
                
                	rt_printk("[CONTROL %d] il mio valore di block è %d", index, block);
                }
                

                
		// computation of the control law
		error = (*reference) - plant_state;
		if (error > 0) control_action = 1;
		else if (error < 0) control_action = 2;
                else control_action = 3;
		
		if (ANTI_SKID_ON) {
			if (((*reference)==0) && (plant_state!=0) && (block!=1)) 
				control_action = 4; //brake only when no skid is detected.
		} else if ((*reference) == 0) control_action = 4;
 
                prev_sensor=(sensor[index]);
		
		//Checks for the signal from the handbrake task
		if(handbrake[index]){                          
			control_action=5;
			handbrake[index]=0;                  
		}

		//Check for the signal from the airbag task
		if(*airbag_stop){
			*(reference) = 0;			
		}
                  
		// sending the control action to the actuator
		rt_send_if(write_Task[index], control_action);
		
			
             	rt_task_wait_period();
             	
		rt_get_exectime(control_Task[index], Timer);
		
		endingTime = Timer[0];
		
		currentTime = endingTime - startingTime;
		
		if(currentTime > WCET)
			WCET = currentTime;

		//Salving the current WCET on the structure
		
		rt_sem_wait(sharedDiag->mutex_diag);
		
		sharedDiag->control_block[index] = block;
		sharedDiag->control_WCET[index] = WCET;
		
		
		rt_sem_signal(sharedDiag->mutex_diag);
		
	}
	rt_task_delete(control_Task[index]);
	return 0;
}



static void * actuator_loop(void * par) {

	int index = (int) par;
	
	RTIME WCET;
	WCET = 0;

	if (!(write_Task[index] = rt_task_init_schmod(nam2num("WRITE")+index, 7, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT ACTUATOR TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(write_Task[index], expected, BUF_SIZE*sampl_interv);
	rt_make_hard_real_time();

	unsigned int control_action = 0;
	int cntr = 0;

	while (keep_on_running)

	{		
		/*	STARTING CURRENT TIME CALCULATION	*/
		RTIME Timer[3];
		RTIME startingTime;
		RTIME endingTime;
		RTIME currentTime;
		
		rt_get_exectime(write_Task[index], Timer);
		
		startingTime = Timer[0];
		/*	ENDING CURRENT TIME CALCULATION		*/
		
		// receiving the control action from the controller
		rt_receive(write_Task[index], &control_action);
		
		switch (control_action) {
			case 1: cntr =  1; break;
			case 2:	cntr = -1; break;
			case 3:	cntr =  0; break;
            		case 4: cntr = -2; break;
			case 5: cntr = -3; break;
			default: cntr = 0;
		}
		
		(actuator[index]) = cntr;
		

		rt_printk("[ACTUATOR %d] Control Action:%d", index, actuator[index]);
		
		rt_task_wait_period();
		
		
		rt_get_exectime(write_Task[index], Timer);
		
		endingTime = Timer[0];
		
		currentTime = endingTime - startingTime;
		
		if(currentTime > WCET)
			WCET = currentTime;

		//Salving the current WCET on the structure
		rt_sem_wait(sharedDiag->mutex_diag);
		
		sharedDiag->actuator_cntr[index] = cntr;
		sharedDiag->actuator_WCET[index] = WCET;
		
		rt_sem_signal(sharedDiag->mutex_diag);
		
	}
	rt_task_delete(write_Task[0]);
	rt_task_delete(write_Task[1]);
	return 0;
}

int main(void)
{
	unsigned int i = 0;


	printf("The controller is STARTED!\n");
 	signal(SIGINT, endme);

	if (!(main_Task = rt_task_init_schmod(nam2num("MAINTSK"), 0, 0, 0, SCHED_FIFO, 0xF))) {
		printf("CANNOT INIT MAIN TASK\n");
		exit(1);
	}

	//attach to data shared with the controller
	sensor = rtai_malloc(SEN_SHM, sizeof(int));
	actuator = rtai_malloc(ACT_SHM, sizeof(int));
	reference = rtai_malloc(REFSENS, sizeof(int));
	handbrake = rtai_malloc(BRKSENS, sizeof(int));
	breakdown = rtai_kmalloc(BRKDOWNSENS, sizeof(int));			
	airbag_stop = rtai_malloc(AIRSENS, sizeof(int));
	

	(*reference) = 110;
	*breakdown = 0;

	for(i = 0; i < NUM_OF_WHEELS; ++i){
		bufferArray[i].space_avail = rt_typed_sem_init(SPACE_SEM + i, BUF_SIZE, CNT_SEM | PRIO_Q);
		bufferArray[i].meas_avail = rt_typed_sem_init(MEAS_SEM + i, 0, CNT_SEM | PRIO_Q);
		sampl_interv = nano2count(CNTRL_TIME);
	}
	
	
	//Init la mailBox
	mailBox = rt_typed_named_mbx_init(MAILBOX_ID, MAILBOX_SIZE, FIFO_Q);
	request_sem = rt_typed_named_sem_init(REQUEST_SEM_ID, 0, FIFO_Q);	
	
	//Init sharedDiag
	sharedDiag = (Diag_shm*)rtai_kmalloc(SHAREDDIAG_ID,sizeof(Diag_shm));
	
	//Init SharedDiag mutex
	sharedDiag->mutex_diag = rt_typed_named_sem_init(DIAG_SEM_ID,1,BIN_SEM | FIFO_Q);
	
	// CONTROL THREADS 
	for(i = 0; i < NUM_OF_WHEELS; ++i){
		pthread_create(&read_thread, NULL, acquire_loop, (void *)i);
		pthread_create(&filter_thread, NULL, filter_loop, (void *)i);
		pthread_create(&control_thread, NULL, control_loop, (void *)i);
		pthread_create(&write_thread, NULL, actuator_loop, (void *)i);
	}

	while (keep_on_running) {
		for(i = 0; i < NUM_OF_WHEELS; ++i){
			printf("[Control %d]: %d",i,(actuator[i]));
			printf("	");
			rt_sleep(10000000);
		}
		printf("\n");
	}

	//Remove data shared
	rt_shm_free(SEN_SHM);
	rt_shm_free(ACT_SHM);
	rt_shm_free(REFSENS);
	rt_shm_free(AIRSENS);

	//Remove sem
	for(i = 0; i < NUM_OF_WHEELS; ++i){
		rt_sem_delete(bufferArray[i].meas_avail);
		rt_sem_delete(bufferArray[i].space_avail);
	}
	
	//Remove sharedDiag mutex
	rt_sem_delete(sharedDiag->mutex_diag);
	
	rt_task_delete(main_Task);
 	printf("The controller is STOPPED\n");
 	
	return 0;
}




