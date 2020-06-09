#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>					//librerie da modificare
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <sys/io.h>
#include "parameters.h"


static MBX* mbx_airbag;			//Mailbox for Sporadic Server
static RT_TASK* airbagTask;		

int* airbag_stop;			//Flag airbag --> controllers


void airbag_fun(void){

	int activate_airbag = 0;
	printf("Airbag Detection...\n");

	//AirbagTask waits for the server activation
	rt_mbx_receive(mbx_airbag, &activate_airbag,sizeof(int));

	printf("Airbag Attivato\n");

	//Activates controllers braking
	*airbag_stop=1;

}
	

int main(void){

	//Init mailbox
	mbx_airbag = rt_typed_named_mbx_init(AIRBAG_MBX_ID, MAILBOX_SIZE, FIFO_Q);		
	
	//Init flag
	airbag_stop = rtai_malloc(AIRSENS, sizeof(int));

	//Init Task
	if(!(airbagTask = rt_task_init_schmod(nam2num("RT_AIRBAG"),1,0,0, SCHED_FIFO, 0x1))){
		printf("failed creating rt task\n");
		exit(-1);
	}

	airbag_fun();
	
	//Remove Task
	rt_task_delete(airbagTask);
	
	//Remove mailbox
	rt_named_mbx_delete(mbx_airbag);
	
	//Remove flag
	rtai_free(AIRSENS, &airbag_stop);
	
	return 0;
}