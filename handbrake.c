//----------------------HANDBRAKE.C ----------------------------

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <rtai_shm.h>
#include "parameters.h"

 
//DECREASES SPEED BY ENABLING HANDBRAKE
int main(void)

{
	int handbrake = 0;

	int *brk_sensor;
	
	//Init brk_sensor
	brk_sensor = rtai_malloc (BRKSENS,1);

	do{
		do{
		printf("Pull the handbrake?\n");
		printf("0.exit\n");
		printf("1.pull handbrake\n");
		printf(">");
		scanf("%d",&handbrake);
		}while((handbrake!=0) && (handbrake!=1));

		if(handbrake){
			for(int i=0;i<NUM_OF_WHEELS;++i)
				brk_sensor[i]=1;
		}else
			handbrake = 0; //if you enter something which is not 1

	}while(handbrake);

	//Remove brk_sensor
	rtai_free(BRKSENS, &brk_sensor);

    return 0;

}
