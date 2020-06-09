#ifndef HEADERS_H
#define HEADERS_H

//lib2share.h must be in the main directory
#include "../lib2share.h"

#define PERIOD_NS 1000000
#define STACK_SIZE 2048

#define NTASKS 15

#define MAX_REQUEST_NUM 5


//struct for serverInfo
typedef struct{

	RTIME Cs;			//Server Capacity
	RTIME Ts;			//Server Period
	
	RTIME Rt[MAX_REQUEST_NUM];	//Replenishment Time
	RTIME Ra[MAX_REQUEST_NUM];	//Replenishment Amount
	
	
}ServerInfo;

#endif
