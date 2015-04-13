#ifndef _ROS_BLOCK_H
#define _ROS_BLOCK_H

#include "simstruc.h"
#include <ros_defines.h>

#ifndef MATLAB_MEX_FILE
#include <rtai_shm.h>
#include <rtai_sem.h>

extern rosBlockConfig_t rosBlockConfigs[];
extern char *RosShmID;
extern char *RosSemID;
extern unsigned int numRosBlocks;


typedef struct rosBlockInitResult_t {
    unsigned int num;
    rosShmData_t *shm;
    SEM *sem;
} rosBlockInitResult_t;

rosBlockInitResult_t registerRosBlock(SimStruct *S, char *rosName, int type, int subType);

void cleanRosBlock(unsigned int num);

#endif

char_T* ssGetBlockName(SimStruct *S);

#endif
