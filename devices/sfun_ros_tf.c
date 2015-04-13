/*
 * ROS Transformation Broadcaster
 *
 * Parameters: 
 *   - Type
 *   - Name, if not set the block name is used
 *   - Reference name
 *   - Sample time
 */

#define S_FUNCTION_NAME sfun_ros_tf
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <ros_block.h>

#define INPUT_TYPE_2D 1
#define INPUT_TYPE_3D 2
#define DATA_SIZE 6

#define U(element)          (*uPtrs[element])
#define PARAM(element)      mxGetPr(ssGetSFcnParam(S,element))
#define PARAM_SIZE(element) mxGetNumberOfElements(ssGetSFcnParam(S,element))

static void mdlInitializeSizes(SimStruct *S) {
	int_T type = PARAM(0)[0];

	ssSetNumSFcnParams(S,4);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		return; /* Parameter mismatch will be reported by Simulink */
	}
	if (!ssSetNumInputPorts(S,1)) return;
	switch (type) {
		case INPUT_TYPE_2D:
			ssSetInputPortWidth(S,0,3);
			break;
		case INPUT_TYPE_3D:
			ssSetInputPortWidth(S,0,6);
			break;
		default:
			ssSetErrorStatus(S, "Invalid type");
	}

	ssSetNumContStates(S,0);
	ssSetNumDiscStates(S,0);

	ssSetInputPortDirectFeedThrough(S,0,1);

	if (!ssSetNumOutputPorts(S,0)) return;

	ssSetNumSampleTimes(S,1);
	ssSetNumRWork(S,0);
	ssSetNumIWork(S,1);
	ssSetNumPWork(S,2);
	ssSetNumModes(S,0);
	ssSetNumNonsampledZCs(S,0);
	ssSetOptions(S,0);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
	real_T sampleTime = PARAM(3)[0];
	ssSetSampleTime(S,0,sampleTime);
	ssSetOffsetTime(S,0,0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
#ifndef MATLAB_MEX_FILE
	int_T i;
	int num;
	rosShmData_t *shm;
	SEM *sem;
	rosBlockInitResult_t res;
	unsigned int strlen = sizeof(char_T)*(PARAM_SIZE(1)+1);
	char_T *str = (char_T *)malloc(strlen);

	mxGetString(ssGetSFcnParam(S,1), str, strlen);

	res = registerRosBlock(S, str, BROADCASTER, 0);
	shm = res.shm;
	sem = res.sem;
	num = res.num;

	strlen = sizeof(char_T)*(PARAM_SIZE(2)+1);
	str = (char_T *)realloc(str, strlen);
	mxGetString(ssGetSFcnParam(S,2), str, strlen);
	memcpy(rosBlockConfigs[num].refName, str, MAX_NAMES_SIZE);

	rt_sem_wait(sem);
	shm->length = DATA_SIZE;
	for (i = 0; i < shm->length; ++i) {
		shm->data[i] = 0.0;
	}
	rt_sem_signal(sem);

	ssSetIWorkValue(S,0,num);
	ssSetPWorkValue(S,0,(void *)shm);
	ssSetPWorkValue(S,1,(void *)sem);

	free(str);
#endif
}

static void mdlOutputs(SimStruct *S,int_T tid) {
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);

#ifndef MATLAB_MEX_FILE
	int_T type = PARAM(0)[0];
	int i;
	rosShmData_t *shm = (rosShmData_t *)ssGetPWorkValue(S,0);
	SEM *sem = (SEM *)ssGetPWorkValue(S,1);
	int sem_ret;

	sem_ret = rt_sem_wait_if(sem);
	if (sem_ret != 0) {
		if (type == INPUT_TYPE_3D) {
			for (i = 0; i < shm->length; ++i) {
				shm->data[i] = U(i);
			}
		} else if (type == INPUT_TYPE_2D) {
			shm->data[0] = U(0);
			shm->data[1] = U(1);
			shm->data[5] = U(2);
		}
		rt_sem_signal(sem);
	}
#endif

	UNUSED_ARG(tid);	 /* not used in single tasking mode */
}

static void mdlTerminate(SimStruct *S) {
#ifndef MATLAB_MEX_FILE
	int num = ssGetIWorkValue(S,0);
	cleanRosBlock(num);
#endif
}

#ifdef MATLAB_MEX_FILE   /* Compile as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"   /* Code generation registration */
#endif
