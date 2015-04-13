/*
 * ROS Publisher
 *
 * Parameters:
 *   - Message type (see ros_defines.h)
 *   - Topic, if not set the block name is used
 *   - Sample time
 */

#define S_FUNCTION_NAME sfun_ros_publisher
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <ros_block.h>

#define U(element)          (*uPtrs[element])
#define PARAM(element)      mxGetPr(ssGetSFcnParam(S,element))
#define PARAM_SIZE(element) mxGetNumberOfElements(ssGetSFcnParam(S,element))

static void mdlInitializeSizes(SimStruct *S) {
	int_T msgType;

	ssSetNumSFcnParams(S,3);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		return; /* Parameter mismatch will be reported by Simulink */
	}
	msgType = (int_T)PARAM(0)[0];
	if (!ssSetNumInputPorts(S,1)) return;
	switch (msgType) {
		case PUBLISHER_FLOAT64:
		case PUBLISHER_TIME:
		case PUBLISHER_INT32:
		case PUBLISHER_BOOL:
			ssSetInputPortWidth(S,0,1);
			break;
		case PUBLISHER_FLOAT64ARRAY:
			ssSetInputPortWidth(S,0,DYNAMICALLY_SIZED);
			break;
		case PUBLISHER_TWIST:
		case PUBLISHER_TWIST_STAMPED:
			ssSetInputPortWidth(S,0,6);
			break;
		case PUBLISHER_POINT:
		case PUBLISHER_POINT_STAMPED:
		case PUBLISHER_POSE2D:
			ssSetInputPortWidth(S,0,3);
			break;
		default:
			ssSetErrorStatus(S, "Unknown message type!");
			return;
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
	real_T sampleTime = PARAM(2)[0];
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

	res = registerRosBlock(S, str, PUBLISHER, PARAM(0)[0]);
	shm = res.shm;
	sem = res.sem;
	num = res.num;

	rt_sem_wait(sem);
	shm->length = ssGetInputPortWidth(S,0);
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
	int i;
	rosShmData_t *shm = (rosShmData_t *)ssGetPWorkValue(S,0);
	SEM *sem = (SEM *)ssGetPWorkValue(S,1);

	if (rt_sem_wait_if(sem) != 0) {
		for (i = 0; i < shm->length; ++i) {
			shm->data[i] = U(i);
		}
		shm->state = NEW_VALUE;
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
