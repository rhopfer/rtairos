/*
 * ROS Subscriber
 *
 * Parameters:
 *   - Message type (see ros_defines.h)
 *   - Topic, if not set the block name is used
 *   - Initial values
 *   - Sample time
 *   - Reset value on enable inside enabled subsystem
 */

#define S_FUNCTION_NAME sfun_ros_subscriber
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <ros_block.h>

#define U(element)          (*uPtrs[element])
#define PARAM(element)      mxGetPr(ssGetSFcnParam(S,element))
#define PARAM_SIZE(element) mxGetNumberOfElements(ssGetSFcnParam(S,element))

static void mdlInitializeSizes(SimStruct *S) {
	int_T msgType;
	int_T initValLen;

	ssSetNumSFcnParams(S,5);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		return; /* Parameter mismatch will be reported by Simulink */
	}
	ssSetNumContStates(S,0);
	ssSetNumDiscStates(S,0);

	if (!ssSetNumInputPorts(S,0)) return;

	msgType = (int_T)*mxGetPr(ssGetSFcnParam(S,0));
	switch (msgType) {
		case SUBSCRIBER_TWIST_STAMPED:
		case PUBLISHER_POINT_STAMPED:
			if (!ssSetNumOutputPorts(S,2)) return;
			ssSetOutputPortWidth(S,1,2);
			break;
		default:
			if (!ssSetNumOutputPorts(S,1)) return;
	}

	switch (msgType) {
		case SUBSCRIBER_FLOAT64:
		case SUBSCRIBER_INT32:
		case SUBSCRIBER_BOOL:
		case SUBSCRIBER_TIME:
			ssSetOutputPortWidth(S,0,1);
			break;
		case SUBSCRIBER_FLOAT64ARRAY:
			ssSetOutputPortWidth(S,0,DYNAMICALLY_SIZED);
			break;
		case SUBSCRIBER_TWIST:
		case SUBSCRIBER_TWIST_STAMPED:
			ssSetOutputPortWidth(S,0,6);
			break;
		case PUBLISHER_POINT:
		case PUBLISHER_POINT_STAMPED:
		case PUBLISHER_POSE2D:
			ssSetOutputPortWidth(S,0,3);
			break;
		default:
			ssSetErrorStatus(S, "Unknown message type!");
			return;
	}
	initValLen = mxGetNumberOfElements(ssGetSFcnParam(S,2));
	if (initValLen > 1) {
		ssSetOutputPortWidth(S,0,initValLen);
	}

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
	real_T *y = ssGetOutputPortRealSignal(S,0);
	const real_T *initVal = PARAM(2);
	int_T initValLen = PARAM_SIZE(2);
	int i;
#ifndef MATLAB_MEX_FILE
	int num;
	rosShmData_t *shm;
	SEM *sem;
	rosBlockInitResult_t res;
	unsigned int strlen = sizeof(char_T)*(PARAM_SIZE(1)+1);
	char_T *str = (char_T *)malloc(strlen);

	mxGetString(ssGetSFcnParam(S,1), str, strlen);

	res = registerRosBlock(S, str, SUBSCRIBER, PARAM(0)[0]);
	shm = res.shm;
	sem = res.sem;
	num = res.num;

	rt_sem_wait(sem);
	shm->length = ssGetOutputPortWidth(S,0);
	for (i = 0; i < shm->length; ++i) {
		if (initValLen > 1) {
			shm->data[i] = initVal[i];
		} else {
			shm->data[i] = initVal[0];
		}
	}
	rt_sem_signal(sem);

	ssSetIWorkValue(S,0,num);
	ssSetPWorkValue(S,0,(void *)shm);
	ssSetPWorkValue(S,1,(void *)sem);

	free(str);
#endif
	for (i = 0; i < ssGetOutputPortWidth(S,0); ++i) {
		if (initValLen > 1) {
			y[i] = initVal[i];
		} else {
			y[i] = initVal[0];
		}
	}
}

#define MDL_ENABLE
static void mdlEnable(SimStruct *S) {
#ifndef MATLAB_MEX_FILE
	const real_T *initVal = PARAM(2);
	int_T initValLen = PARAM_SIZE(2);
	bool reset = PARAM(4)[0];
	int i;
	rosShmData_t *shm = (rosShmData_t *)ssGetPWorkValue(S,0);
	SEM *sem = (SEM *)ssGetPWorkValue(S,1);

	if (!reset) return;

	if (rt_sem_wait_if(sem) != 0) {
		for (i = 0; i < shm->length; ++i) {
			if (initValLen > 1) {
				shm->data[i] = initVal[i];
			} else {
				shm->data[i] = initVal[0];
			}
		}
		rt_sem_signal(sem);
	}
#endif
}

static void mdlOutputs(SimStruct *S,int_T tid) {
	real_T *y;
#ifndef MATLAB_MEX_FILE
	int i;
	rosShmData_t *shm = (rosShmData_t *)ssGetPWorkValue(S,0);
	SEM *sem = (SEM *)ssGetPWorkValue(S,1);

	if (rt_sem_wait_if(sem) != 0) {
		y = ssGetOutputPortRealSignal(S,0);
		for (i = 0; i < shm->length; ++i) {
			y[i] = shm->data[i];
		}
		if (ssGetNumOutputPorts(S) > 1) {
			y = ssGetOutputPortRealSignal(S,1);
			y[0] = shm->header.time;
			y[1] = shm->header.seq;
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
