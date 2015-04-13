/*
 * ROS Joy Subscriber
 *
 * Parameters:
 *   - Number of axes
 *   - Number of buttons
 *   - Topic
 *   - Sample time
 */

#define S_FUNCTION_NAME sfun_ros_joy
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include <ros_block.h>

#define U(element)          (*uPtrs[element])
#define PARAM(element)      mxGetPr(ssGetSFcnParam(S,element))
#define PARAM_SIZE(element) mxGetNumberOfElements(ssGetSFcnParam(S,element))

static void mdlInitializeSizes(SimStruct *S) {
	int_T axes, buttons, numOut, buttonPort;

	ssSetNumSFcnParams(S,4);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		return; /* Parameter mismatch will be reported by Simulink */
	}
	axes = PARAM(0)[0];
	buttons = PARAM(1)[0];
	if (axes < 0 || buttons < 0) {
		return;
	}

	numOut = (axes > 0) + (buttons > 0);
	buttonPort = (axes > 0);

	ssSetNumContStates(S,0);
	ssSetNumDiscStates(S,0);

	if (!ssSetNumInputPorts(S,0)) return;

	if (!ssSetNumOutputPorts(S,numOut)) return;
	if (axes > 0) {
		ssSetOutputPortWidth(S,0,axes);
	}
	if (buttons > 0) {
		ssSetOutputPortWidth(S,buttonPort,buttons);
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
	int_T axes = PARAM(0)[0];
	int_T buttons = PARAM(1)[0];
	real_T *y;
	int i, j;
#ifndef MATLAB_MEX_FILE
	int num;
	rosShmData_t *shm;
	SEM *sem;
	rosBlockInitResult_t res;
	unsigned int strlen = sizeof(char_T)*(PARAM_SIZE(2)+1);
	char_T *str = (char_T *)malloc(strlen);

	mxGetString(ssGetSFcnParam(S,2), str, strlen);

	res = registerRosBlock(S, str, SUBSCRIBER, SUBSCRIBER_JOY);
	shm = res.shm;
	sem = res.sem;
	num = res.num;

	rt_sem_wait(sem);
	shm->length = axes + buttons;
	for (i = 0; i < shm->length; ++i) {
		shm->data[i] = 0.0;
	}
	rt_sem_signal(sem);

	ssSetIWorkValue(S,0,num);
	ssSetPWorkValue(S,0,(void *)shm);
	ssSetPWorkValue(S,1,(void *)sem);
#endif
	for (i = 0; i < ssGetNumOutputPorts(S); ++i) {
		for (j = 0; j < ssGetOutputPortWidth(S,i); ++j) {
			y = ssGetOutputPortRealSignal(S,i);
			y[j] = 0.0;
		}
	}
}

static void mdlOutputs(SimStruct *S,int_T tid) {
	real_T *y;
	int_T axes = PARAM(0)[0];
#ifndef MATLAB_MEX_FILE
	int i;
	rosShmData_t *shm = (rosShmData_t *)ssGetPWorkValue(S,0);
	SEM *sem = (SEM *)ssGetPWorkValue(S,1);
	int sem_ret;
	int buttonsOut;

	buttonsOut = (axes > 0);
	sem_ret = rt_sem_wait_if(sem);
	if (sem_ret != 0) {
		for (i = 0; i < shm->length; ++i) {
			if (i < axes) {
				y = ssGetOutputPortRealSignal(S,0);
				y[i] = shm->data[i];
			} else {
				y = ssGetOutputPortRealSignal(S,buttonsOut);
				y[i-axes] = shm->data[i];
			}
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
