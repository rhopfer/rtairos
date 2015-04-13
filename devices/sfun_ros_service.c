/*
 * ROS Service
 *
 * Parameters:
 *   - Name
 *   - Sample Time
 */

#define S_FUNCTION_NAME sfun_ros_service
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <ros_block.h>

#define U(element)          (*uPtrs[element])
#define PARAM(element)      mxGetPr(ssGetSFcnParam(S,element))
#define PARAM_SIZE(element) mxGetNumberOfElements(ssGetSFcnParam(S,element))

static void mdlInitializeSizes(SimStruct *S) {
	ssSetNumSFcnParams(S,2);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		return; /* Parameter mismatch will be reported by Simulink */
	}
	ssSetNumContStates(S,0);
	ssSetNumDiscStates(S,0);

	if (!ssSetNumInputPorts(S,0)) return;
	if (!ssSetNumOutputPorts(S,1)) return;
	ssSetOutputPortWidth(S,0,1);

	ssSetNumSampleTimes(S,1);
	ssSetNumRWork(S,0);
	ssSetNumIWork(S,1);
	ssSetNumPWork(S,2);
	ssSetNumModes(S,0);
	ssSetNumNonsampledZCs(S,0);
	ssSetOptions(S,0);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
	real_T sampleTime = PARAM(1)[0];
	ssSetSampleTime(S,0,sampleTime);
	ssSetOffsetTime(S,0,0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
#ifndef MATLAB_MEX_FILE
	int num;
	rosShmData_t *shm;
	SEM *sem;
	rosBlockInitResult_t res;
	unsigned int strlen = sizeof(char_T)*(PARAM_SIZE(0)+1);
	char_T *str = (char_T *)malloc(strlen);

	mxGetString(ssGetSFcnParam(S,0), str, strlen);

	res = registerRosBlock(S, str, SERVICE, 0);
	shm = res.shm;
	sem = res.sem;
	num = res.num;

	ssSetIWorkValue(S,0,num);
	ssSetPWorkValue(S,0,(void *)shm);
	ssSetPWorkValue(S,1,(void *)sem);

	free(str);
#endif
}

static void mdlOutputs(SimStruct *S,int_T tid) {
	real_T *y = ssGetOutputPortRealSignal(S,0);
#ifndef MATLAB_MEX_FILE
	rosShmData_t *shm = (rosShmData_t *)ssGetPWorkValue(S,0);
	SEM *sem = (SEM *)ssGetPWorkValue(S,1);

	if (rt_sem_wait_if(sem) != 0) {
		if (shm->state == STATE_REQUEST) {
			y[0] = 1;
			shm->state = STATE_RESPONSE;
		} else if (shm->state == STATE_RESPONSE) {
			shm->state = STATE_DONE;
		} else {
			y[0] = 0;
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
