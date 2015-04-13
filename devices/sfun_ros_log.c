/*
 */

#define S_FUNCTION_NAME sfun_ros_log
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#ifndef MATLAB_MEX_FILE
#include <ros_block.h>
#endif

#define U0(element)         (*uPtrs0[element])
#define U1(element)         (*uPtrs1[element])
#define PARAM(element)      mxGetPr(ssGetSFcnParam(S,element))
#define PARAM_SIZE(element) mxGetNumberOfElements(ssGetSFcnParam(S,element))

static void mdlInitializeSizes(SimStruct *S) {
	bool dataPort;

	ssSetNumSFcnParams(S,3);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		return; /* Parameter mismatch will be reported by Simulink */
	}
	dataPort = PARAM(2)[0];

	if (dataPort) {
		if (!ssSetNumInputPorts(S,2)) return;
		ssSetInputPortWidth(S,1,DYNAMICALLY_SIZED);
		ssSetInputPortDirectFeedThrough(S,1,1);
	} else {
		if (!ssSetNumInputPorts(S,1)) return;
	}
	ssSetInputPortWidth(S,0,1);
	ssSetInputPortDirectFeedThrough(S,0,1);

	ssSetNumContStates(S,0);
	ssSetNumDiscStates(S,0);

	if (!ssSetNumOutputPorts(S,0)) return;

	ssSetInputPortDataType(S, 0, DYNAMICALLY_TYPED);
	ssSetNumSampleTimes(S,1);
	ssSetNumRWork(S,1);
	ssSetNumIWork(S,1);
	ssSetNumPWork(S,2);
	ssSetNumModes(S,0);
	ssSetNumNonsampledZCs(S,0);
	ssSetOptions(S,0);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
	ssSetSampleTime(S,0,INHERITED_SAMPLE_TIME);
	ssSetOffsetTime(S,0,FIXED_IN_MINOR_STEP_OFFSET);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
#ifndef MATLAB_MEX_FILE
	int num;
	rosShmData_t *shm;
	SEM *sem;
	rosBlockInitResult_t res;
	unsigned int strlen = sizeof(char_T)*(PARAM_SIZE(1)+1);

	res = registerRosBlock(S, "rosout", LOGGER, 0);
	shm = res.shm;
	sem = res.sem;
	num = res.num;
	shm->msg.level = PARAM(0)[0]-1;

	ssSetIWorkValue(S,0,num);
	ssSetPWorkValue(S,0,(void *)shm);
	ssSetPWorkValue(S,1,(void *)sem);
#endif
	ssSetRWorkValue(S,0,0.0);
}

static void mdlOutputs(SimStruct *S,int_T tid) {
	InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,0);
	InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,1);
	real_T prev = ssGetRWorkValue(S,0);
	bool dataPort = PARAM(2)[0];
	int_T i;
#ifndef MATLAB_MEX_FILE
	rosShmData_t *shm = (rosShmData_t *)ssGetPWorkValue(S,0);
	SEM *sem = (SEM *)ssGetPWorkValue(S,1);
#endif
	char_T *msg;
	unsigned int strlen = sizeof(char_T)*(PARAM_SIZE(1)+1);

	UNUSED_ARG(tid);	 /* not used in single tasking mode */

	if (U0(0) > 0.5 && U0(0) > prev) {
		msg = (char_T *)malloc(strlen);
		mxGetString(ssGetSFcnParam(S,1), msg, strlen);
#ifndef MATLAB_MEX_FILE
		if (dataPort) {
			for (i = 0; i < ssGetInputPortWidth(S,1); ++i) {
				asprintf(&msg, "%s %f", msg, U1(i));
			}
		}
		if (rt_sem_wait_if(sem) != 0) {
			memcpy(shm->msg.text, msg, MAX_LOG_MSG_SIZE);
			shm->msg.state = NEW_VALUE;
			rt_sem_signal(sem);
		}
#else
		switch ((int)PARAM(0)[0]) {
			case 1: printf("DEBUG"); break;
			case 2: printf("INFO"); break;
			case 3: printf("WARN"); break;
			case 4: printf("ERROR"); break;
			case 5: printf("FATAL"); break;
			default: printf("NONE"); break;
		}
		printf(": %s", msg);
		if (dataPort) {
			for (i = 0; i < ssGetInputPortWidth(S,1); ++i) {
				printf(" %f", U1(i));
			}
		}
		printf("\n");
#endif
		free(msg);
	}
	ssSetRWorkValue(S,0,U0(0));
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
