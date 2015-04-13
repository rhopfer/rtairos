/*
 */

#define S_FUNCTION_NAME sfun_rtai_fifo
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#ifndef MATLAB_MEX_FILE
#include <rtai_fifos.h>
#endif

#define U(element)          (*uPtrs[element])
#define PARAM(element)      mxGetPr(ssGetSFcnParam(S,element))
#define PARAM_SIZE(element) mxGetNumberOfElements(ssGetSFcnParam(S,element))

static void mdlInitializeSizes(SimStruct *S) {
	ssSetNumSFcnParams(S,3);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		return; /* Parameter mismatch will be reported by Simulink */
	}
	ssSetNumContStates(S,0);
	ssSetNumDiscStates(S,0);

	if (!ssSetNumInputPorts(S,1)) return;
	ssSetInputPortWidth(S,0,DYNAMICALLY_SIZED);
	ssSetInputPortDirectFeedThrough(S,0,1);

	ssSetNumSampleTimes(S,1);
	ssSetNumRWork(S,0);
	ssSetNumIWork(S,0);
	ssSetNumPWork(S,0);
	ssSetNumModes(S,0);
	ssSetNumNonsampledZCs(S,0);
	ssSetOptions(S,0);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
#ifndef MATLAB_MEX_FILE
	real_T num = *mxGetPr(ssGetSFcnParam(S,0));
	rtf_create(num,0);
	rtf_reset(num);
#endif
}

static void mdlInitializeSampleTimes(SimStruct *S) {
#ifndef MATLAB_MEX_FILE
	real_T sampleTime = *mxGetPr(ssGetSFcnParam(S,1));
	ssSetSampleTime(S,0,sampleTime);
	ssSetOffsetTime(S,0,0.0);
#endif
}

static void mdlOutputs(SimStruct *S,int_T tid) {
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
	real_T num = *mxGetPr(ssGetSFcnParam(S,0));
	unsigned int width = ssGetInputPortWidth(S,0);
	bool addTime = *mxGetPr(ssGetSFcnParam(S,2));
	time_T time = ssGetT(S);
	int_T i;
	char str[255];

	UNUSED_ARG(tid);	 /* not used in single tasking mode */

#ifndef MATLAB_MEX_FILE
	if (addTime) {
		sprintf(str, "%f\t%f", time, U(0));
	} else {
		sprintf(str, "%f", U(0));
	}
	for (i = 1; i < width; i++) {
		sprintf(str, "%s\t%f", str, U(i));
	}
	sprintf(str, "%s\n", str);
	rtf_put(num, &str, strlen(str)*sizeof(char));
#endif
}

static void mdlTerminate(SimStruct *S) {
#ifndef MATLAB_MEX_FILE
	real_T num = *mxGetPr(ssGetSFcnParam(S,0));
	rtf_destroy(num);
#endif
}

#ifdef MATLAB_MEX_FILE   /* Compile as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"   /* Code generation registration */
#endif
