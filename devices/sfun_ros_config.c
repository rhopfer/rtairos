/*
 * ROS Config
 */

#define S_FUNCTION_NAME sfun_ros_config
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#ifndef MATLAB_MEX_FILE
#include <ros_defines.h>

extern rosConfig_t rosConfig;

#endif

#define PARAM(element)      mxGetPr(ssGetSFcnParam(S,element))
#define PARAM_SIZE(element) mxGetNumberOfElements(ssGetSFcnParam(S,element))

static void mdlInitializeSizes(SimStruct *S) {
	ssSetNumSFcnParams(S,5);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		return; /* Parameter mismatch will be reported by Simulink */
	}
	ssSetNumContStates(S,0);
	ssSetNumDiscStates(S,0);

	if (!ssSetNumInputPorts(S,0)) return;
	if (!ssSetNumOutputPorts(S,0)) return;

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
	unsigned int strlen;
	char_T *str;
	int_T rate = (int_T)PARAM(0)[0];

	strlen = sizeof(char_T)*(PARAM_SIZE(1)+1);
	str = (char_T *)malloc(strlen);
	mxGetString(ssGetSFcnParam(S,1), str, strlen);
	strncpy(rosConfig.ns, str, MAX_NAMES_SIZE);

	if (rate > 0) {
		rosConfig.rate = rate;
	}
	rosConfig.pubStackSize = PARAM(2)[0];
	rosConfig.subStackSize = PARAM(3)[0];
	rosConfig.exposeParams = PARAM(4)[0] - 1;
	free(str);
#endif
}

static void mdlInitializeSampleTimes(SimStruct *S) {
	ssSetSampleTime(S,0,INHERITED_SAMPLE_TIME);
	ssSetOffsetTime(S,0,FIXED_IN_MINOR_STEP_OFFSET);
}

static void mdlOutputs(SimStruct *S,int_T tid) {
	UNUSED_ARG(tid);
}

static void mdlTerminate(SimStruct *S) {
	UNUSED_ARG(S);
}

#ifdef MATLAB_MEX_FILE   /* Compile as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"   /* Code generation registration */
#endif
