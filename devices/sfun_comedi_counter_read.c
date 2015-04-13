/*
  COPYRIGHT (C) 2003  Lorenzo Dozio (dozio@aero.polimi.it)
  2009 Guillaume Millet (millet@isir.fr)
  2013 Roland Hopferwieser

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
*/


#define S_FUNCTION_NAME		sfun_comedi_counter_read
#define S_FUNCTION_LEVEL	2

#ifdef MATLAB_MEX_FILE
#include "mex.h"
#endif
#include "simstruc.h"

#define NUMBER_OF_PARAMS	4

#define COMEDI_DEVICE_PARAM	ssGetSFcnParam(S,0)
#define COMEDI_CHANNEL_PARAM	ssGetSFcnParam(S,1)
#define COMEDI_MODE_PARAM	ssGetSFcnParam(S,2)
#define SAMPLE_TIME_PARAM	ssGetSFcnParam(S,3)

#define COMEDI_DEVICE		((uint_T) mxGetPr(COMEDI_DEVICE_PARAM)[0])
#define COMEDI_CHANNEL		((uint_T) mxGetPr(COMEDI_CHANNEL_PARAM)[0])
#define COMEDI_MODE		((uint_T) mxGetPr(COMEDI_MODE_PARAM)[0])
#define SAMPLE_TIME		((real_T) mxGetPr(SAMPLE_TIME_PARAM)[0])

#ifndef MATLAB_MEX_FILE

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>

#include <rtai_lxrt.h>
#include <rtai_comedi.h>

extern void *ComediDev[];
extern int ComediDev_InUse[];
extern int ComediDev_AIInUse[];

#endif

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
  static char_T errMsg[256];
  int_T paramsErr = 0;

  if (mxGetNumberOfElements(COMEDI_CHANNEL_PARAM) != 1) {
    sprintf(errMsg, "Channel parameter must be a scalar.\n");
	++paramsErr;
  }
  if (mxGetNumberOfElements(COMEDI_MODE_PARAM) != 1) {
    sprintf(errMsg, "Counter Mode parameter must be a scalar\n");
	++paramsErr;
  }

  if (paramsErr) {
    ssSetErrorStatus(S, errMsg);
  }
}
#endif

static void mdlInitializeSizes(SimStruct *S)
{
  uint_T i;

  ssSetNumSFcnParams(S, NUMBER_OF_PARAMS);
#if defined(MATLAB_MEX_FILE)
  if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) {
      return;
    }
  } else {
    return;
  }
#endif
  for (i = 0; i < NUMBER_OF_PARAMS; i++) {
    ssSetSFcnParamNotTunable(S, i);
  }
  ssSetNumInputPorts(S, 0);
  ssSetNumOutputPorts(S, 1);
  ssSetOutputPortWidth(S, 0, 1);
  ssSetNumContStates(S, 0);
  ssSetNumDiscStates(S, 0);
  ssSetNumSampleTimes(S, 1);
  ssSetNumPWork(S,1);
  ssSetNumIWork(S,1);
  ssSetNumRWork(S,2);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, SAMPLE_TIME);
  ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
  unsigned int counter_mode = (unsigned int)COMEDI_MODE;
  unsigned int channel = (unsigned int)COMEDI_CHANNEL;

#ifndef MATLAB_MEX_FILE
  void *dev;
  int subdev;
  int index = (int)COMEDI_DEVICE - 1;
  int n_channels;
  char *devname[4] = {"/dev/comedi0","/dev/comedi1","/dev/comedi2","/dev/comedi3"};
  char board[50];
  static char_T errMsg[256];
  comedi_insn insn;
  lsampl_t data[5];
  int retval;
//  comedi_krange krange;
//  double range_min, range_max;

  if (!ComediDev[index]) {
    dev = comedi_open(devname[index]);
    if (!dev) {
      sprintf(errMsg, "Comedi open failed\n");
      ssSetErrorStatus(S, errMsg);
      printf("%s", errMsg);
      return;
    }
    rt_comedi_get_board_name(dev, board);
    printf("COMEDI %s (%s) opened.\n\n", devname[index], board);
    ComediDev[index] = dev;
  } else {
    dev = ComediDev[index];
  }

  if ((subdev = comedi_find_subdevice_by_type(dev, COMEDI_SUBD_COUNTER, 0)) < 0) {
    sprintf(errMsg, "Comedi find_subdevice failed (No counter input)\n");
    ssSetErrorStatus(S, errMsg);
    printf("%s", errMsg);
    comedi_close(dev);
    return;
  }
  if (!ComediDev_AIInUse[index] && comedi_lock(dev, subdev) < 0) {
    sprintf(errMsg, "Comedi lock failed for subdevice %d\n", subdev);
    ssSetErrorStatus(S, errMsg);
    printf("%s", errMsg);
    comedi_close(dev);
    return;
  }

  if ((n_channels = comedi_get_n_channels(dev, subdev)) < 0) {
    sprintf(errMsg, "Comedi get_n_channels failed for subdevice %d\n", subdev);
    ssSetErrorStatus(S, errMsg);
    printf("%s", errMsg);
    comedi_unlock(dev, subdev);
    comedi_close(dev);
    return;
  }

//  if ((comedi_get_krange(dev, subdev, channel, range, &krange)) < 0) {
//    sprintf(errMsg, "Comedi get range failed for subdevice %d\n", subdev);
//    ssSetErrorStatus(S, errMsg);
//    printf("%s", errMsg);
//    comedi_unlock(dev, subdev);
//    comedi_close(dev);
//    return;
//  }

  ComediDev_InUse[index]++;
  ComediDev_AIInUse[index]++;
  /* Initialize counter */
  insn.insn = INSN_CONFIG;
  insn.n = 2;
  insn.subdev = subdev;
  insn.chanspec = channel;
  data[0] = INSN_CONFIG_GPCT_QUADRATURE_ENCODER;
  data[1] = counter_mode;
  insn.data = data;
  if ((retval = comedi_do_insn(dev, &insn)) < 0) {
    sprintf(errMsg, "Comedi comedi_do_insn failed for subdevice %d\n", subdev);
    ssSetErrorStatus(S, errMsg);
    printf("%s", errMsg);
    comedi_unlock(dev, subdev);
    comedi_close(dev);
    return;
  }

  //printf("AI Channel %d - Range : %1.2f [V] - %1.2f [V]\n", channel, range_min, range_max);
  ssGetPWork(S)[0] = (void *)dev;
  ssGetIWork(S)[0] = subdev;
//  ssGetRWork(S)[0] = range_min;
//  ssGetRWork(S)[1] = range_max;
#endif
  printf("Counter Channel %i, counter mode 0x%04x\n", channel, counter_mode);
}
#endif

static void mdlOutputs(SimStruct *S, int_T tid)
{
  double *y = ssGetOutputPortRealSignal(S,0);

#ifndef MATLAB_MEX_FILE
  unsigned int channel = (unsigned int)COMEDI_CHANNEL;
  void *dev        = (void *)ssGetPWork(S)[0];
  int subdev       = ssGetIWork(S)[0];
  lsampl_t data, maxdata = comedi_get_maxdata(dev, subdev, COMEDI_CHANNEL);

  comedi_data_read(dev, subdev, channel, 0, 0, &data);
  *y = (double)data;
#endif
}

static void mdlTerminate(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
  int index  = (int)COMEDI_DEVICE - 1;
  void *dev  = (void *)ssGetPWork(S)[0];
  int subdev = ssGetIWork(S)[0];
  char *devname[4] = {"/dev/comedi0","/dev/comedi1","/dev/comedi2","/dev/comedi3"};

  if (ssGetErrorStatus(S) == NULL) {
    ComediDev_InUse[index]--;
    ComediDev_AIInUse[index]--;
    if (!ComediDev_AIInUse[index]) {
      comedi_unlock(dev, subdev);
    }
    if (!ComediDev_InUse[index]) {
      comedi_close(dev);
      printf("\nCOMEDI %s closed.\n\n", devname[index]);
      ComediDev[index] = NULL;
    }
  }
#endif
}

#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
