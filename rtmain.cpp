/*
  COPYRIGHT (C) 2003  Lorenzo Dozio (dozio@aero.polimi.it)
  Paolo Mantegazza (mantegazza@aero.polimi.it)
  Roberto Bucher (roberto.bucher@supsi.ch)
  Daniele Gasperini (daniele.gasperini@elet.polimi.it)

  Modified August 2009 by Henrik Slotholt (rtai@slotholt.net)
  Modified 2014 by Roland Hopferwieser

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

#define RTAIROS_VERSION "1.0"
#define RTAIROS_NAME "RTAI-ROS"

#define _XOPEN_SOURCE	600
#define DEFAULT_STACKING 30000
#define DEFAULT_CPUMAP 0xF

extern "C" {
#include <popt.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <getopt.h>
#include <pthread.h>
#include <math.h>
#include <semaphore.h>

#include <unistd.h>

#include "tmwtypes.h"
#include "rtmodel.h"
#include "rt_sim.h"
#include "rt_nonfinite.h"
#include "mdl_info.h"
#include "bio_sig.h"
#include "simstruc.h"

#include <rtai_netrpc.h>
#include <rtai_msg.h>
#include <rtai_mbx.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <rtai_fifos.h>
}

#undef RT	// FIX for boost
#include "ros_defines.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Empty.h>
#include <boost/regex.hpp>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#define RT
#include <sstream>
#include <vector>

#define EXPAND_CONCAT(name1,name2)	name1 ## name2
#define CONCAT(name1,name2)		EXPAND_CONCAT(name1,name2)
#define RT_MODEL			CONCAT(MODEL,_rtModel)

#if TID01EQ == 1
#define FIRST_TID 1
#else
#define FIRST_TID 0
#endif
#ifndef RT
# error "must define RT"
#endif
#ifndef MODEL
# error "must define MODEL"
#endif
#ifndef NUMST
# error "must define number of sample times, NUMST"
#endif
#ifndef NCSTATES
# error "must define NCSTATES"
#endif

extern "C" void MdlInitializeSizes(void);
extern "C" void MdlInitializeSampleTimes(void);
extern "C" void MdlStart(void);
extern "C" void MdlOutputs(int_T tid);
extern "C" void MdlUpdate(int_T tid);
extern "C" void MdlTerminate(void);
#define INITIALIZE_SIZES(M)		MdlInitializeSizes()
#define INITIALIZE_SAMPLE_TIMES(M)	MdlInitializeSampleTimes()
#define START(M)			MdlStart()
#define OUTPUTS(M,tid)			MdlOutputs(tid)
#define UPDATED(M,tid)			MdlUpdate(tid)
#define TERMINATE(M)			MdlTerminate()
#if NCSTATES > 0
extern "C" void rt_ODECreateIntegrationData(RTWSolverInfo *si);
extern "C" void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);
#else
#define rt_ODECreateIntegrationData(si)		rtsiSetSolverName(si,"FixedStepDiscrete")
#define rt_ODEUpdateContinuousStates(si)	rtsiSetT(si,rtsiGetSolverStopTime(si))
#endif

#define XSTR(x)    #x
#define STR(x)     XSTR(x)

extern "C" RT_MODEL *MODEL(void);
static RT_MODEL *rtM;

#define RTAILAB_VERSION         "3.5.1"
#define MAX_NTARGETS		1000
#define MAX_NAMES_SIZE		256
#define RUN_FOREVER		-1.0
#define POLL_PERIOD		100000000
#define CONNECT_TO_TARGET	0
#define DISCONNECT_TO_TARGET	1
#define START_TARGET		2
#define STOP_TARGET		3
#define UPDATE_PARAM		4

#define MAX_DATA_SIZE		512

#define rt_MainTaskPriority		97
#define rt_HostInterfaceTaskPriority	96
#define rt_RosInterfaceTaskPriority	    95

typedef struct rtTargetParamInfo {
  char modelName[MAX_NAMES_SIZE];
  char blockName[MAX_NAMES_SIZE];
  char paramName[MAX_NAMES_SIZE];
  unsigned int nRows;
  unsigned int nCols;
  unsigned int dataType;
  unsigned int dataClass;
  double dataValue[MAX_DATA_SIZE];
} rtTargetParamInfo;

static sem_t err_sem;
static RT_TASK *rt_MainTask;

static pthread_t rt_HostInterfaceThread;
static RT_TASK *rt_HostInterfaceTask;

static pthread_t rt_BaseRateThread;
static RT_TASK *rt_BaseRateTask;

#ifdef MULTITASKING
static pthread_t *rt_SubRateThreads;
static RT_TASK **rt_SubRateTasks;
#endif

static pthread_t rosThread;
static RT_TASK *rt_rosTask;

const char *HostInterfaceTaskName     = "HIT";
const char *TargetScopeMbxID          = "RTS";
const char *TargetALogMbxID           = "RAL";
const char *TargetLogMbxID            = "RTL";
const char *TargetLedMbxID            = "RTE";
const char *TargetMeterMbxID          = "RTM";
const char *TargetSynchronoscopeMbxID = "RTY";
const char *RosTaskID                 = "ROS";
const char *RosShmID                  = "SHM";
const char *RosSemID                  = "SEM";

static volatile int Verbose       = 0;
static volatile int UseSoftRT     = 0;
static volatile int WaitToStart   = 0;
static volatile int IsRunning     = 0;
static volatile int ExternalTimer = 0;
static volatile int OneShot       = 0;
static volatile int Priority      = 0;
static volatile int CpuMap	      = 0xF;
static volatile int StackInc	  = 30000;
static volatile int endBaseRate   = 0;
static volatile int endInterface  = 0;
static float FinalTime            = RUN_FOREVER;
static RTIME rt_BaseRateTick;
#ifdef MULTITASKING
static volatile int endSubRate    = 0;
#endif
static int EnableHostInterface = 0;

static volatile int endex;
static volatile bool endRos = 0;
static volatile bool startRos = 0;

#ifdef TASKDURATION
RTIME RTTSKinit=0, RTTSKend=0;
#endif

#define MAX_RTAI_SCOPES	1000
#define MAX_RTAI_LOGS	1000
#define MAX_RTAI_LEDS	1000
#define MAX_RTAI_METERS	1000
#define MAX_RTAI_SYNCHS	1000

SimStruct *rtaiScope[MAX_RTAI_SCOPES];
SimStruct *rtaiALog[MAX_RTAI_LOGS];
SimStruct *rtaiLog[MAX_RTAI_LOGS];
SimStruct *rtaiLed[MAX_RTAI_LEDS];
SimStruct *rtaiMeter[MAX_RTAI_METERS];
SimStruct *rtaiSynchronoscope[MAX_RTAI_SYNCHS];

rosConfig_t rosConfig = {ROS_SAMPLETIME, "", true, PUBLISHER_STACK_SIZE, SUBSCRIBER_STACK_SIZE};
rosBlockConfig_t rosBlockConfigs[MAX_ROS_BLOCKS];
unsigned int numRosBlocks = 0;
const char *rosNode = STR(MODEL);


#define MAX_COMEDI_DEVICES        10

void *ComediDev[MAX_COMEDI_DEVICES];
int ComediDev_InUse[MAX_COMEDI_DEVICES] = {0};
int ComediDev_AIInUse[MAX_COMEDI_DEVICES] = {0};
int ComediDev_AOInUse[MAX_COMEDI_DEVICES] = {0};
int ComediDev_DIOInUse[MAX_COMEDI_DEVICES] = {0};

/*
static void (*WaitTimingEvent)(unsigned long);
static void (*SendTimingEvent)(unsigned long);
static unsigned long TimingEventArg;
static void DummyWait(void) { }
static void DummySend(void) { }
*/



static inline void strncpyz(char *dest, const char *src, size_t n)
{
  if (src != NULL) {
    strncpy(dest, src, n);
    n = strlen(src);
  } else
    n = 0;

  dest[n] = '\0';
}

/* This function is hacked from /usr/src/linux/include/asm-i386/system.h */
static inline void set_double(double *to, double *from)
{
  unsigned long l = ((unsigned long *)from)[0];
  unsigned long h = ((unsigned long *)from)[1];
  __asm__ __volatile__ (
			"1: movl (%0), %%eax; movl 4(%0), %%edx; lock; cmpxchg8b (%0); jnz 1b" : : "D"(to), "b"(l), "c"(h) : "ax", "dx", "memory");
}

#define RT_MODIFY_PARAM_VALUE_IF(rtcase, rttype) \
	case rtcase: \
		for (paramIdx = 0; paramIdx < nParams; paramIdx++) { \
			rttype *param = (rttype *)(pMap[mapOffset + paramIdx]); \
			switch (ptinfoGetClass(ptRec)) { \
				case rt_SCALAR: \
					set_double((double *)param, (double *)_newVal); \
					if (Verbose) { \
						printf("%s : %G\n", mmiGetBlockTuningBlockName(mmi,i), (double)*param); \
					} \
					break; \
				case rt_VECTOR: \
					param[matIdx] = ((double *) _newVal)[0]; \
					if (Verbose) { \
						for (colIdx = 0; colIdx < nCols; colIdx++) { \
							printf("%s : %G\n", mmiGetBlockTuningBlockName(mmi,i), (double)param[colIdx]); \
						} \
					} \
					break; \
				case rt_MATRIX_ROW_MAJOR: \
					param[matIdx] = ((double *) _newVal)[0]; \
					if (Verbose) { \
						for (rowIdx = 0; rowIdx < nRows; rowIdx++) { \
							for(colIdx = 0; colIdx < nCols; colIdx++){ \
								printf("%s : %G\n", mmiGetBlockTuningBlockName(mmi,i), (double)param[rowIdx*nCols+colIdx]); \
							} \
						} \
					} \
					break; \
				case rt_MATRIX_COL_MAJOR: \
					param[matIdx] = ((double *) _newVal)[0]; \
					if (Verbose) { \
						for (rowIdx = 0; rowIdx < nRows; rowIdx++) { \
							for(colIdx = 0; colIdx < nCols; colIdx++) { \
								printf("%s : %G\n", mmiGetBlockTuningBlockName(mmi,i), (double)param[colIdx*nRows+rowIdx]); \
							} \
						} \
					} \
					break; \
				default: \
					return(1); \
			} \
		} \
		break;

int_T rt_ModifyParameterValue(void *mpi, int i, int matIdx, void *_newVal)
{
  ModelMappingInfo *mmi;
  ParameterTuning *ptRec;
  void * const     *pMap;
  uint_T nRows;
  uint_T nCols;
  uint_T nParams;
  uint_T mapOffset;
  uint_T paramIdx;
  uint_T rowIdx;
  uint_T colIdx;

  mmi   = (ModelMappingInfo *)mpi;
  ptRec = (ParameterTuning*)mmiGetBlockTuningParamInfo(mmi,i);
  pMap  = mmiGetParametersMap(mmi);
  nRows = ptinfoGetNumRows(ptRec);
  nCols = ptinfoGetNumCols(ptRec);
  nParams   = ptinfoGetNumInstances(ptRec);
  mapOffset = ptinfoGetParametersOffset(ptRec);

  switch (ptinfoGetDataTypeEnum(ptRec)) {
    RT_MODIFY_PARAM_VALUE_IF(SS_DOUBLE, real_T)
      RT_MODIFY_PARAM_VALUE_IF(SS_SINGLE, real32_T)
      RT_MODIFY_PARAM_VALUE_IF(SS_INT8, int8_T)
      RT_MODIFY_PARAM_VALUE_IF(SS_UINT8, uint8_T)
      RT_MODIFY_PARAM_VALUE_IF(SS_INT16, int16_T)
      RT_MODIFY_PARAM_VALUE_IF(SS_UINT16, uint16_T)
      RT_MODIFY_PARAM_VALUE_IF(SS_INT32, int32_T)
      RT_MODIFY_PARAM_VALUE_IF(SS_UINT32, uint32_T)
      RT_MODIFY_PARAM_VALUE_IF(SS_BOOLEAN, boolean_T)
      default:
    return(1);
  }

  return(0);
}

#define RT_GET_PARAM_INFO_IF(rtcase, rttype) \
	case rtcase: \
		for (paramIdx = 0; paramIdx < nParams; paramIdx++) { \
			rttype *param = (rttype *)(pMap[mapOffset + paramIdx]); \
			switch (ptinfoGetClass(ptRec)) { \
				case rt_SCALAR: \
					rtpi->dataValue[0] = *param; \
					break; \
				case rt_VECTOR: \
					for (colIdx = 0; colIdx < nCols; colIdx++) { \
						rtpi->dataValue[colIdx] = param[colIdx]; \
					} \
					break; \
				case rt_MATRIX_ROW_MAJOR: \
					for (rowIdx = 0; rowIdx < nRows; rowIdx++) { \
						for (colIdx = 0; colIdx < nCols; colIdx++) { \
							rtpi->dataValue[rowIdx*nCols+colIdx] = param[rowIdx*nCols+colIdx]; \
						} \
					} \
					break; \
				case rt_MATRIX_COL_MAJOR: \
					for (rowIdx = 0; rowIdx < nRows; rowIdx++) { \
						for (colIdx = 0; colIdx < nCols; colIdx++) { \
							rtpi->dataValue[colIdx*nRows+rowIdx] = param[colIdx*nRows+rowIdx]; \
						} \
					} \
					break; \
				default: \
					return(1); \
			} \
		} \
		break;

int_T rt_GetParameterInfo(void *mpi, rtTargetParamInfo *rtpi, int i)
{
  ModelMappingInfo *mmi;
  ParameterTuning *ptRec;
  void * const     *pMap;
  uint_T nRows;
  uint_T nCols;
  uint_T nParams;
  uint_T mapOffset;
  uint_T paramIdx;
  uint_T rowIdx;
  uint_T colIdx;
  uint_T dataType;
  uint_T dataClass;

  mmi   = (ModelMappingInfo *)mpi;

  ptRec = (ParameterTuning*)mmiGetBlockTuningParamInfo(mmi,i);
  pMap  = mmiGetParametersMap(mmi);
  nRows = ptinfoGetNumRows(ptRec);
  nCols = ptinfoGetNumCols(ptRec);
  nParams   = ptinfoGetNumInstances(ptRec);
  mapOffset = ptinfoGetParametersOffset(ptRec);
  dataType  = ptinfoGetDataTypeEnum(ptRec);
  dataClass = ptinfoGetClass(ptRec);

  strncpyz(rtpi->modelName, STR(MODEL), MAX_NAMES_SIZE);
  strncpyz(rtpi->blockName, mmiGetBlockTuningBlockName(mmi,i), MAX_NAMES_SIZE);
  strncpyz(rtpi->paramName, mmiGetBlockTuningParamName(mmi,i), MAX_NAMES_SIZE);
  rtpi->dataType  = dataType;
  rtpi->dataClass = dataClass;
  rtpi->nRows = nRows;
  rtpi->nCols = nCols;

  switch (ptinfoGetDataTypeEnum(ptRec)) {
    RT_GET_PARAM_INFO_IF(SS_DOUBLE, real_T)
      RT_GET_PARAM_INFO_IF(SS_SINGLE, real32_T)
      RT_GET_PARAM_INFO_IF(SS_INT8, int8_T)
      RT_GET_PARAM_INFO_IF(SS_UINT8, uint8_T)
      RT_GET_PARAM_INFO_IF(SS_INT16, int16_T)
      RT_GET_PARAM_INFO_IF(SS_UINT16, uint16_T)
      RT_GET_PARAM_INFO_IF(SS_INT32, int32_T)
      RT_GET_PARAM_INFO_IF(SS_UINT32, uint32_T)
      RT_GET_PARAM_INFO_IF(SS_BOOLEAN, boolean_T)
      default:
    return(1);
  }

  return(0);
}

#ifdef MULTITASKING
static void *rt_SubRate(int *arg)
{
  int sample, i;
  char myname[7];
  int rt_SubRateTaskPriority;

  sample = arg[0];
  rt_SubRateTaskPriority = arg[1];

  rt_allow_nonroot_hrt();
  for (i = 0; i < MAX_NTARGETS; i++) {
    sprintf(myname, "TSR%d", i);
    if (!rt_get_adr(nam2num(myname))) break;
  }
  if (!(rt_SubRateTasks[sample] = rt_task_init_schmod(nam2num(myname), rt_SubRateTaskPriority, 0, 0, SCHED_FIFO, CpuMap))) {
    printf("Cannot init rt_SubRateTask #%d\n", sample);
    return (void *)1;
  }
  //	sem_post(&err_sem);

  iopl(3);
  rt_task_use_fpu(rt_SubRateTasks[sample], 1);
  rt_grow_and_lock_stack(StackInc);

  if (!UseSoftRT) {
    rt_make_hard_real_time();
  }
  while (!endSubRate) {
    rt_task_suspend(rt_SubRateTasks[sample]);
    if (endSubRate) break;
    OUTPUTS(rtM, sample);
    UPDATED(rtM, sample);
    rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(rtM), rtmGetTimingData(rtM), sample);
  }
  if (!UseSoftRT) {
    rt_make_soft_real_time();
  }

  rt_task_delete(rt_SubRateTasks[sample]);
  return (void *)(rt_SubRateTasks[sample] = 0);
}
#endif

static void *rt_BaseRate(void *args)
{
  real_T tnext;
  char myname[7];
  int i;
#ifdef MULTITASKING
  int_T  sample, *sampleHit = rtmGetSampleHitPtr(rtM);
#endif

  int rt_BaseRateTaskPriority = *((int *)args);

  rt_allow_nonroot_hrt();
  for (i = 0; i < MAX_NTARGETS; i++) {
    sprintf(myname, "TBR%d", i);
    if (!rt_get_adr(nam2num(myname))) break;
  }
  if (!(rt_BaseRateTask = rt_task_init_schmod(nam2num(myname), rt_BaseRateTaskPriority, 0, 0, SCHED_FIFO, CpuMap))) {
    printf("Cannot init rt_BaseRateTask\n");
    return (void *)1;
  }
  sem_post(&err_sem);

  iopl(3);
  rt_task_use_fpu(rt_BaseRateTask, 1);
  rt_grow_and_lock_stack(StackInc);
  if (!UseSoftRT) {
    rt_make_hard_real_time();
  }
  rt_rpc(rt_MainTask,0,(void *)myname);
  rt_task_make_periodic(rt_BaseRateTask, rt_get_time() + rt_BaseRateTick, rt_BaseRateTick);

  while (!endBaseRate) {
#ifdef TASKDURATION
    RTTSKend=rt_get_cpu_time_ns();
#endif
//    WaitTimingEvent(TimingEventArg);
    if (!ExternalTimer) rt_task_wait_period();
    if (endBaseRate) break;

#ifdef TASKDURATION
    RTTSKinit=rt_get_cpu_time_ns();
#endif

#ifdef MULTITASKING
    tnext = rt_SimUpdateDiscreteEvents(rtmGetNumSampleTimes(rtM), rtmGetTimingData(rtM), rtmGetSampleHitPtr(rtM), rtmGetPerTaskSampleHitsPtr(rtM));
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(rtM), tnext);
    for (sample = FIRST_TID + 1; sample < NUMST; sample++) {
      if (sampleHit[sample]) {
	rt_task_resume(rt_SubRateTasks[sample]);
      }
    }
    OUTPUTS(rtM, FIRST_TID);
    UPDATED(rtM, FIRST_TID);
    if (rtmGetSampleTime(rtM, 0) == CONTINUOUS_SAMPLE_TIME) {
      rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(rtM));
    } else {
      rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(rtM), rtmGetTimingData(rtM), 0);
    }
#if FIRST_TID == 1
    rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(rtM), rtmGetTimingData(rtM), 1);
#endif
#else
    tnext = rt_SimGetNextSampleHit();
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(rtM), tnext);
    OUTPUTS(rtM, 0);
    UPDATED(rtM, 0);
    rt_SimUpdateDiscreteTaskSampleHits(rtmGetNumSampleTimes(rtM), rtmGetTimingData(rtM), rtmGetSampleHitPtr(rtM), rtmGetTPtr(rtM));
    if (rtmGetSampleTime(rtM,0) == CONTINUOUS_SAMPLE_TIME) {
      rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(rtM));
    }
#endif

  }

  if (!UseSoftRT) {
    rt_make_soft_real_time();
  }

  rt_task_delete(rt_BaseRateTask);
  return 0;
}

static char_T* ssGetBlockName(SimStruct *S) {
	char *path = (char *)ssGetPath(S);
	char *slash = strrchr(path, '/');
	if (slash != NULL) {
			return slash + 1;
	}
	return path;
}

/**** ROS ****/
static std::string cleanupName(std::string str) {
	str = boost::regex_replace(str, boost::regex(":"), "/");
	str = boost::regex_replace(str, boost::regex("^[^a-zA-Z/]"), "X");
	str = boost::regex_replace(str, boost::regex("[^a-zA-Z0-9/_]"), "_");
	return str;
}

static void secureRosConfig() {
	std::string ns(rosConfig.ns);
	ns = cleanupName(ns);
	memcpy(rosConfig.ns, ns.c_str(), MAX_NAMES_SIZE);
	if (rosConfig.rate <= 0) {
		ROS_WARN("Illegal ROS rate %f found. Set to default %f", rosConfig.rate, (double)ROS_SAMPLETIME);
		rosConfig.rate = ROS_SAMPLETIME;
	}
}

struct RosObject {
	std::string name;			// ROS name
	std::string refName;		// ROS frame_id
	char shmName[7];			// Shared memory ID
	char semName[7];			// Semaphore ID
	unsigned int num;			// Position in rosBlockConfig[]
	rosShmData_t *shm;
	SEM *sem;

	RosObject(rosBlockConfig_t conf, unsigned int num) {
		std::string name(conf.name);
		if (name.length() == 0) {
			char *blockName = ssGetBlockName(conf.S);
			ROS_INFO("Name/Topic of block '%s' not set, block name will be used", blockName);
			name = std::string(blockName);
		}
		std::string refName(conf.refName);

		this->name = cleanupName(name);
		if (this->name != name) {
			ROS_WARN("Invalid name '%s' of block '%s' replaced by '%s'", conf.name, ssGetBlockName(conf.S), this->name.c_str());
		}
		this->refName = cleanupName(refName);
		this->num = num;

		// Init shared memory
		memcpy(this->shmName, conf.shmName, 7);
		this->shm = (rosShmData_t*)rt_shm_alloc(nam2num(this->shmName), sizeof(rosShmData_t), USE_VMALLOC);
		memcpy(this->semName, conf.semName, 7);
		this->sem = (SEM*)rt_get_adr(nam2num(this->semName));
	}

	~RosObject() {
		rt_shm_free(nam2num(this->shmName));
	}

	bool sem_wait() {
		int sem_ret;

		if (this->sem == NULL) {
			ROS_ERROR("No semaphore found for %s", name.c_str());
			return false;
		}
		sem_ret = rt_sem_wait_until(sem, (RTIME)1000000000.0*ROS_SEMAPHORE_TIMEOUT);
		if (sem_ret == 0) {
			ROS_ERROR("Semaphore timeout for %s", name.c_str());
			return false;
		}
		return true;
	}

	void sem_signal() {
		rt_sem_signal(this->sem);
	}

	void logNew() {
		rosShmData_t shmData;
		shmData.msg.state = 0;

		if (!this->sem_wait()) return;
		if (this->shm->msg.state > 0) {
			memcpy(&shmData, (rosShmData_t *)this->shm, sizeof(rosShmData_t));
			this->shm->msg.state = 0;
		}
		this->sem_signal();
		this->logNew(shmData.msg);
	}

	void logNew(rosMsg_t msg) {
		if (msg.state > 0) {
			ROS_LOG((ros::console::Level)msg.level, ROSCONSOLE_DEFAULT_NAME, "%s", msg.text);
		}
	}

};

struct RosPublisher : RosObject {
	int subType;
	ros::Publisher pub;

	RosPublisher(rosBlockConfig_t conf, unsigned int num) : RosObject(conf, num) {
		this->subType = conf.subType;
	};

	void publish() {
		rosShmData_t shmData;

		if (!this->sem_wait()) return;
		memcpy(&shmData, (rosShmData_t *)this->shm, sizeof(rosShmData_t));
		this->shm->msg.state = 0;
		this->shm->state = 0;
		this->sem_signal();
		this->logNew(shmData.msg);

		if (shmData.state > 0) {
			if (subType == PUBLISHER_FLOAT64) {
				std_msgs::Float64 msg;

				msg.data = shmData.data[0];

				pub.publish(msg);

			} else if (subType == PUBLISHER_FLOAT64ARRAY) {
				std_msgs::Float64MultiArray msg;
				std_msgs::MultiArrayDimension dim;

				for (unsigned int j = 0; j < shmData.length; ++j) {
					msg.data.push_back(shmData.data[j]);
				}
				dim.size = shmData.length;
				dim.stride = dim.size;
				msg.layout.dim.push_back(dim);

				pub.publish(msg);

			} else if (subType == PUBLISHER_BOOL) {
				std_msgs::Bool msg;

				msg.data = (bool)shmData.data[0];

				pub.publish(msg);

			} else if (subType == PUBLISHER_INT32) {
				std_msgs::Int32 msg;

				msg.data = (int)shmData.data[0];

				pub.publish(msg);

			} else if (subType == PUBLISHER_POINT) {
				geometry_msgs::Point msg;

				msg.x = shmData.data[0];
				msg.y = shmData.data[1];
				msg.z = shmData.data[2];

				pub.publish(msg);

			} else if (subType == PUBLISHER_POINT_STAMPED) {
				geometry_msgs::PointStamped msg;

				msg.point.x = shmData.data[0];
				msg.point.y = shmData.data[1];
				msg.point.z = shmData.data[2];
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = this->refName;

				pub.publish(msg);

			} else if (subType == PUBLISHER_TWIST) {
				geometry_msgs::Twist msg;

				msg.linear.x = shmData.data[0];
				msg.linear.y = shmData.data[1];
				msg.linear.z = shmData.data[2];
				msg.angular.x = shmData.data[3];
				msg.angular.y = shmData.data[4];
				msg.angular.z = shmData.data[5];

				pub.publish(msg);

			} else if (subType == PUBLISHER_TWIST_STAMPED) {
				geometry_msgs::TwistStamped msg;

				msg.twist.linear.x = shmData.data[0];
				msg.twist.linear.y = shmData.data[1];
				msg.twist.linear.z = shmData.data[2];
				msg.twist.angular.x = shmData.data[3];
				msg.twist.angular.y = shmData.data[4];
				msg.twist.angular.z = shmData.data[5];
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = this->refName;

				pub.publish(msg);

			} else if (subType == PUBLISHER_POSE2D) {
				geometry_msgs::Pose2D msg;

				msg.x = shmData.data[0];
				msg.y = shmData.data[1];
				msg.theta = shmData.data[2];

				pub.publish(msg);

			} else if (subType == PUBLISHER_TIME) {
				std_msgs::Time msg;

				msg.data = ros::Time(shmData.data[0]);

				pub.publish(msg);
			}
		}
	}
};

struct RosSubscriber : RosObject {
	int subType;
	ros::Subscriber sub;

	RosSubscriber(rosBlockConfig_t conf, unsigned int num) : RosObject(conf, num) {
		this->subType = conf.subType;
	};

	void callback(const std_msgs::Float64MultiArray msg) {
		if (!this->sem_wait()) return;
		for (unsigned int i = 0; i < shm->length; ++i) {
			shm->data[i] = msg.data[i];
		}
		this->sem_signal();

		std::stringstream ss;
		for (unsigned int i = 0; i < msg.data.size(); ++i) {
			ss << msg.data[i] << (i == msg.data.size()-1 ? "" : ",");
		}
	}

	void callback(const sensor_msgs::Joy msg) {
		unsigned int axes = msg.axes.size();
		unsigned int buttons = msg.buttons.size();
		if (!this->sem_wait()) return;
		for (unsigned int i = 0; i < shm->length; ++i) {
			if (i < axes) {
				shm->data[i] = msg.axes[i];
			} else if (i < axes + buttons) {
				shm->data[i] = msg.buttons[i-axes];
			} else {
				shm->data[i] = 0.0;
			}
		}
		this->sem_signal();
	}

	void callback(const std_msgs::Float64 msg) {
		if (!this->sem_wait()) return;
		shm->data[0] = msg.data;
		this->sem_signal();
	}

	void callback(const std_msgs::Int32 msg) {
		if (!this->sem_wait()) return;
		shm->data[0] = msg.data;
		this->sem_signal();
	}

	void callback(const std_msgs::Bool msg) {
		if (!this->sem_wait()) return;
		shm->data[0] = msg.data;
		this->sem_signal();
	}

	void callback(const std_msgs::Time msg) {
		double time = msg.data.toSec();
		if (!this->sem_wait()) return;
		shm->data[0] = time;
		this->sem_signal();
	}

	void callback(const geometry_msgs::Twist msg) {
		if (!this->sem_wait()) return;
		shm->data[0] = msg.linear.x;
		shm->data[1] = msg.linear.y;
		shm->data[2] = msg.linear.z;
		shm->data[3] = msg.angular.x;
		shm->data[4] = msg.angular.y;
		shm->data[5] = msg.angular.z;
		this->sem_signal();
	}

	void callback(const geometry_msgs::TwistStamped msg) {
		if (!this->sem_wait()) return;
		shm->data[0] = msg.twist.linear.x;
		shm->data[1] = msg.twist.linear.y;
		shm->data[2] = msg.twist.linear.z;
		shm->data[3] = msg.twist.angular.x;
		shm->data[4] = msg.twist.angular.y;
		shm->data[5] = msg.twist.angular.z;
		shm->header.time = msg.header.stamp.toSec();
		shm->header.seq = msg.header.seq;
		this->sem_signal();
	}

	void callback(const geometry_msgs::Point msg) {
		if (!this->sem_wait()) return;
		shm->data[0] = msg.x;
		shm->data[1] = msg.y;
		shm->data[2] = msg.z;
		this->sem_signal();
	}

	void callback(const geometry_msgs::PointStamped msg) {
		if (!this->sem_wait()) return;
		shm->data[0] = msg.point.x;
		shm->data[1] = msg.point.y;
		shm->data[2] = msg.point.z;
		shm->header.time = msg.header.stamp.toSec();
		shm->header.seq = msg.header.seq;
		this->sem_signal();
	}

	void callback(const geometry_msgs::Pose2D msg) {
		if (!this->sem_wait()) return;
		shm->data[0] = msg.x;
		shm->data[1] = msg.y;
		shm->data[2] = msg.theta;
		this->sem_signal();
	}

};

struct RosBroadcaster : RosObject {
	tf::TransformBroadcaster br;

	RosBroadcaster(rosBlockConfig_t conf, unsigned int num) : RosObject(conf, num) { };

	void send() {
		rosShmData_t shmData;
		ros::Time time;

		if (!this->sem_wait()) return;
		memcpy(&shmData, (rosShmData_t *)this->shm, sizeof(rosShmData_t));
		this->shm->msg.state = 0;
		this->shm->state = 0;
		this->sem_signal();
		this->logNew(shmData.msg);

		tf::Transform transform;
		transform.setOrigin( tf::Vector3(shmData.data[0], shmData.data[1], shmData.data[2]) );
		tf::Quaternion q;
		q.setRPY(shmData.data[3], shmData.data[4], shmData.data[5]);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), this->refName, this->name));
	}

};

struct RosService : RosObject {
	ros::ServiceServer srv;

	RosService(rosBlockConfig_t conf, unsigned int num) : RosObject(conf, num) { };

	bool callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
		ros::Rate loop_rate(rosConfig.rate);
		if (!this->sem_wait()) return false;
		this->shm->state = STATE_REQUEST;
		this->sem_signal();
		ROS_INFO("Service '%s' called", name.c_str());
		return true;
	}
};

struct RosJointState : RosObject {

	RosJointState(rosBlockConfig_t conf, unsigned int num) : RosObject(conf, num) { };

	void fill(sensor_msgs::JointState *joint_state, unsigned int num) {
		rosShmData_t shmData;

		if (!this->sem_wait()) return;
		shmData.length = this->shm->length;
		memcpy(shmData.data, this->shm->data, sizeof(double)*shmData.length);
		this->sem_signal();

		joint_state->name[num] = this->name;
		joint_state->position[num] = shmData.data[0];
		joint_state->velocity[num] = shmData.data[1];
		joint_state->effort[num] = shmData.data[2];
	}
};

static void rosCleanParams() {
	ros::NodeHandle nh;
	nh.deleteParam(std::string(rosNode));
}

static void rosPublishParams() {
	ros::NodeHandle nh;
	uint_T nBlockParams;
	rtTargetParamInfo rtParameters;
	ModelMappingInfo *MMI;

	// Cleanup first
	rosCleanParams();

	MMI = (ModelMappingInfo *)rtmGetModelMappingInfo(rtM);
	nBlockParams = mmiGetNumBlockParams(MMI);
	for (uint_T i = 0; i < nBlockParams; i++) {
		rt_GetParameterInfo(MMI, &rtParameters, i);
		std::stringstream ss;
		ss << rosNode << &rtParameters.blockName[strlen(STR(MODEL))] << '/' << rtParameters.paramName;

		std::string str = cleanupName(ss.str());

		uint_T len = rtParameters.nCols*rtParameters.nRows;
		if (len > MAX_DATA_SIZE) {
			ROS_ERROR("rosPublishParams: MAX_DATA_SIZE exceeded: %s, %i", str.c_str(), len);
		} if (len > 1) {
			std::vector<double> val;
			for (uint_T j = 0; j < len; j++) {
				val.push_back(rtParameters.dataValue[j]);
			}
			nh.setParam(str.c_str(), val);
		} else {
			double val = rtParameters.dataValue[0];
			nh.setParam(str.c_str(), val);
		}
	}

}

static bool rosSetParams(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	uint_T nBlockParams;
	rtTargetParamInfo rtParameters;
	ModelMappingInfo *MMI;
	ros::NodeHandle nh;

	MMI = (ModelMappingInfo *)rtmGetModelMappingInfo(rtM);
	nBlockParams = mmiGetNumBlockParams(MMI);
	for (uint_T i = 0; i < nBlockParams; i++) {
		rt_GetParameterInfo(MMI, &rtParameters, i);
		std::stringstream ss;
		ss << rosNode << &rtParameters.blockName[strlen(STR(MODEL))] << '/' << rtParameters.paramName;
		std::string str = cleanupName(ss.str());

		if (!nh.hasParam(str.c_str())) {
			ROS_DEBUG("Parameter /%s unknown; ignored", str.c_str());
			continue;
		}

		uint_T len = rtParameters.nCols*rtParameters.nRows;
		if (len > MAX_DATA_SIZE) {
			ROS_ERROR("rosSetParams: MAX_DATA_SIZE exceeded: %s, %i", str.c_str(), len);
		} else if (len > 1) {
			std::vector<double> newVal;
			nh.getParam(str.c_str(), newVal);
			if (newVal.size() == len) {
				int modified = 0;
				for (uint_T j = 0; j < len; j++) {
					if (rtParameters.dataValue[j] != newVal[j]) {
						rt_ModifyParameterValue(MMI, i, j, &newVal[j]);
						modified = 1;
					}
				}
				if (modified) {
					ss.str("");
					for (unsigned int i = 0; i < len; ++i) {
						ss << rtParameters.dataValue[i] << (i == len-1 ? "" : ",");
					}
					std::string s1 = ss.str();
					ss.str("");
					for (unsigned int i = 0; i < len; ++i) {
						ss << newVal[i] << (i == len-1 ? "" : ",");
					}
					std::string s2 = ss.str();
					ROS_DEBUG("Parameter /%s ([%s]) changed to [%s]", str.c_str(), s1.c_str(), s2.c_str());
				}
			} else {
				ROS_WARN("Wrong parameter length in /%s; expected %i, got %i; ignored.", str.c_str(), len, newVal.size());
			}
		} else {
			double newVal;
			nh.getParam(str.c_str(), newVal);
			if (rtParameters.dataValue[0] != newVal) {
				rt_ModifyParameterValue(MMI, i, 0, &newVal);
				ROS_DEBUG("Parameter /%s (%f) changed to %f", str.c_str(), rtParameters.dataValue[0], newVal);
			}
		}
	}
	rosPublishParams();
	return true;
}

static bool rosStart(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	ROS_DEBUG("Service start called");
	rt_task_resume(rt_MainTask);
	return true;
}

static bool rosRefreshParams(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	rosPublishParams();
	return true;
}

static void rosInitMsg(int pub, int sub, int srv, int br, int jnt) {
	if (Verbose) {
		std::cout << "\nROS Task\n========\n";
		std::cout << "  Rate                   : " << rosConfig.rate << " [Hz]" << std::endl;
		std::cout << "  Node                   : " << rosNode << std::endl;
		std::cout << "  Namespace              : " << rosConfig.ns << std::endl;
		std::cout << "  Publisher stack size   : " << rosConfig.pubStackSize << std::endl;
		std::cout << "  Subscriber stack size  : " << rosConfig.subStackSize << std::endl;
		std::cout << "  Expose params          : ";
		if (rosConfig.exposeParams > 1) {
			std::cout << "Yes (writeable)";
		} else if (rosConfig.exposeParams > 0)	{
			std::cout << "Yes (read-only)";
		} else {
			std::cout << "No";
		}
		std::cout << std::endl;
		std::cout << "  Number of Publishers   : " << pub << std::endl;
		std::cout << "  Number of Subscribers  : " << sub << std::endl;
		std::cout << "  Number of Services     : " << srv << std::endl;
		std::cout << "  Number of Broadcasters : " << br << std::endl;
		std::cout << "  Number of Joint States : " << jnt << std::endl;
		std::cout << std::endl;
	}
}

static void *rosInterface(void *args) {
	std::vector<RosPublisher *> publishers;
	std::vector<RosSubscriber *> subscribers;
	std::vector<RosService *> services;
	std::vector<RosBroadcaster *> broadcasters;
	std::vector<RosJointState *> jointstates;

	ROS_INFO("Starting ROS Task");

	secureRosConfig();

	ros::NodeHandle nh(rosConfig.ns);
	ros::Publisher joint_pub;

	for (unsigned int i = 0; i < numRosBlocks; ++i) {

		// Services
		if (rosBlockConfigs[i].type == SERVICE) {
			RosService *service = new RosService(rosBlockConfigs[i], i);
			service->srv = nh.advertiseService(service->name, &RosService::callback, service);
			services.push_back(service);

		// Subscribers
		} else if (rosBlockConfigs[i].type == SUBSCRIBER) {
			RosSubscriber *subscriber = new RosSubscriber(rosBlockConfigs[i], i);
			bool unknown = false;

			if (subscriber->subType == SUBSCRIBER_FLOAT64) {
				subscriber->sub = nh.subscribe<std_msgs::Float64>(subscriber->name, rosConfig.subStackSize, &RosSubscriber::callback, subscriber);
			} else if (subscriber->subType == SUBSCRIBER_INT32) {
				subscriber->sub = nh.subscribe<std_msgs::Int32>(subscriber->name, rosConfig.subStackSize, &RosSubscriber::callback, subscriber);
			} else if (subscriber->subType == SUBSCRIBER_BOOL) {
				subscriber->sub = nh.subscribe<std_msgs::Bool>(subscriber->name, rosConfig.subStackSize, &RosSubscriber::callback, subscriber);
			} else if (subscriber->subType == SUBSCRIBER_TIME) {
				subscriber->sub = nh.subscribe<std_msgs::Time>(subscriber->name, rosConfig.subStackSize, &RosSubscriber::callback, subscriber);
			} else if (subscriber->subType == SUBSCRIBER_FLOAT64ARRAY) {
				subscriber->sub = nh.subscribe<std_msgs::Float64MultiArray>(subscriber->name, rosConfig.subStackSize, &RosSubscriber::callback, subscriber);
			} else if (subscriber->subType == SUBSCRIBER_POINT) {
				subscriber->sub = nh.subscribe<geometry_msgs::Point>(subscriber->name, rosConfig.subStackSize, &RosSubscriber::callback, subscriber);
			} else if (subscriber->subType == SUBSCRIBER_POINT_STAMPED) {
				subscriber->sub = nh.subscribe<geometry_msgs::PointStamped>(subscriber->name, rosConfig.subStackSize, &RosSubscriber::callback, subscriber);
			} else if (subscriber->subType == SUBSCRIBER_TWIST) {
				subscriber->sub = nh.subscribe<geometry_msgs::Twist>(subscriber->name, rosConfig.subStackSize, &RosSubscriber::callback, subscriber);
			} else if (subscriber->subType == SUBSCRIBER_TWIST_STAMPED) {
				subscriber->sub = nh.subscribe<geometry_msgs::TwistStamped>(subscriber->name, rosConfig.subStackSize, &RosSubscriber::callback, subscriber);
			} else if (subscriber->subType == SUBSCRIBER_POSE2D) {
				subscriber->sub = nh.subscribe<geometry_msgs::Pose2D>(subscriber->name, rosConfig.subStackSize, &RosSubscriber::callback, subscriber);
			} else if (subscriber->subType == SUBSCRIBER_JOY) {
				subscriber->sub = nh.subscribe<sensor_msgs::Joy>(subscriber->name, rosConfig.subStackSize, &RosSubscriber::callback, subscriber);
			} else {
				unknown = true;
				ROS_ERROR("Subscriber: Unknown message type %i in %s", subscriber->subType, subscriber->name.c_str());
			}
			if (!unknown) {
				subscribers.push_back(subscriber);
			} else {
				delete(subscriber);
			}

		// Publishers
		} else if (rosBlockConfigs[i].type == PUBLISHER) {
			RosPublisher *publisher = new RosPublisher(rosBlockConfigs[i], i);
			bool unknown = false;

			if (publisher->subType == PUBLISHER_FLOAT64) {
				publisher->pub = nh.advertise<std_msgs::Float64>(publisher->name, rosConfig.pubStackSize);
			} else if (publisher->subType == PUBLISHER_INT32) {
				publisher->pub = nh.advertise<std_msgs::Int32>(publisher->name, rosConfig.pubStackSize);
			} else if (publisher->subType == PUBLISHER_BOOL) {
				publisher->pub = nh.advertise<std_msgs::Bool>(publisher->name, rosConfig.pubStackSize);
			} else if (publisher->subType == PUBLISHER_TIME) {
				publisher->pub = nh.advertise<std_msgs::Time>(publisher->name, rosConfig.pubStackSize);
			} else if (publisher->subType == PUBLISHER_FLOAT64ARRAY) {
				publisher->pub = nh.advertise<std_msgs::Float64MultiArray>(publisher->name, rosConfig.pubStackSize);
			} else if (publisher->subType == PUBLISHER_POINT) {
				publisher->pub = nh.advertise<geometry_msgs::Point>(publisher->name, rosConfig.pubStackSize);
			} else if (publisher->subType == PUBLISHER_POINT_STAMPED) {
				publisher->pub = nh.advertise<geometry_msgs::PointStamped>(publisher->name, rosConfig.pubStackSize);
			} else if (publisher->subType == PUBLISHER_TWIST) {
				publisher->pub = nh.advertise<geometry_msgs::Twist>(publisher->name, rosConfig.pubStackSize);
			} else if (publisher->subType == PUBLISHER_TWIST_STAMPED) {
				publisher->pub = nh.advertise<geometry_msgs::TwistStamped>(publisher->name, rosConfig.pubStackSize);
			} else if (publisher->subType == PUBLISHER_POSE2D) {
				publisher->pub = nh.advertise<geometry_msgs::Pose2D>(publisher->name, rosConfig.pubStackSize);
			} else {
				ROS_ERROR("Publisher: Unknown message type in %s", publisher->name.c_str());
			}
			if (!unknown) {
				publishers.push_back(publisher);
			} else {
				delete(publisher);
			}

		} else if (rosBlockConfigs[i].type == BROADCASTER) {
			RosBroadcaster *broadcaster = new RosBroadcaster(rosBlockConfigs[i], i);
			broadcasters.push_back(broadcaster);

		} else if (rosBlockConfigs[i].type == JOINTSTATE) {
			RosJointState *state = new RosJointState(rosBlockConfigs[i], i);
			jointstates.push_back(state);

		} else if (rosBlockConfigs[i].type == LOGGER) {
			RosPublisher *publisher = new RosPublisher(rosBlockConfigs[i], i);
			publishers.push_back(publisher);

		} else {
			ROS_ERROR("Unknown block type %s : %i", rosBlockConfigs[i].name, rosBlockConfigs[i].type);
		}

	}


	// Start service
	ros::ServiceServer startSrv;
	char srvName[100];
	if (WaitToStart) {
		snprintf(srvName, 100, "%s/start", rosNode);
		startSrv = nh.advertiseService(srvName, &rosStart);
	}

	// Parameters
	ros::ServiceServer srvSetParam;
	ros::ServiceServer srvRefreshParam;
	if (rosConfig.exposeParams > 0) {
		rosPublishParams();
		snprintf(srvName, 100, "/%s/refresh_parameters", rosNode);
		srvRefreshParam = nh.advertiseService(srvName, &rosRefreshParams);
		if (rosConfig.exposeParams > 1) {
			snprintf(srvName, 100, "/%s/set_parameters", rosNode);
			srvSetParam = nh.advertiseService(srvName, &rosSetParams);
		}
	}

	if (jointstates.size() > 0) {
		joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", rosConfig.pubStackSize);
	}

	rosInitMsg(publishers.size(), subscribers.size(), services.size(), broadcasters.size(), jointstates.size());


	// Init soft realtime task in user space
	rt_allow_nonroot_hrt();
	char RosInterfaceTaskName[7];
	for (unsigned int i = 0; i < 10; ++i) {
		sprintf(RosInterfaceTaskName, "%s%d", RosTaskID, i);
		if (!rt_get_adr(nam2num(RosInterfaceTaskName))) break;
	}
	if (!(rt_rosTask = rt_task_init(nam2num(RosInterfaceTaskName), rt_RosInterfaceTaskPriority, 0, 0))) {
		fprintf(stderr, "Cannot init rt_rosTask\n");
	}

	ros::Rate loop_rate(rosConfig.rate);
	loop_rate.sleep();
	ROS_INFO("Entering loop");
	while (!endRos) {
		for (unsigned int i = 0; i < publishers.size(); ++i) {
			publishers[i]->publish();
		}
		for (unsigned int i = 0; i < subscribers.size(); ++i) {
			subscribers[i]->logNew();
		}
		for (unsigned int i = 0; i < services.size(); ++i) {
			services[i]->logNew();
		}
		for (unsigned int i = 0; i < broadcasters.size(); ++i) {
			broadcasters[i]->send();
		}

		if (jointstates.size() > 0) {
			int size = jointstates.size();
			sensor_msgs::JointState joint_state;
			joint_state.header.stamp = ros::Time::now();
			joint_state.name.resize(size);
			joint_state.position.resize(size);
			joint_state.velocity.resize(size);
			joint_state.effort.resize(size);

			for (unsigned int i = 0; i < jointstates.size(); ++i) {
				jointstates[i]->fill(&joint_state, i);
				jointstates[i]->logNew();
			}
			joint_pub.publish(joint_state);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	rt_task_delete(rt_rosTask);

	// Cleanup
	for (unsigned int i = 0; i < publishers.size(); ++i) {
		delete(publishers[i]);
	}
	publishers.clear();
	for (unsigned int i = 0; i < subscribers.size(); ++i) {
		delete(subscribers[i]);
	}
	subscribers.clear();
	for (unsigned int i = 0; i < services.size(); ++i) {
		delete(services[i]);
	}
	services.clear();
	for (unsigned int i = 0; i < broadcasters.size(); ++i) {
		delete(broadcasters[i]);
	}
	broadcasters.clear();
	for (unsigned int i = 0; i < jointstates.size(); ++i) {
		delete(jointstates[i]);
	}
	jointstates.clear();
	if (rosConfig.exposeParams) {
		rosCleanParams();
	}

	ros::shutdown();

	printf("ROS Task terminated\n");

	return 0;
}


static void *rt_HostInterface(void *args)
{
  RT_TASK *task;
  unsigned int IRequest;
  char Request;
  long len;

  rt_allow_nonroot_hrt();
  if (!(rt_HostInterfaceTask = rt_task_init_schmod(nam2num(HostInterfaceTaskName), rt_HostInterfaceTaskPriority, 0, 0, SCHED_FIFO, 0xFF))) {
    printf("Cannot init rt_HostInterfaceTask\n");
    return (void *)1;
  }
  sem_post(&err_sem);

  while (!endInterface) {
    task = rt_receive(0, &IRequest);
    Request = (char)(IRequest);

    if (endInterface) break;

    switch (Request) {

    case 'c': {
      ModelMappingInfo *MMI;
      uint_T nBlockParams;
      { int Reply;
      MMI = (ModelMappingInfo *)rtmGetModelMappingInfo(rtM);
      nBlockParams = mmiGetNumBlockParams(MMI);
      Reply = (IsRunning << 16) | (nBlockParams & 0xffff); /* max 2^16-1 blocks */
      rt_return(task, Reply);
      }
      { uint_T i;
      rtTargetParamInfo rtParameters;
      rt_receivex(task, &rtParameters, sizeof(char), &len);
      rt_GetParameterInfo(MMI, &rtParameters, 0);
      rt_returnx(task, &rtParameters, sizeof(rtParameters));

      for (i = 0; i < nBlockParams; i++) {
	rt_receivex(task, &rtParameters, sizeof(char), &len);
	rt_GetParameterInfo(MMI, &rtParameters, i);
	rt_returnx(task, &rtParameters, sizeof(rtParameters));
      }
      }
      { int scopeIdx, Reply;
      float samplingTime;
      int ntraces;
      while (1) {
	rt_receivex(task, &scopeIdx, sizeof(int), &len);
	if (scopeIdx < 0) {
	  Reply = scopeIdx;
	  rt_returnx(task, &Reply, sizeof(int));
	  break;
	} else {
	  ntraces = ssGetNumInputPorts(rtaiScope[scopeIdx]);
	  rt_returnx(task, &ntraces, sizeof(int));
	  rt_receivex(task, &scopeIdx, sizeof(int), &len);
	  rt_returnx(task, (char *)ssGetModelName(rtaiScope[scopeIdx]), MAX_NAMES_SIZE*sizeof(char));
		while(1) {
	      int j;
	      rt_receivex(task, &j, sizeof(int), &len);
				if(j < 0) break;
	      char traceName[10];
				sprintf(traceName, "Trace %d", j+1);
	      rt_returnx(task, traceName, strlen(traceName)+1); // return null terminated string	    
	  }
	  samplingTime = ssGetSampleTime(rtaiScope[scopeIdx],0);
	  rt_returnx(task, &samplingTime, sizeof(float));
	}
      }
      }
      { int logIdx, Reply;
      float samplingTime;
      int nrow, ncol, *dim;
      while (1) {
	rt_receivex(task, &logIdx, sizeof(int), &len);
	if (logIdx < 0) {
	  Reply = logIdx;
	  rt_returnx(task, &Reply, sizeof(int));
	  break;
	} else {
	  dim = ssGetInputPortDimensions(rtaiLog[logIdx],0);
	  nrow = dim[0];
	  ncol = dim[1];
	  rt_returnx(task, &nrow, sizeof(int));
	  rt_receivex(task, &logIdx, sizeof(int), &len);
	  rt_returnx(task, &ncol, sizeof(int));
	  rt_receivex(task, &logIdx, sizeof(int), &len);
	  rt_returnx(task, (char *)ssGetModelName(rtaiLog[logIdx]), MAX_NAMES_SIZE*sizeof(char));
	  rt_receivex(task, &logIdx, sizeof(int), &len);
	  samplingTime = ssGetSampleTime(rtaiLog[logIdx],0);
	  rt_returnx(task, &samplingTime, sizeof(float));
	}
      }
      }
      { int alogIdx, Reply;  /* section added to support automatic log block  taken from log code */
      float samplingTime;
      int nrow, ncol, *dim;
      while (1) {
	rt_receivex(task, &alogIdx, sizeof(int), &len);
	if (alogIdx < 0) {
	  Reply = alogIdx;
	  rt_returnx(task, &Reply, sizeof(int));
	  break;
	} else {
	  dim = ssGetInputPortDimensions(rtaiALog[alogIdx],0);
	  nrow = dim[0];
	  ncol = dim[1];
	  rt_returnx(task, &nrow, sizeof(int));
	  rt_receivex(task, &alogIdx, sizeof(int), &len);
	  rt_returnx(task, &ncol, sizeof(int));
	  rt_receivex(task, &alogIdx, sizeof(int), &len);
	  rt_returnx(task, (char *)ssGetModelName(rtaiALog[alogIdx]), MAX_NAMES_SIZE*sizeof(char));
	  rt_receivex(task, &alogIdx, sizeof(int), &len);
	  samplingTime = ssGetSampleTime(rtaiALog[alogIdx],0);
	  rt_returnx(task, &samplingTime, sizeof(float));
	}
      }
      }
      { int ledIdx, Reply;
      float samplingTime;
      int n_leds;
      while (1) {
	rt_receivex(task, &ledIdx, sizeof(int), &len);
	if (ledIdx < 0) {
	  Reply = ledIdx;
	  rt_returnx(task, &Reply, sizeof(int));
	  break;
	} else {
	  n_leds = ssGetNumInputPorts(rtaiLed[ledIdx]);
	  rt_returnx(task, &n_leds, sizeof(int));
	  rt_receivex(task, &ledIdx, sizeof(int), &len);
	  rt_returnx(task, (char *)ssGetModelName(rtaiLed[ledIdx]), MAX_NAMES_SIZE*sizeof(char));
	  rt_receivex(task, &ledIdx, sizeof(int), &len);
	  samplingTime = ssGetSampleTime(rtaiLed[ledIdx],0);
	  rt_returnx(task, &samplingTime, sizeof(float));
	}
      }
      }
      { int meterIdx, Reply;
      float samplingTime;
      while (1) {
	rt_receivex(task, &meterIdx, sizeof(int), &len);
	if (meterIdx < 0) {
	  Reply = meterIdx;
	  rt_returnx(task, &Reply, sizeof(int));
	  break;
	} else {
	  rt_returnx(task, (char *)ssGetModelName(rtaiMeter[meterIdx]), MAX_NAMES_SIZE*sizeof(char));
	  rt_receivex(task, &meterIdx, sizeof(int), &len);
	  samplingTime = ssGetSampleTime(rtaiMeter[meterIdx],0);
	  rt_returnx(task, &samplingTime, sizeof(float));
	}
      }
      }
      { int synchIdx, Reply;
      float samplingTime;
      while (1) {
	rt_receivex(task, &synchIdx, sizeof(int), &len);
	if (synchIdx < 0) {
	  Reply = synchIdx;
	  rt_returnx(task, &Reply, sizeof(int));
	  break;
	} else {
	  rt_returnx(task, (char *)ssGetModelName(rtaiSynchronoscope[synchIdx]), MAX_NAMES_SIZE*sizeof(char));
	  rt_receivex(task, &synchIdx, sizeof(int), &len);
	  samplingTime = ssGetSampleTime(rtaiSynchronoscope[synchIdx],0);
	  rt_returnx(task, &samplingTime, sizeof(float));
	}
      }
      }
      break;
    }

    case 's': { int Reply = 1;

    rt_task_resume(rt_MainTask);
    rt_return(task, Reply);
    break;
    }

    case 't': { int Reply = 0;

    rt_return(task, Reply);
    endex = 1;
    break;
    }

    case 'p': { ModelMappingInfo *MMI;
    int index;
    int Reply;
    int matIdx;
    double newv;

    rt_return(task, IsRunning);
    Reply = 0;
    rt_receivex(task, &index, sizeof(int), &len);
    rt_returnx(task, &Reply, sizeof(int));
    Reply = 1;
    MMI = (ModelMappingInfo *)rtmGetModelMappingInfo(rtM);
    rt_receivex(task, &newv, sizeof(double), &len);
    rt_returnx(task, &Reply, sizeof(int));
    rt_receivex(task, &matIdx, sizeof(int), &len);
    rt_ModifyParameterValue(MMI, index, matIdx, &newv);
    rt_returnx(task, &Reply, sizeof(int));
    break;
    }

    case 'g': { uint_T i;
    uint_T nBlockParams;
    rtTargetParamInfo rtParameters;
    ModelMappingInfo *MMI;

    rt_return(task, IsRunning);
    MMI = (ModelMappingInfo *)rtmGetModelMappingInfo(rtM);
    nBlockParams = mmiGetNumBlockParams(MMI);
    for (i = 0; i < nBlockParams; i++) {
      rt_receivex(task, &rtParameters, sizeof(char), &len);
      rt_GetParameterInfo(MMI, &rtParameters, i);
      rt_returnx(task, &rtParameters, sizeof(rtParameters));
    }
    break;
    }

    case 'd': { int ParamCnt;
    int Reply;

    rt_return(task, IsRunning);
    Reply = 0;
    rt_receivex(task, &ParamCnt, sizeof(int), &len);
    rt_returnx(task, &Reply, sizeof(int));

    { int i;
    ModelMappingInfo *MMI;
    struct {
      int index;
      int mat_index;
      double value;
    } BatchParams[ParamCnt];

    Reply = 1;
    MMI = (ModelMappingInfo *)rtmGetModelMappingInfo(rtM);
    rt_receivex(task, &BatchParams, sizeof(BatchParams), &len);
    for (i = 0; i < ParamCnt; i++) {
      rt_ModifyParameterValue(MMI, BatchParams[i].index, BatchParams[i].mat_index, &BatchParams[i].value);
    }
    }
    rt_returnx(task, &ParamCnt, sizeof(int));
    break;
    }

    case 'm': { float ttime = (float)rtmGetT(rtM);
    int Reply;

    rt_return(task, IsRunning);
    rt_receivex(task, &Reply, sizeof(int), &len);
    rt_returnx(task, &ttime, sizeof(float));
    }

    default:
      break;
    }

  }

  rt_task_delete(rt_HostInterfaceTask);

  return 0;
}

static int_T rt_Main(RT_MODEL * (*model_name)(void), int_T priority)
{
  const char *status;
  RTIME rt_BaseTaskPeriod;
  struct timespec rt_MainTaskPollPeriod = { 0, POLL_PERIOD };
  struct timespec err_timeout;
  int msg, i;
  char myname[7];
  SEM *hard_timers_cnt = NULL;
#ifdef MULTITASKING
  int sample, *taskarg;
#endif

  rt_allow_nonroot_hrt();
  for (i = 0; i < MAX_NTARGETS; i++) {
    sprintf(myname, "TMN%d", i);
    if (!rt_get_adr(nam2num(myname))) break;
  }
  if (!(rt_MainTask = rt_task_init_schmod(nam2num(myname), rt_MainTaskPriority, 0, 0, SCHED_FIFO, 0xFF))) {
    fprintf(stderr, "Cannot init rt_MainTask\n");
    return 1;
  }
  sem_init(&err_sem, 0, 0);
  iopl(3);

  rt_InitInfAndNaN(sizeof(real_T));
  rtM = model_name();
  if (rtM == NULL) {
    fprintf(stderr, "Memory allocation error during target registration.\n");
    goto finish;
  }
  if (rtmGetErrorStatus(rtM) != NULL) {
    fprintf(stderr, "Error during target registration: %s.\n", rtmGetErrorStatus(rtM));
    goto finish;
  }

  if (FinalTime > 0.0 || FinalTime == RUN_FOREVER) {
    rtmSetTFinal(rtM, (real_T)FinalTime);
  }

  INITIALIZE_SIZES(rtM);
  INITIALIZE_SAMPLE_TIMES(rtM);

  status = rt_SimInitTimingEngine(rtmGetNumSampleTimes(rtM), rtmGetStepSize(rtM), rtmGetSampleTimePtr(rtM), rtmGetOffsetTimePtr(rtM), rtmGetSampleHitPtr(rtM), rtmGetSampleTimeTaskIDPtr(rtM), rtmGetTStart(rtM), &rtmGetSimTimeStep(rtM), &rtmGetTimingData(rtM));
  if (status != NULL) {
    fprintf(stderr, "Failed to initialize target sample time engine: %s.\n", status);
    goto finish;
  }

  rt_ODECreateIntegrationData(rtmGetRTWSolverInfo(rtM));

  START(rtM);
  if (rtmGetErrorStatus(rtM) != NULL) {
    fprintf(stderr, "Failed in target initialization.\n");
    TERMINATE(rtM);
    fprintf(stderr, "Target is terminated.\n");
    goto finish;
  }

  if (EnableHostInterface) {
	  if ((pthread_create(&rt_HostInterfaceThread, NULL, rt_HostInterface, NULL)) != 0) {
		fprintf(stderr, "Failed to create HostInterfaceThread.\n");
		TERMINATE(rtM);
		fprintf(stderr, "Target is terminated.\n");
		goto finish;
	  }
	  err_timeout.tv_sec = (long int)(time(NULL)) + 1;
	  err_timeout.tv_nsec = 0;
	  if ((sem_timedwait(&err_sem, &err_timeout)) != 0) {
		TERMINATE(rtM);
		fprintf(stderr, "Target is terminated.\n");
		goto finish;
	  }
  }

#ifdef MULTITASKING
  rt_SubRateTasks   = malloc(NUMST*sizeof(RT_TASK *));
  rt_SubRateThreads = malloc(NUMST*sizeof(pthread_t));
  taskarg           = malloc(2*NUMST*sizeof(int));
  for (sample = FIRST_TID + 1; sample < NUMST; sample++) {
    taskarg[0] = sample;
    taskarg[1] = priority + sample;
    pthread_create(rt_SubRateThreads + sample, NULL, (void *)rt_SubRate, (void *)taskarg);
    taskarg += 2;
  }
#endif
  if ((pthread_create(&rt_BaseRateThread, NULL, rt_BaseRate, &priority)) != 0) {
    fprintf(stderr, "Failed to create BaseRateThread.\n");
    endInterface = 1;
	if (EnableHostInterface) {
		rt_send(rt_HostInterfaceTask, 0);
		pthread_join(rt_HostInterfaceThread, NULL);
	}
    TERMINATE(rtM);
    fprintf(stderr, "Target is terminated.\n");
    goto finish;
  }

  err_timeout.tv_sec = (long int)(time(NULL)) + 1;
  err_timeout.tv_nsec = 0;
  if ((sem_timedwait(&err_sem, &err_timeout)) != 0) {
    endInterface = 1;
	if (EnableHostInterface) {
		rt_send(rt_HostInterfaceTask, 0);
		pthread_join(rt_HostInterfaceThread, NULL);
	}
    TERMINATE(rtM);
    fprintf(stderr, "Target is terminated.\n");
    goto finish;
  }

  if (ros::master::check()) {
	  if ((pthread_create(&rosThread, NULL, rosInterface, NULL)) != 0) {
		fprintf(stderr, "Failed to create ROS thread.\n");
	  }
  } else {
	  ROS_ERROR("Failed to contact master [%s]. ROS thread disabled!", ros::master::getURI().c_str());
  }

  rt_BaseTaskPeriod = (RTIME)(1000000000.0*rtmGetStepSize(rtM));
  if (!ExternalTimer) {
//    WaitTimingEvent = (void *)rt_task_wait_period;
    if (!(hard_timers_cnt = (SEM *)rt_get_adr(nam2num("HTMRCN")))) {
      if (OneShot) {
	rt_set_oneshot_mode();
	start_rt_timer(0);
	rt_BaseRateTick = nano2count(rt_BaseTaskPeriod);
      } else {
	rt_set_periodic_mode();
	rt_BaseRateTick = start_rt_timer(nano2count(rt_BaseTaskPeriod));
      }
      hard_timers_cnt = rt_sem_init(nam2num("HTMRCN"), 0);
    } else {
      rt_BaseRateTick = nano2count(rt_BaseTaskPeriod);
      rt_sem_signal(hard_timers_cnt);
    }
  } else {
//    WaitTimingEvent = (void *)DummyWait;
//    SendTimingEvent = (void *)DummySend;
  }

  if (Verbose) {
    int j;
    printf("\nTarget info\n");
    printf("===========\n");
    printf("  Model name             : %s\n", STR(MODEL));
    printf("  Base sample time       : %f [s]\n", rtmGetStepSize(rtM));
    printf("  Number of sample times : %d\n", rtmGetNumSampleTimes(rtM));
    for (j = 0; j < rtmGetNumSampleTimes(rtM); j++) {
      printf("  Sample Time %d          : %f [s]\n", j, rtmGetSampleTimePtr(rtM)[j]);
    }
    printf("\n");
  }

  if (WaitToStart) {
    if (Verbose) {
      printf("Target is waiting to start.\n");
    }
    rt_task_suspend(rt_MainTask);
  }
  rt_receive(0, &msg);
  rt_return(rt_BaseRateTask,0);
  IsRunning = 1;
  if (Verbose) {
    printf("Target is running.\n");
  }

  while (!endex) {
    if (rtmGetTFinal(rtM) != RUN_FOREVER && (rtmGetTFinal(rtM) - rtmGetT(rtM)) <= rtmGetT(rtM)*DBL_EPSILON) {
      if (Verbose) {
	printf("Final time occured.\n");
      }
      break;
    }
    if (rtmGetErrorStatus(rtM) != NULL) {
      fprintf(stderr, "%s.\n", rtmGetErrorStatus(rtM));
      break;
    }

    nanosleep(&rt_MainTaskPollPeriod, NULL);
  }

  // Shutdown ROS thread
  endRos = 1;
  pthread_join(rosThread, NULL);

  endBaseRate = 1;
  if (!!ExternalTimer) {
//    SendTimingEvent(TimingEventArg);
  }
  pthread_join(rt_BaseRateThread, NULL);

#ifdef MULTITASKING
  endSubRate = 1;
  for (sample = FIRST_TID + 1; sample < NUMST; sample++) {
    if (rt_SubRateTasks[sample]) {
      rt_task_resume(rt_SubRateTasks[sample]);
    }
    pthread_join(rt_SubRateThreads[sample], NULL);
  }
#endif

  IsRunning = 0;
  if (Verbose) {
    printf("Target is stopped.\n");
  }

  endInterface = 1;
  if (EnableHostInterface) {
    rt_send(rt_HostInterfaceTask, 0);
    pthread_join(rt_HostInterfaceThread, NULL);
  }

  if (!ExternalTimer) {
    if (!rt_sem_wait_if(hard_timers_cnt)) {
      rt_sem_delete(hard_timers_cnt);
    }
  }

#ifdef MULTITASKING
  free(rt_SubRateTasks);
  free(rt_SubRateThreads);
#endif


  TERMINATE(rtM);
  printf("Target is terminated.\n");

 finish:
  sem_destroy(&err_sem);
  rt_task_delete(rt_MainTask);

  return 0;
}

static void endme(int dummy)
{
  signal(SIGINT, endme);
  signal(SIGTERM, endme);
  endex = 1;
}

enum {
    OPT_VERSION = 1000,
    OPT_RTAILAB,
    OPT_FINALTIME,
    OPT_IFTASK,
    OPT_SCOPEID,
    OPT_LOGID,
    OPT_ALOGID,
    OPT_METERID,
    OPT_LEDID,
    OPT_SYNCHID,
    OPT_RANDOM,
    OPT_NOROSOUT
};

static struct poptOption long_options[] = {
	/* longName, shortName, argInfo, argPtr, value, descrip, argDesc */
	{ "verbose", 'v', POPT_ARG_NONE, (int *)&Verbose, 'v', "Verbose output", 0 },
	{ "version", 0, POPT_ARG_NONE, 0, OPT_VERSION, "Print version information", 0 },
	{ "rtailab", 0, POPT_ARG_NONE, &EnableHostInterface, OPT_RTAILAB, "start the legacy RTAI-Lab host interface task", 0 },
	{ "wait", 'w', POPT_ARG_NONE, (int *)&WaitToStart, 'w', "Wait to start", 0 },
	{ "soft", 's', POPT_ARG_NONE, (int *)&UseSoftRT, 's', "Run RT-model in soft instead of hard real-time", 0 },
	{ "priority", 'p', POPT_ARG_INT, (int *)&Priority, 'p', "Set the priority for the real-time task", "0" },
	{ "finaltime", 0, POPT_ARG_FLOAT, &FinalTime, OPT_FINALTIME, "Set the final time", "inf" },
	{ "ifname", 0, POPT_ARG_STRING, &HostInterfaceTaskName, OPT_IFTASK, "Set the host interface task identifier", HostInterfaceTaskName },
	{ "scopeid", 0, POPT_ARG_STRING, &TargetScopeMbxID, OPT_SCOPEID, "Set the scope mailboxes identifier", TargetScopeMbxID },
	{ "logid", 0, POPT_ARG_STRING, &TargetLogMbxID, OPT_LOGID, "Set the log mailboxes identifier", TargetLogMbxID },
	{ "alogid", 0, POPT_ARG_STRING, &TargetALogMbxID, OPT_ALOGID, "Set the automatic log mailboxes identifier", TargetALogMbxID },
	{ "meterid", 0, POPT_ARG_STRING, &TargetMeterMbxID, OPT_METERID, "Set the meter mailboxes identifier", TargetMeterMbxID },
	{ "ledid", 0, POPT_ARG_STRING, &TargetLedMbxID, OPT_LEDID, "Set the led mailboxes identifier", TargetLedMbxID },
	{ "synchid", 0, POPT_ARG_STRING, &TargetSynchronoscopeMbxID, OPT_SYNCHID, "Set the synchronoscope mailboxes identifier", TargetSynchronoscopeMbxID },
	{ "cpumap", 'c', POPT_ARG_INT, (int *)&CpuMap, 'c', "(1 << cpunum) on which the RT-model runs", STR(DEFAULT_CPUMAP) },
	{ "external", 'e', POPT_ARG_NONE, (int *)&ExternalTimer, 'e', "RT-model timed by an external resume", 0 },
	{ "oneshot", 'o', POPT_ARG_NONE, (int *)&OneShot, 'o', "the hard timer will run in oneshot mode", 0 },
	{ "stack", 'm', POPT_ARG_INT, (int *)&StackInc, 'm', "set a guaranteed stack size extension", STR(DEFAULT_STACKING) },
	{ "rosnode", 'N', POPT_ARG_STRING, &rosNode, 'N', "set the name of the ros node", STR(MODEL) },
//	{ "random", 0, POPT_ARG_NONE, &randomRosName, OPT_RANDOM, "adds a random number to the end of your node's name, to make it unique", 0 },
//	{ "norosout", 0, POPT_ARG_NONE, &norosout, OPT_NOROSOUT, "don't broadcast rosconsole output to the /rosout topic", 0 },
//	{ "namespace", 'n', POPT_ARG_STRING, &rosNamespace, 'n', "set a namespace", rosNamespace },
	POPT_AUTOHELP
	{ 0, 0, 0, 0, 0, 0 }
};

void print_usage(poptContext optCon, int exitcode, const char *error, const char *addl) {
    poptPrintUsage(optCon, stderr, 0);
    if (error) fprintf(stderr, "%s: %s\n", error, addl);
    poptFreeContext(optCon);
    exit(exitcode);
}

static void print_version() {
    fprintf(stderr, "%s %s\n", RTAIROS_NAME, RTAIROS_VERSION);
    exit(0);
}

static void print_rtailab() {
	printf("RTAI-Lab\n========\n");
	printf("  Host Interface ID : %s\n", HostInterfaceTaskName);
	printf("  Scope ID          : %s\n", TargetScopeMbxID);
	printf("  Log ID            : %s\n", TargetLogMbxID);
	printf("  Async. Scope ID   : %s\n", TargetALogMbxID);
	printf("  Meter ID          : %s\n", TargetMeterMbxID);
	printf("  Led ID            : %s\n", TargetLedMbxID);
	printf("  Sync. Scope ID    : %s\n", TargetSynchronoscopeMbxID);
	printf("\n");
}

int parse_arguments(int *argc_p, const char ***argv_p) {
    static poptContext optCon;
    const char **argv = *argv_p;
    int argc = *argc_p;
    int opt;

    optCon = poptGetContext(NULL, argc, argv, long_options, 0);

    while ((opt = poptGetNextOpt(optCon)) > 0) {
        switch (opt) {
            case OPT_VERSION:
                print_version();
                break;
        }
    }

	// Error processing
    if (opt < -1) {
        print_usage(optCon, 1, poptBadOption(optCon, opt), poptStrerror(opt));
    }
	return 0;
}

int main(int argc, char **argv) {
	//int orig_argc = argc;
	//char **orig_argv = argv;

	signal(SIGINT, endme);
	signal(SIGTERM, endme);

	parse_arguments(&argc, (const char ***) &argv);

	ros::init(argc, argv, std::string(rosNode), ros::init_options::NoSigintHandler);

	if (Verbose) {
		printf("\nTarget settings\n");
		printf("===============\n");
		printf("  Real-time : %s\n", UseSoftRT ? "SOFT" : "HARD");
		printf("  Timing    : %s / ", ExternalTimer ? "external" : "internal");
		printf("%s\n", OneShot ? "oneshot" : "periodic");
		printf("  Priority  : %d\n", Priority);
		if (FinalTime > 0) {
			printf("  Finaltime : %f [s]\n", FinalTime);
		} else {
			printf("  Finaltime : RUN FOREVER\n");
		}
		printf("  CPU map   : 0x%x\n", CpuMap);
		printf("\n");
	}
	if (Verbose && EnableHostInterface) {
		print_rtailab();
	}

	return rt_Main(MODEL, Priority);
}
