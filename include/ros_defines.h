#ifndef _ROS_DEFINES_H
#define _ROS_DEFINES_H

#include "simstruc.h"

#define MAX_ROS_BLOCKS 1000

#define ROS_SAMPLETIME 10
#define PUBLISHER_STACK_SIZE 10
#define SUBSCRIBER_STACK_SIZE 10
#define ROS_SEMAPHORE_TIMEOUT 0.5

/* Block types */
#define PUBLISHER    1
#define SUBSCRIBER   2
#define SERVICE      3
#define BROADCASTER  4
#define JOINTSTATE   5
#define LOGGER       6

/* Publisher messages */
#define PUBLISHER_FLOAT64         1       // std_msg/Float64
#define PUBLISHER_FLOAT64ARRAY    2       // std_msg/Float64MultiArray
#define PUBLISHER_BOOL            3       // std_msg/Bool
#define PUBLISHER_INT32           4       // std_msg/Int32
#define PUBLISHER_TIME            5       // std_msg/Time
#define PUBLISHER_POINT           6       // geometry_msgs/Point
#define PUBLISHER_POINT_STAMPED   7       // geometry_msgs/PointStamped
#define PUBLISHER_TWIST           8       // geometry_msgs/Twist 
#define PUBLISHER_TWIST_STAMPED   9       // geometry_msgs/TwistStamped
#define PUBLISHER_POSE2D         10       // geometry_msgs/Pose2D

/* Subscriber messages */
#define SUBSCRIBER_FLOAT64         1       // std_msg/Float64
#define SUBSCRIBER_FLOAT64ARRAY    2       // std_msg/Float64MultiArray
#define SUBSCRIBER_BOOL            3       // std_msg/Bool
#define SUBSCRIBER_INT32           4       // std_msg/Int32
#define SUBSCRIBER_TIME            5       // std_msg/Time
#define SUBSCRIBER_POINT           6       // geometry_msgs/Point
#define SUBSCRIBER_POINT_STAMPED   7       // geometry_msgs/PointStamped
#define SUBSCRIBER_TWIST           8       // geometry_msgs/Twist 
#define SUBSCRIBER_TWIST_STAMPED   9       // geometry_msgs/TwistStamped
#define SUBSCRIBER_POSE2D         10       // geometry_msgs/Pose2D
#define SUBSCRIBER_JOY            65       // sensor_msgs/Joy

#define STATE_DONE     0
#define STATE_RESPONSE 1
#define STATE_REQUEST  2
#define NEW_VALUE      1

#define MAX_NAMES_SIZE       256
#define MAX_SHMDATA_SIZE     100
#define MAX_LOG_MSG_SIZE     256
#define MAX_DESC_SIZE		256

#define LOG_DEBUG 1
#define LOG_INFO  2
#define LOG_WARN  3
#define LOG_ERROR 4
#define LOG_FATAL 5


typedef struct {
	char text[MAX_NAMES_SIZE];
	int level;
	int state;
} rosMsg_t;

typedef struct {
    double data[MAX_SHMDATA_SIZE];
    unsigned int length;
    int state;
    //header_t header;
	struct {
		double time;
		uint32_T seq;
		char refName[MAX_NAMES_SIZE];
	} header;
	rosMsg_t msg;
} rosShmData_t;

/* Configuration for the ROS task */
typedef struct rosConfig_t {
    double rate;                    // ROS rate (Hz)
    char ns[MAX_NAMES_SIZE];   // Namespace
    int exposeParams;               // Expose block params to parameter server 
    unsigned int pubStackSize;      // Publishers stack size
    unsigned int subStackSize;      // Subscribers stack size
} rosConfig_t;


typedef struct rosBlockConfig_t {
    char name[MAX_NAMES_SIZE];
    char refName[MAX_NAMES_SIZE];
    SimStruct *S;
    int type;
    int subType;
	char shmName[7];
	char semName[7];
} rosBlockConfig_t;

#endif
