#ifndef _ROS_INTERFACE_H
#define _ROS_INTERFACE_H

extern "C" {
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/io.h>
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

#define XSTR(x)    #x
#define STR(x)     XSTR(x)
#define EXPAND_CONCAT(name1,name2)	name1 ## name2
#define CONCAT(name1,name2)		EXPAND_CONCAT(name1,name2)
#define RT_MODEL			CONCAT(MODEL,_rtModel)
#define MAX_DATA_SIZE		512
#define rt_RosInterfaceTaskPriority	    95

struct RosObject {

	std::string name;			// ROS name
	std::string refName;		// ROS frame_id
	char shmName[7];			// Shared memory ID
	char semName[7];			// Semaphore ID
	unsigned int num;			// Position in rosBlockConfig[]
	rosShmData_t *shm;
	SEM *sem;

	RosObject(rosBlockConfig_t conf, unsigned int num);
	~RosObject();
	bool sem_wait();
	void sem_signal();
	void logNew();
	void logNew(rosMsg_t msg);

};

struct RosPublisher : RosObject {
	int subType;
	ros::Publisher pub;

	RosPublisher(rosBlockConfig_t conf, unsigned int num) : RosObject(conf, num) {
		this->subType = conf.subType;
	};

	void publish();
};

struct RosSubscriber : RosObject {
	int subType;
	ros::Subscriber sub;

	RosSubscriber(rosBlockConfig_t conf, unsigned int num) : RosObject(conf, num) {
		this->subType = conf.subType;
	};

	void callback(const std_msgs::Float64MultiArray msg);
	void callback(const sensor_msgs::Joy msg);
	void callback(const std_msgs::Float64 msg);
	void callback(const std_msgs::Int32 msg);
	void callback(const std_msgs::Bool msg);
	void callback(const std_msgs::Time msg);
	void callback(const geometry_msgs::Twist msg);
	void callback(const geometry_msgs::TwistStamped msg);
	void callback(const geometry_msgs::Point msg);
	void callback(const geometry_msgs::PointStamped msg);
	void callback(const geometry_msgs::Pose2D msg);
};

struct RosBroadcaster : RosObject {
	tf::TransformBroadcaster br;

	RosBroadcaster(rosBlockConfig_t conf, unsigned int num) : RosObject(conf, num) { /* empty */ };
	void send();
};

struct RosService : RosObject {
	ros::ServiceServer srv;

	RosService(rosBlockConfig_t conf, unsigned int num) : RosObject(conf, num) { /* empty */ };
	bool callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
};

struct RosJointState : RosObject {

	RosJointState(rosBlockConfig_t conf, unsigned int num) : RosObject(conf, num) { /* empty */ };
	void fill(sensor_msgs::JointState *joint_state, unsigned int num);
};

class RosInterface {

public:

	void* init(void);
	bool start(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	void publishParams(void);
	void cleanParams(void);
	bool refreshParams(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setParams(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	void printInitMessage(int pub, int sub, int srv, int br, int jnt);

	static inline void *thread(void *context) {
		return ((RosInterface *)context)->init();
	}

};

#endif
