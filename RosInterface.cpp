#include "RosInterface.h"

extern RT_TASK *rt_MainTask;
static RT_TASK *rt_rosTask;
extern rosConfig_t rosConfig;
extern const char *rosNode;
extern RT_MODEL *rtM;
extern volatile int Verbose;
extern unsigned int numRosBlocks;
extern volatile int WaitToStart;
extern const char *RosTaskID;
extern volatile bool endRos;
extern rosBlockConfig_t rosBlockConfigs[MAX_ROS_BLOCKS];

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


extern int_T rt_GetParameterInfo(void *mpi, rtTargetParamInfo *rtpi, int i);
extern int_T rt_ModifyParameterValue(void *mpi, int i, int matIdx, void *_newVal);

static char_T* ssGetBlockName(SimStruct *S) {
	char *path = (char *)ssGetPath(S);
	char *slash = strrchr(path, '/');
	if (slash != NULL) {
			return slash + 1;
	}
	return path;
}

static std::string sanitizeName(std::string str) {
	str = boost::regex_replace(str, boost::regex(":"), "/");
	str = boost::regex_replace(str, boost::regex("^[^a-zA-Z/]"), "X");
	str = boost::regex_replace(str, boost::regex("[^a-zA-Z0-9/_]"), "_");
	return str;
}

static void secureRosConfig() {
	std::string ns(rosConfig.ns);
	ns = sanitizeName(ns);
	memcpy(rosConfig.ns, ns.c_str(), MAX_NAMES_SIZE);
	if (rosConfig.rate <= 0) {
		ROS_WARN("Illegal ROS rate %f found. Set to default %f", rosConfig.rate, (double)ROS_SAMPLETIME);
		rosConfig.rate = ROS_SAMPLETIME;
	}
}


RosObject::RosObject(rosBlockConfig_t conf, unsigned int num) {
	std::string name(conf.name);
	if (name.length() == 0) {
		char *blockName = ssGetBlockName(conf.S);
		ROS_INFO("Name/Topic of block '%s' not set, block name will be used", blockName);
		name = std::string(blockName);
	}
	std::string refName(conf.refName);

	this->name = sanitizeName(name);
	if (this->name != name) {
		ROS_WARN("Invalid name '%s' of block '%s' replaced by '%s'", conf.name, ssGetBlockName(conf.S), this->name.c_str());
	}
	this->refName = sanitizeName(refName);
	this->num = num;

	// Init shared memory
	memcpy(this->shmName, conf.shmName, 7);
	this->shm = (rosShmData_t*)rt_shm_alloc(nam2num(this->shmName), sizeof(rosShmData_t), USE_VMALLOC);
	memcpy(this->semName, conf.semName, 7);
	this->sem = (SEM*)rt_get_adr(nam2num(this->semName));
}

RosObject::~RosObject() {
	rt_shm_free(nam2num(this->shmName));
}

bool RosObject::sem_wait() {
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

void RosObject::sem_signal() {
	rt_sem_signal(this->sem);
}

void RosObject::logNew() {
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

void RosObject::logNew(rosMsg_t msg) {
	if (msg.state > 0) {
		ROS_LOG((ros::console::Level)msg.level, ROSCONSOLE_DEFAULT_NAME, "%s", msg.text);
	}
}


void RosPublisher::publish() {
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


void RosSubscriber::callback(const std_msgs::Float64MultiArray msg) {
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

void RosSubscriber::callback(const sensor_msgs::Joy msg) {
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

void RosSubscriber::callback(const std_msgs::Float64 msg) {
	if (!this->sem_wait()) return;
	shm->data[0] = msg.data;
	this->sem_signal();
}

void RosSubscriber::callback(const std_msgs::Int32 msg) {
	if (!this->sem_wait()) return;
	shm->data[0] = msg.data;
	this->sem_signal();
}

void RosSubscriber::callback(const std_msgs::Bool msg) {
	if (!this->sem_wait()) return;
	shm->data[0] = msg.data;
	this->sem_signal();
}

void RosSubscriber::callback(const std_msgs::Time msg) {
	double time = msg.data.toSec();
	if (!this->sem_wait()) return;
	shm->data[0] = time;
	this->sem_signal();
}

void RosSubscriber::callback(const geometry_msgs::Twist msg) {
	if (!this->sem_wait()) return;
	shm->data[0] = msg.linear.x;
	shm->data[1] = msg.linear.y;
	shm->data[2] = msg.linear.z;
	shm->data[3] = msg.angular.x;
	shm->data[4] = msg.angular.y;
	shm->data[5] = msg.angular.z;
	this->sem_signal();
}

void RosSubscriber::callback(const geometry_msgs::TwistStamped msg) {
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

void RosSubscriber::callback(const geometry_msgs::Point msg) {
	if (!this->sem_wait()) return;
	shm->data[0] = msg.x;
	shm->data[1] = msg.y;
	shm->data[2] = msg.z;
	this->sem_signal();
}

void RosSubscriber::callback(const geometry_msgs::PointStamped msg) {
	if (!this->sem_wait()) return;
	shm->data[0] = msg.point.x;
	shm->data[1] = msg.point.y;
	shm->data[2] = msg.point.z;
	shm->header.time = msg.header.stamp.toSec();
	shm->header.seq = msg.header.seq;
	this->sem_signal();
}

void RosSubscriber::callback(const geometry_msgs::Pose2D msg) {
	if (!this->sem_wait()) return;
	shm->data[0] = msg.x;
	shm->data[1] = msg.y;
	shm->data[2] = msg.theta;
	this->sem_signal();
}


void RosBroadcaster::send() {
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


bool RosService::callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	ros::Rate loop_rate(rosConfig.rate);
	if (!this->sem_wait()) return false;
	this->shm->state = STATE_REQUEST;
	this->sem_signal();
	ROS_INFO("Service '%s' called", name.c_str());
	return true;
}


void RosJointState::fill(sensor_msgs::JointState *joint_state, unsigned int num) {
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


void RosInterface::cleanParams() {
	ros::NodeHandle nh;
	nh.deleteParam(std::string(rosNode));
}

void RosInterface::publishParams() {
	ros::NodeHandle nh;
	uint_T nBlockParams;
	rtTargetParamInfo rtParameters;
	ModelMappingInfo *MMI;

	// Cleanup first
	this->cleanParams();

	MMI = (ModelMappingInfo *)rtmGetModelMappingInfo(rtM);
	nBlockParams = mmiGetNumBlockParams(MMI);
	for (uint_T i = 0; i < nBlockParams; i++) {
		rt_GetParameterInfo(MMI, &rtParameters, i);
		std::stringstream ss;
		ss << rosNode << &rtParameters.blockName[strlen(STR(MODEL))] << '/' << rtParameters.paramName;

		std::string str = sanitizeName(ss.str());

		uint_T len = rtParameters.nCols*rtParameters.nRows;
		if (len > MAX_DATA_SIZE) {
			ROS_ERROR("publishParams: MAX_DATA_SIZE exceeded: %s, %i", str.c_str(), len);
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

bool RosInterface::setParams(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
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
		std::string str = sanitizeName(ss.str());

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
	this->publishParams();
	return true;
}

bool RosInterface::start(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	ROS_DEBUG("Service start called");
	rt_task_resume(rt_MainTask);
	return true;
}

bool RosInterface::refreshParams(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	this->publishParams();
	return true;
}

void RosInterface::printInitMessage(int pub, int sub, int srv, int br, int jnt) {
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
void *RosInterface::init(void) {
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
		startSrv = nh.advertiseService(srvName, &RosInterface::start, this);
	}

	// Parameters
	ros::ServiceServer srvSetParam;
	ros::ServiceServer srvRefreshParam;
	if (rosConfig.exposeParams > 0) {
		this->publishParams();
		snprintf(srvName, 100, "/%s/refresh_parameters", rosNode);
		srvRefreshParam = nh.advertiseService(srvName, &RosInterface::refreshParams, this);
		if (rosConfig.exposeParams > 1) {
			snprintf(srvName, 100, "/%s/set_parameters", rosNode);
			srvSetParam = nh.advertiseService(srvName, &RosInterface::setParams, this);
		}
	}

	if (jointstates.size() > 0) {
		joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", rosConfig.pubStackSize);
	}

	this->printInitMessage(publishers.size(), subscribers.size(), services.size(), broadcasters.size(), jointstates.size());


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
		this->cleanParams();
	}

	ros::shutdown();

	printf("ROS Task terminated\n");

	return 0;
}


