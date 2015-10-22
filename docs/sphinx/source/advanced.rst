Advanced usage
==============

Extend own S-Function 
---------------------

The following code shows the base structure to add log functionality to your own S-Function::

    #ifndef MATLAB_MEX_FILE
    #include <ros_block.h>
    #endif

    static void mdlInitializeSizes(SimStruct *S) {
        ssSetNumIWork(S, 1);
        ssSetNumPWork(S, 2);
    }
    
    #define MDL_START
    static void mdlStart(SimStruct *S) {
        #ifndef MATLAB_MEX_FILE
        rosBlockInitResult_t block = registerRosBlock(S, "rosout", LOGGER, 0);
        block.shm->msg.level = LOG_ERROR;
        ssSetIWorkValue(S, 0, block.num);
        ssSetPWorkValue(S, 0, (void *)block.shm);
        ssSetPWorkValue(S, 1, (void *)block.sem);
        #endif
    }
    
    static void mdlOutputs(SimStruct *S, int_T tid) {
        #ifndef MATLAB_MEX_FILE
        rosShmData_t *shm = (rosShmData_t *)ssGetPWorkValue(S, 0);
        SEM *sem = (SEM *)ssGetPWorkValue(S, 1);
        if (error) {
            if (rt_sem_wait_if(sem) != 0) {
                memcpy(shm->msg.text, "Error occurred", MAX_LOG_MSG_SIZE);
                shm->msg.state = NEW_VALUE;
                rt_sem_signal(sem);
            }
        }
        #endif
    }
    
    static void mdlTerminate(SimStruct *S) {
        #ifndef MATLAB_MEX_FILE
        cleanRosBlock(ssGetIWorkValue(S, 0));
        #endif
    }

Add own message types
---------------------

For the moment this requires to change the source code.
The necessary steps are shown using ``geometry_msgs/Vector3`` as an example.

1. In ``include/ros_defines.h`` add a new preprocessor variable, e.g.::

     #define PUBLISHER_VECTOR3 42    // or
     #define SUBSCRIBER_VECTOR3 42

2. Include the header file in ``rtmain.cpp``::

     #undef RT
     #include <geometry_msgs/Vector3.h>
     #define RT

3. For a publisher add a new case to class ``RosPublisher``::

     void publish() {
         ...
         } else if (subType == PUBLISHER_VECTOR3) {
             geometry_msgs::Vector3 msg;
             msg.x = shmData.data[0];
             msg.y = shmData.data[1];
             msg.z = shmData.data[2];
             pub.publish(msg);
         }
     }

   For a subscriber add a new callback function to ``RosSubscriber``::

     void callback(const geometry_msgs::Vector3 msg) {
         if (!this->sem_wait()) return;
         shm->data[0] = msg.x;
         shm->data[1] = msg.y;
         shm->data[2] = msg.z;
         this->sem_signal();
     }

4. In function ``rosInterface()`` add an initialization block::

     // Subscribers
     } else if (rosBlockConfigs[i].type == SUBSCRIBER) {
         ...
         } else if (subscriber->subType == SUBSCRIBER_VECTOR3) {
             subscriber->sub = nh.subscribe<geometry_msgs::Vector3>(subscriber->name,
                 rosConfig.subStackSize, &RosSubscriber::callback, subscriber);

     // Publishers
     } else if (rosBlockConfigs[i].type == PUBLISHER) {
         ...
         } else if (publisher->subType == PUBLISHER_VECTOR3) {
             publisher->pub = nh.advertise<geometry_msgs::Vector3>(publisher->name,
                 rosConfig.pubStackSize);


