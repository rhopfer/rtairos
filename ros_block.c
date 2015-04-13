#include <ros_block.h>

rosBlockInitResult_t registerRosBlock(SimStruct *S, char *rosName, int type, int subType) {
    char name[7];
	int_T i;
    int num = numRosBlocks;
	rosBlockInitResult_t res;

    rosBlockConfigs[num].S = S;
    memcpy(rosBlockConfigs[num].name, rosName, MAX_NAMES_SIZE);
    rosBlockConfigs[num].type = type;
    rosBlockConfigs[num].subType = subType;

	for (i = 0; i < MAX_ROS_BLOCKS; i++) {
		sprintf(name, "%s%d", RosShmID, i);
		if (!rt_get_adr(nam2num(name))) break;
	}
    if (!(res.shm = (rosShmData_t *)rt_shm_alloc(nam2num(name), sizeof(rosShmData_t), USE_VMALLOC))) {
		printf("Cannot init shared memory %s: Time to die!\n", name);
		exit(1);
	}
	memcpy(rosBlockConfigs[num].shmName, name, 7);
	for (i = 0; i < MAX_ROS_BLOCKS; i++) {
		sprintf(name, "%s%d", RosSemID, i);
		if (!rt_get_adr(nam2num(name))) break;
	}
    if (!(res.sem = rt_typed_sem_init(nam2num(name), 1, BIN_SEM | PRIO_Q))) {
		printf("Cannot init semaphore %s: Time to kill myself!\n", name);
		exit(1);
	}
	memcpy(rosBlockConfigs[num].semName, name, 7);
	res.shm->length = 0;
	res.shm->state = 0;
	res.shm->msg.state = 0;

	res.num = num;
    numRosBlocks++;
    return res;
}

void cleanRosBlock(unsigned int num) {
    SEM *sem;

    rt_shm_free(nam2num(rosBlockConfigs[num].shmName));
    sem = (SEM*)rt_get_adr(nam2num(rosBlockConfigs[num].semName));

    rt_sem_broadcast(sem);
    rt_sem_delete(sem);
}

char_T* ssGetBlockName(SimStruct *S) {
	char *path = (char *)ssGetPath(S);
	char *slash = strrchr(path, '/');
	if (slash != NULL) {
			return slash + 1;
	}
	return path;
}


