/*============================================================================
==============================================================================

                              hoapSendTask.cpp

==============================================================================
Remarks:

      C++ task for sending data to hoap robot

============================================================================*/

// SL system headers
#include "SL_system_headers.h"

// SL includes
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"

// local includes
#include "hoapSendTask.h"

#include <arpa/inet.h>
#include <netinet/in.h>
//#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#define SRV_IP "178.172.42.110"
#define SND_BUFLEN 324
#define RCV_BUFLEN 440
#define HOAP_PORT 15005
#define MY_PORT 15006
static int mainSend(char *buf);
static int mainRead(char *buf);

static int hoap_read();

static int hoap_connected=FALSE;

void* hoapListeningThread(void * arg){
	int success=0;
	printf("Ready to connect\n");
	while(1){
		sleep(0.001);
		if (hoap_connected){
			success=hoap_read();
			if (success){
				//printf("read something\n");
			}
		}
	}
	return 0;
}

static int hoap_read()
{
	int success;

	char buf_read[RCV_BUFLEN];

	success=mainRead(buf_read);

	//indices start with 0; these variables should be global, or be written into globals at the end of this function
	float motor_position[28];
	short fsr[8];
	short acc_gyro[6];
	float motor_velocity[28];
	int time_counter;
	float time_in_seconds;
	float endeffector_positions[42];
	short gripper_force[2];
	short distance_sensor;
	short battery_voltage;
	char robot_status;

	int buf_count=0;
	for (int i=0; i<28; ++i){
		memcpy(&motor_position[i], &buf_read[buf_count], 4);
		buf_count += 4;
	}
	for (int i=0; i<8; ++i){
		memcpy(&fsr[i], &buf_read[buf_count], 2);
		buf_count += 2;
	}
	for (int i=0; i<6; ++i){
		memcpy(&acc_gyro[i], &buf_read[buf_count], 2);
		buf_count += 2;
	}
	for (int i=0; i<28; ++i){
		memcpy(&motor_velocity[i], &buf_read[buf_count], 4);
		buf_count += 4;
	}
	memcpy(&time_counter, &buf_read[buf_count], 4);
	buf_count += 4;
	memcpy(&time_in_seconds, &buf_read[buf_count], 4);
	buf_count += 4;
	for (int i=0; i<42; ++i){
		memcpy(&endeffector_positions[i], &buf_read[buf_count], 4);
		buf_count += 4;
	}
	for (int i=0; i<2; ++i){
		memcpy(&gripper_force[i], &buf_read[buf_count], 2);
		buf_count += 2;
	}
	memcpy(&distance_sensor, &buf_read[buf_count], 2);
	buf_count += 2;
	memcpy(&battery_voltage, &buf_read[buf_count], 2);
	buf_count += 2;
	memcpy(&robot_status, &buf_read[buf_count], 1);
	buf_count += 1;

	//printf("motor pos:\n");
	//for (int i=0; i<28;++i){
	//	printf("% .2f,",motor_position[i]);
	//}


	return success;
}




HoapSendTask::HoapSendTask() : start_time_(0), freq_(0), amp_(0) {

	pthread_t t1;
	pthread_create(&t1, NULL, hoapListeningThread, NULL);


}

HoapSendTask::~HoapSendTask() {

}


int HoapSendTask::initialize()
{
	start_time_ = 0.0;

	static int firsttime = TRUE;

	if (firsttime){
		firsttime = FALSE;

		freq_=0.05;
		amp_=0.25;
	}

	float joint_rotations[28];
	for (int i=0; i<28; ++i)
		joint_rotations[i]=0;
	hoap_connect(joint_rotations);


	// prepare going to the default posture
	bzero((char *)&(target_[1]),N_DOFS*sizeof(target_[1]));
	for (int i=1; i<=N_DOFS; i++)
	{
		target_[i] = joint_default_state[i];
	}

	// go to the target using inverse dynamics (ID)
	if (!go_target_wait_ID(target_))
	{
		return FALSE;
	}

	// ready to go
	int ans = 999;
	while (ans == 999) {
		if (!get_int(const_cast<char*>("Enter 1 to start or anthing else to abort ..."),ans,&ans))
		{
			return FALSE;
		}
	}

	// only go when user really types the right thing
	if (ans != 1)
	{
		return FALSE;
	}




	start_time_ = task_servo_time;
	printf("start time = %.3f, task_servo_time = %.3f\n", start_time_, task_servo_time);

	return TRUE;
}



int HoapSendTask::run()
{
	double task_time = task_servo_time - start_time_;
	double omega = 4.0*PI*freq_;

	// NOTE: all array indices start with 1 in SL
	for (int i=LA_J3; i<=LA_J4; ++i) {
		target_[i].th   = joint_default_state[i].th + 2*amp_*sin(omega*task_time);
		target_[i].thd = amp_ * omega*cos(omega*task_time);
		target_[i].thdd = -amp_ * omega*omega*sin(omega*task_time);
	}


	// the following variables need to be assigned
	for (int i=1; i<=N_DOFS; ++i) {
		joint_des_state[i].th = target_[i].th;
		joint_des_state[i].thd = target_[i].thd;
		joint_des_state[i].thdd = target_[i].thdd;
		joint_des_state[i].uff = 0.0;
	}

	//sending
	float joint_rot[N_DOFS+1];
	for (int i=1; i<=N_DOFS; ++i)
		joint_rot[i]= target_[i].th;

	if (hoap_connected)
		hoap_run(joint_rot);

	//check if reading works. may be wrong buffer offsets. read and write need to be in separate threads
	//reading
	//hoap_read();


	return TRUE;
}

static void diep(const char *s)
{
	perror(s);
	exit(1);
}
static int mainSend(char *buf)
{
	struct sockaddr_in si_other;
	struct sockaddr_in si_my;
	int ret, s, i, slen=sizeof(si_other);
	//char buf[BUFLEN];

	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
		diep("socket");

	int opt=1;
	int err;
	err=setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	memset((char *) &si_my, 0, sizeof(si_my));
	si_my.sin_family = AF_INET;
	si_my.sin_port = htons(MY_PORT);

	ret=bind(s, (struct sockaddr *) &si_my, sizeof(struct sockaddr));

	memset((char *) &si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(HOAP_PORT);
	if (inet_aton(SRV_IP, &si_other.sin_addr)==0) {
		fprintf(stderr, "inet_aton() failed\n");
		exit(1);
	}

	//printf("Sending packet %d\n", i);
	if (sendto(s, buf, SND_BUFLEN, 0, (struct sockaddr*)&si_other, slen)==-1)
		diep("sendto()");

	close(s);

	return 0;
}

static int mainRead(char * buf)
{
	//returns 1 on success
	struct sockaddr_in si_me, si_other;
	int s, i;
	socklen_t slen=sizeof(si_other);
	int error=0;

	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
		diep("socket");

	int opt=1;
	int err;
	err=setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	memset((char *) &si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(MY_PORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(s, (struct sockaddr*)&si_me, sizeof(si_me))==-1)
		diep("bind");

	if (recvfrom(s, buf, RCV_BUFLEN, 0, (struct sockaddr*)&si_other, &slen)==-1){
		error=1;
		diep("recvfrom()");
	}
	//printf("Received packet from %s:%d\nData: %s\n\n",inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port), buf);

	close(s);
	return !error;
}


int HoapSendTask::changeParameters()
{

	hoap_disconnect();
	return TRUE;
}

int HoapSendTask::hoap_disconnect()
{
	int success;
	//disconnect from hoap
	char buf[SND_BUFLEN], buf_read[RCV_BUFLEN];

	float command_period=0.01;
	int command_counter=0;
	char filter='f';
	char command_type='j';

	int motor_power[21];
	for (int i=0; i<21; ++i)
		motor_power[i]=0;

	float joint_rotations[28];
	for (int i=0; i<28; ++i)
		joint_rotations[i]=0;

	float joint_velocities[28];
	for (int i=0; i<28; ++i)
		joint_velocities[i]=0;

	char user_status='e';	//disconnect


	//construct buffer for sending
	int buf_count=0;
	memcpy(buf, (void*) &command_period, sizeof(int));
	buf_count+=sizeof(int);
	memcpy(&buf[buf_count], (void*) &command_counter, sizeof(int));
	buf_count+=sizeof(int);
	buf[buf_count++]=filter;
	buf[buf_count++]=command_type;
	buf_count+=2;
	for (int i=0; i<21; ++i){
		memcpy(&buf[buf_count], (void*) &motor_power[i], sizeof(int));
		buf_count+=sizeof(int);
	}
	for (int i=0; i<28; ++i){
		memcpy(&buf[buf_count], (void*) &joint_rotations[i], sizeof(float));
		buf_count+=sizeof(float);
	}
	for (int i=0; i<28; ++i){
		memcpy(&buf[buf_count], (void*) &joint_velocities[i], sizeof(float));
		buf_count+=sizeof(float);
	}
	buf[buf_count++]= user_status;

	if (!mainSend(buf)){
		success = TRUE;
		hoap_connected = FALSE;
		printf ("hoap disconnected\n");
	}
	return success;
}

int HoapSendTask::hoap_connect(float joint_rot[N_DOFS+1])
{
	//connect to hoap
	int success=FALSE;

	char buf[SND_BUFLEN];

	float command_period=0.01;
	int command_counter=0;
	char filter='f';
	char command_type='j';

	int motor_power[21];
	for (int i=0; i<21; ++i)
		motor_power[i]=1;

	float joint_rotations[28];
	for (int i=0; i<28; ++i)
		joint_rotations[i]=joint_default_state[i+1].th;
	//joint_rotations[LA_J2]= PI/6;

	float joint_velocities[28];
	for (int i=0; i<28; ++i)
		joint_velocities[i]=0;

	char user_status='b';	//begin

	//construct buffer for sending
	int buf_count=0;
	memcpy(buf, (void*) &command_period, sizeof(int));
	buf_count+=sizeof(int);
	memcpy(&buf[buf_count], (void*) &command_counter, sizeof(int));
	buf_count+=sizeof(int);
	buf[buf_count++]=filter;
	buf[buf_count++]=command_type;
	buf_count+=2;
	for (int i=0; i<21; ++i){
		memcpy(&buf[buf_count], (void*) &motor_power[i], sizeof(int));
		buf_count+=sizeof(int);
	}
	for (int i=0; i<28; ++i){
		memcpy(&buf[buf_count], (void*) &joint_rotations[i], sizeof(float));
		buf_count+=sizeof(float);
	}
	for (int i=0; i<28; ++i){
		memcpy(&buf[buf_count], (void*) &joint_velocities[i], sizeof(float));
		buf_count+=sizeof(float);
	}
	buf[buf_count++]= user_status;

	if (!mainSend(buf)){
		success = TRUE;
		hoap_connected = TRUE;
	}
	return success;
}



int HoapSendTask::hoap_run(float joint_rot[N_DOFS+1])
{
	//connect to hoap
	int success;

	char buf[SND_BUFLEN];

	float command_period=0.01;
	static int command_counter=0;
	char filter='f';
	char command_type='j';

	int motor_power[21];
	for (int i=0; i<21; ++i)
		motor_power[i]=1;

	float joint_rotations[28];
	for (int i=0; i<28; ++i)
		joint_rotations[i]=joint_rot[i+1];

	float joint_velocities[28];
	for (int i=0; i<28; ++i)
		joint_velocities[i]=0;

	char user_status='c';	//continue


	//construct buffer for sending
	int buf_count=0;
	memcpy(buf, (void*) &command_period, sizeof(int));
	buf_count+=sizeof(int);
	memcpy(&buf[buf_count], (void*) &command_counter, sizeof(int));
	buf_count+=sizeof(int);
	buf[buf_count++]=filter;
	buf[buf_count++]=command_type;
	buf_count+=2;
	for (int i=0; i<21; ++i){
		memcpy(&buf[buf_count], (void*) &motor_power[i], sizeof(int));
		buf_count+=sizeof(int);
	}
	for (int i=0; i<28; ++i){
		memcpy(&buf[buf_count], (void*) &joint_rotations[i], sizeof(float));
		buf_count+=sizeof(float);
	}
	for (int i=0; i<28; ++i){
		memcpy(&buf[buf_count], (void*) &joint_velocities[i], sizeof(float));
		buf_count+=sizeof(float);
	}
	buf[buf_count++]= user_status;

	success =! mainSend(buf);
	command_counter++;
	return success;
}

