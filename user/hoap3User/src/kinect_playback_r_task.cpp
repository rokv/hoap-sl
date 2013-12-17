/*============================================================================
==============================================================================
							KinectPlaybackRobot.cpp
==============================================================================
Remarks : Reproduction of human movement estimated by Kinect
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
#include "SL_filters.h"
#include "SL_common.h"

//#include "SL_for_dynamics_body.h"
#include "SL_terrains.h"
#include "SL_objects.h"
#include "unistd.h"

// Local includes
#include "kinect_playback_r_task.h"
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <math.h>

static int servo_rate = SERVO_BASE_RATE / TASK_SERVO_RATIO;

#define MAX_BUFFER  10000
#define KINECT_FREQ 30
static const char *fifo_name = "KinectReaderFIFO";
static int Kinect_pipe = 0;
const double freq_factor = 2.0;

// #define GENERATE_LEG_MOTION
#define MAX_LENGTH 10*60

#define my_SQR(a) ((a)*(a))
#define my_CUBE(a) ((a)*(a)*(a))

// time out for semaphores
#define TIME_OUT_NS 1000000000

//#define COMMUNICATION
#define SRV_IP "178.172.42.110"
#define SND_BUFLEN 324
#define RCV_BUFLEN 440
#define HOAP_PORT 15005
#define MY_PORT 15006
static int mainSend(char *buf);
static int mainRead(char *buf);

static int hoap_read();
static int hoap_connected=FALSE;
static float motor_position_old[28]; // compare for new data
static char firstTime=0;

// prototypes correction and limits functions
static void correctAngsRightLeg(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w);
static void correctAngsLeftLeg(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w);
static void correctAngsRightArm(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w);
static void correctAngsLeftArm(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w);

static void* hoapListeningThread(void * arg)
{
	int success=0;
	printf("Ready to connect\n");
	while(1)
	{
		sleep(0.001);
		if (hoap_connected)
		{
			success=hoap_read(); //read motors position
		}
	}
	return 0;
}

static int hoap_read()
{
	int success;
	float radToDegres = 180/PI;
	float degToRadians = PI/180;
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


	for (int i=0; i<28; i++) // copy motors positions
	{
		memcpy(&motor_position[i], &buf_read[buf_count], 4);
		buf_count += 4;
	}
	for (int y=0; y<20; y++)
	{
		if(firstTime>19) // no display the initialization
		{
			if ((fabs(motor_position[y] - motor_position_old[y])) > 2*degToRadians) // changes ???
			{
				printf("motor_position[%d] = %.2f \n", y, motor_position[y]*radToDegres); //printf new data

				motor_position_old[y] = motor_position[y]; // update motor position
			}
		}
		else
		{
			firstTime++;
			motor_position_old[y] = motor_position[y];
		}
	}
	return success;
}

KinectPlaybackRobot::KinectPlaybackRobot() : start_time_(0), previous_time_(0)
{
	pthread_t t1;
	pthread_create(&t1, NULL, hoapListeningThread, NULL);
	if (Kinect_pipe != 0)
    {
		close(Kinect_pipe);
		Kinect_pipe = 0;
    }
	task_servo_steps = 0;
}

KinectPlaybackRobot::~KinectPlaybackRobot()
{
	if (Kinect_pipe > 0)
    close(Kinect_pipe);
}

int KinectPlaybackRobot::initialize()
{
	int ans;
	int tmp=0;
	start_time_ = 0.0;
    static int firsttime = TRUE;

    if (firsttime)
    	firsttime = FALSE;

	float joint_rotations[28];
	for (int i=0; i<28; ++i)
		joint_rotations[i]=0;
	hoap_connect(joint_rotations);

	//check whether any other task is running
	if (strcmp(current_task_name, NO_TASK) != 0)
	{
		printf("New task can only be run if no other task is running!\n");
		return FALSE;
	}

	// meta parameters
    track = TRUE;              // Reproduce human motion measured by Kinect on a robot
    use_cop = FALSE;            // Use center of pressure for stability control
    save_active_dofs = TRUE;   // Save active or all degrees of freedom

    playback = TRUE;          // Directly reproduce Kinect data
    if (real_robot_flag)
    {
    	track_legs = FALSE;        // Control legs by Kinect
    	track_torso = FALSE;       // Control torso by Kinect
    }
    else
    {
    	track_legs = FALSE;        // Control legs by Kinect
    	track_torso = FALSE;       // Control torso by Kinect
    }
    balance_with_legs = FALSE; // Control legs to improve balance control
    desired = real_robot_flag;

    if (real_robot_flag)
    	use_cop = FALSE;

    // ensure consistency
    if (playback == TRUE)
    {
    	track_legs = track_torso = balance_with_legs = FALSE;
    }
    else if (track_legs == TRUE || track_torso == TRUE)
    	balance_with_legs = FALSE;

    // set active degrees of freedom
    control_dofs();

    // prepare Kinect communication
    if (!KinectReader(fifo_name, NULL))
    {
    	printf("Communication with Kinect could not be established!\n");
    	return FALSE;
    }
    count_Kinect_frames = 0;
    waiting_for_data = 0;
    last_kinect_time = -1.0;
    time1 = time2 = time_kinect1 = time_kinect2 = -1.0;

    // initialize Butterworth filters for Kinect data
    if (!init_filters())
    	return FALSE;
    for (int i = 1; i <= N_DOFS; i++)
    	fth[i].cutoff = 9;
    filters_initialized = FALSE;
    for (int i = 0; i < 3; i++)
    {
    	for (int j = 0; j < 4; j++)
    	{
    		if (i != j)
    			initial_torso_pose[i][j] = 0.0;
    		else
    			initial_torso_pose[i][j] = 1.0;
    	}
    }

    // prepare going to the initial posture
    bzero((char *)&(target_[1]), N_DOFS*sizeof(target_[1]));
    for (int i = 1; i <= N_DOFS; i++)
    	target_[i] = joint_default_state[i];
  	target_[LA_J4].th = -0,05;
  	target_[RA_J4].th = 0.05;
  	target_[B_J].th = 0.2;

	freezeBase(TRUE);

	changeRealTime(TRUE);

	// ready to go
	ans = 0;
	while (ans != 1)
	{
		if (!get_int(const_cast<char*>("Enter 1 to start or 'q' to abort ... \n"), ans, &ans))
		{
			return FALSE;
		}
	}

	previous_time_ = start_time_ = task_servo_time;
	printf("Servo rate: %d, servo base rate: %d, ratio: %d.\n", servo_rate, SERVO_BASE_RATE, TASK_SERVO_RATIO);
	printf("Start time: %.3f, task servo time: %.3f\n", start_time_, task_servo_time);

	// go to the target
	bool there = true;
	for (int i = 1; i <= N_DOFS; i++)
	{
		if (fabs(target_[i].th - joint_des_state[i].th) > 1.0e-3)
		{
			there = false;
			break;
		}
	}
	if (!there)
		if (!go_target_wait_ID(target_))
		{
			return FALSE;
		}
	printf("Initial position reached. Now start tracking!\n\n");

	// set the desired joint state equal to the commanded joints
	for (int i = 1; i <= N_DOFS; i++)
	{
		joint_des_state[i].th = target_[i].th;
		joint_des_state[i].thd = target_[i].thd = 0;
		joint_des_state[i].thdd = target_[i].thdd = 0;
	}
	return TRUE;
}

int KinectPlaybackRobot::run()
{
	int i;
	double task_time = task_servo_time - start_time_;
	char buffer[MAX_BUFFER];
	double body_TFE, body_TAA;

	#ifdef GENERATE_LEG_MOTION
	#include "leg_playback1.h"
	#endif

	// Check if new data from Kinect arrived
	if (KinectReader(NULL, buffer) == 720)
	{
		kinect2joint(buffer);

		if (track)
		{
			// Initialize filtering and legt tracking
			if (!filters_initialized)
				{
					printf("Kinect tracker initialization!\n");

					for (i = 1; i <= N_DOFS; i++)
						{
							fth[i].raw[0] = fth[i].raw[1] = fth[i].raw[2] = target_[i].th;
							fth[i].filt[0] = fth[i].filt[1] = fth[i].filt[2] = target_[i].th;
						}
					filters_initialized = TRUE;
					time_kinect1 = time1 = time_kinect2 = time2 = -1.0;
					last_kinect_time = -1.0;
					waiting_for_data = 0;
				}
			else
			{
				// Filter Kinect data with a Butterworth filter
				for (i = 1; i <= N_DOFS; i++)
					target_[i].th = filt(target_[i].th, &fth[i]);
			}

			last_kinect_time = task_time;

			// In this mode we simply copy Kinect trajectories to the robot and exit
			if (playback && track)
				{
					joint_range[RL_J3][MAX_THETA] = PI * 45.0 / 180.0;
					joint_range[LL_J3][MAX_THETA] = PI * 45.0 / 180.0; //-
					for (i = 1; i <= active_dofs[0]; i++)
						if (joint_range[active_dofs[i]][MAX_THETA] < target_[active_dofs[i]].th)
							target_[active_dofs[i]].th = joint_range[active_dofs[i]][MAX_THETA];
						else if (joint_range[active_dofs[i]][MIN_THETA] > target_[active_dofs[i]].th)
							target_[active_dofs[i]].th = joint_range[active_dofs[i]][MIN_THETA];

					//kinect_velocities(task_time);
					for (i = 1; i <= active_dofs[0]; i++)
					{
						joint_des_state[active_dofs[i]].th = target_[active_dofs[i]].th;
						joint_des_state[active_dofs[i]].thd = target_[active_dofs[i]].thd;
						joint_des_state[active_dofs[i]].thdd = target_[active_dofs[i]].thdd;
					}
					return TRUE;
				}

		}
	}
	else if ((task_time - last_kinect_time) > (30.0 / KINECT_FREQ))
	{
		filters_initialized = FALSE;
		time_kinect1 = time1 = time_kinect2 = time2 = -1.0;
		last_kinect_time = -1.0;
		waiting_for_data++;

		if ((waiting_for_data % (2*servo_rate)) == 0)
			printf("No data from Kinect for too long. Now waiting ... \n");
	}

	task_servo_steps++;
	previous_time_ = task_time;

	//sending
	float joint_rot[N_DOFS+1];
	for (int i=1; i<=N_DOFS; ++i)
  	joint_rot[i]= target_[i].th;

  	if (hoap_connected)
  		hoap_run(joint_rot);

  	return TRUE;
}

int KinectPlaybackRobot::changeParameters()
{
	hoap_disconnect();
    return TRUE;
}

int KinectPlaybackRobot::KinectReader(const char *fifo_name, char *buf)
{
	int i, j, n, n_tmp;

	// Open pipe to receive data from Kinect
	if (fifo_name != NULL)
	{
		printf("Waiting to open Kinect pipe %s!\n", fifo_name);
		Kinect_pipe = open(fifo_name, O_RDONLY | O_NONBLOCK);
		if (Kinect_pipe <= 0)
		{
			printf("Kinect pipe could not be opened!\n");
			Kinect_pipe = 0;
			return 0;
		}
		printf("Kinect pipe opened for reading, pipe ID %d!\n", Kinect_pipe);

		return 1;
	}

	// Receive data
	if (Kinect_pipe == 0)
		return 0;
	else
	{
		n = 0;
		do
		{
			n_tmp = read(Kinect_pipe, buf, MAX_BUFFER);
			if (n_tmp == 720)
			{
				n = n_tmp;
				if (n < MAX_BUFFER)
					buf[n] = '\0';
				count_Kinect_frames++;
			}
		}
		while (n_tmp > 0);

		if ((task_servo_steps % 500) == 500)
		{
			//printf("time: %lf, number of packets: %d, %d\n", task_servo_time, packet_count, n);
		}
	}
	return n;
}

typedef struct {float x, y, z, w;} Quat; // Quaternion
enum QuatPart {X, Y, Z, W};
typedef float HMatrix[4][4]; // Right-handed, for column vectors _
typedef Quat EulerAngles;
#define EulOrd(i,p,r,f)	   (((((((i)<<1)+(p))<<1)+(r))<<1)+(f))
#define EulOrdZYXr    EulOrd(X,EulParEven,EulRepNo,EulFrmR)
#define EulOrdXYXr    EulOrd(X,EulParEven,EulRepYes,EulFrmR)
#define EulOrdYZXr    EulOrd(X,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdXZXr    EulOrd(X,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdXZYr    EulOrd(Y,EulParEven,EulRepNo,EulFrmR)
#define EulOrdYZYr    EulOrd(Y,EulParEven,EulRepYes,EulFrmR)
#define EulOrdZXYr    EulOrd(Y,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdYXYr    EulOrd(Y,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdYXZr    EulOrd(Z,EulParEven,EulRepNo,EulFrmR)
#define EulOrdZXZr    EulOrd(Z,EulParEven,EulRepYes,EulFrmR)
#define EulOrdXYZr    EulOrd(Z,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdZYZr    EulOrd(Z,EulParOdd,EulRepYes,EulFrmR)
#define EulParOdd     1
#define EulParEven    0
#define EulFrmR	      1
#define EulFrmS	      0
#define EulRepNo      0
#define EulRepYes     1
#define EulOrdXYZs    EulOrd(X,EulParEven,EulRepNo,EulFrmS)
#define EulOrdXYXs    EulOrd(X,EulParEven,EulRepYes,EulFrmS)
#define EulOrdXZYs    EulOrd(X,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdXZXs    EulOrd(X,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdYZXs    EulOrd(Y,EulParEven,EulRepNo,EulFrmS)
#define EulOrdYZYs    EulOrd(Y,EulParEven,EulRepYes,EulFrmS)
#define EulOrdYXZs    EulOrd(Y,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdYXYs    EulOrd(Y,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdZXYs    EulOrd(Z,EulParEven,EulRepNo,EulFrmS)
#define EulOrdZXZs    EulOrd(Z,EulParEven,EulRepYes,EulFrmS)
#define EulOrdZYXs    EulOrd(Z,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdZYZs    EulOrd(Z,EulParOdd,EulRepYes,EulFrmS)

extern "C"
{
	EulerAngles Eul_FromHMatrix(HMatrix M, int order);
}

static void matMultABt(double A[3][4], double B[3][4], double C[3][4])
{
	int i, j, k;
	for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				C[i][j] = 0;
				for (k = 0; k < 3; k++)
				{
					C[i][j] += A[i][k]*B[j][k];
				}
			}
		}
}

void KinectPlaybackRobot::kinect2joint(char buffer[])
{
	int i, j, k, tmp;
	char *buffer_pt;

	double tmp_rot[3][4], tmp_rot2[3][4];
	double elbNum, elbDen; //elbow numerator, denumerator
	double rot45[3][4];
	HMatrix R;
	EulerAngles outAngs = {0,0,0,EulOrdYXZr};
	EulerAngles newOutAngs = {0,0,0,EulOrdYXZr};
	float x, y, z, w;
	float radToDegres = 180/PI;
	double kneeSt, kneeIm; // knee steve, imenovalec

	buffer_pt = &(buffer[0]);
	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 3; j++)
		{
			joint_pose[i][j][3] = ((float *) buffer_pt)[0];
			buffer_pt += 4;
		}
		for (k = 0; k < 3; k++)
			{
				for (j = 0; j < 3; j++)
				{
					joint_pose[i][j][k] = ((float *) buffer_pt)[0];
					buffer_pt += 4;
				}
			}
	}

	if ((count_Kinect_frames % 300) == 300)
	{
		printf("Counter %d\n", count_Kinect_frames);
	    for (i = 0; i < 15; i++)
	    {
	    	for (j = 0; j < 3; j++)
	    	{
	    		for (k = 0; k < 4; k++)
	    			printf("%lf ", joint_pose[i][j][k]);
	    		printf("\n");
	    	}
	    	printf("\n");
	    }
	    printf("\n\n");
	 }

	for (i = 0; i < 3; i++)
	tmp_rot[i][3] = tmp_rot2[i][3] = 0.0;

	////// TORSO ///////////////////////////////////////////////////////////////////

	matMultABt(initial_torso_pose, joint_pose[2], tmp_rot);
	for (i = 0; i < 3; ++i)
		for (j = 0; j < 4; ++j)
			R[i][j] = tmp_rot[i][j];
	R[3][0] = R[3][1] = R[3][2] = 0.0f; R[3][3] = 1.0f;

	outAngs = Eul_FromHMatrix(R, EulOrdXZYr);

	//target_[B_J].th = -outAngs.x; //there are no limits for the moment

	////// RIGHT LEG ///////////////////////////////////////////////////////////////

	// Knee:
	kneeSt = (joint_pose[13][0][3] - joint_pose[12][0][3]) * (joint_pose[13][0][3] - joint_pose[14][0][3]) +
			 (joint_pose[13][1][3] - joint_pose[12][1][3]) * (joint_pose[13][1][3] - joint_pose[14][1][3]) +
			 (joint_pose[13][2][3] - joint_pose[12][2][3]) * (joint_pose[13][2][3] - joint_pose[14][2][3]);

	kneeIm = sqrt(my_SQR((joint_pose[13][0][3]-joint_pose[12][0][3])) +
				  my_SQR((joint_pose[13][1][3]-joint_pose[12][1][3])) +
				  my_SQR((joint_pose[13][2][3]-joint_pose[12][2][3]))) *
			 sqrt(my_SQR((joint_pose[13][0][3]-joint_pose[14][0][3])) +
				  my_SQR((joint_pose[13][1][3]-joint_pose[14][1][3])) +
				  my_SQR((joint_pose[13][2][3]-joint_pose[14][2][3])));

	newOutAngs.w = -(acos(kneeSt/kneeIm) - PI);

	// Hip:
	matMultABt(initial_torso_pose, joint_pose[12], tmp_rot);

	for (i = 0; i < 3; ++i)
		for (j = 0; j < 4; ++j)
			R[i][j] = tmp_rot[i][j];
	R[3][0] = R[3][1] = R[3][2] = 0.0f; R[3][3] = 1.0f;

	outAngs = Eul_FromHMatrix(R, EulOrdYXZr);

	newOutAngs.x = outAngs.x;
	newOutAngs.y = outAngs.y;
	newOutAngs.z = -outAngs.z;

	correctAngsRightLeg(&newOutAngs.x, &newOutAngs.y, &newOutAngs.z, &newOutAngs.w, &x, &y, &z, &w);

	target_[RL_J1].th = x; // Euler z
	target_[RL_J3].th = y; // Euler x
	target_[RL_J2].th = z; // Euler y

	target_[RL_J4].th= w;

	////// LEFT LEG //////////////////////////////////////////////////////////////////

	// Knee:
	kneeSt = (joint_pose[10][0][3]-joint_pose[9][0][3]) * (joint_pose[10][0][3]-joint_pose[11][0][3]) +
			 (joint_pose[10][1][3]-joint_pose[9][1][3]) * (joint_pose[10][1][3]-joint_pose[11][1][3]) +
			 (joint_pose[10][2][3]-joint_pose[9][2][3]) * (joint_pose[10][2][3]-joint_pose[11][2][3]);

	kneeIm = sqrt(my_SQR(joint_pose[10][0][3] - joint_pose[9][0][3]) +
				  my_SQR(joint_pose[10][1][3] - joint_pose[9][1][3]) +
				  my_SQR(joint_pose[10][2][3]-joint_pose[9][2][3])) *
			 sqrt(my_SQR(joint_pose[10][0][3]-joint_pose[11][0][3]) +
				  my_SQR(joint_pose[10][1][3]-joint_pose[11][1][3]) +
				  my_SQR(joint_pose[10][2][3]-joint_pose[11][2][3]));

	newOutAngs.w = (acos(kneeSt/kneeIm) - PI);

	// Hip:
	matMultABt(initial_torso_pose, joint_pose[9], tmp_rot);

	for (i = 0; i < 3; ++i)
		for (j = 0; j < 4; ++j)
			R[i][j] = tmp_rot[i][j];
	R[3][0] = R[3][1] = R[3][2] = 0.0f; R[3][3] = 1.0f;

	outAngs = Eul_FromHMatrix(R, EulOrdYXZr);

	newOutAngs.x = outAngs.x;
	newOutAngs.y = -outAngs.y;
	newOutAngs.z = -outAngs.z;

	correctAngsLeftLeg(&newOutAngs.x, &newOutAngs.y, &newOutAngs.z, &newOutAngs.w, &x, &y, &z, &w);

	target_[LL_J1].th = x; // Euler z
	target_[LL_J3].th = y; // Euler x
	target_[LL_J2].th = z; // Euler y

	target_[LL_J4].th = w;


	///// RIGHT ARM //////////////////////////////////////////////////////////

	// Elbow:
	elbNum = (joint_pose[7][0][3]-joint_pose[6][0][3]) * (joint_pose[7][0][3]-joint_pose[8][0][3]) +
			 (joint_pose[7][1][3]-joint_pose[6][1][3]) * (joint_pose[7][1][3]-joint_pose[8][1][3]) +
			 (joint_pose[7][2][3]-joint_pose[6][2][3]) * (joint_pose[7][2][3]-joint_pose[8][2][3]);

	elbDen = sqrt(my_SQR(joint_pose[7][0][3]-joint_pose[6][0][3]) +
				  my_SQR(joint_pose[7][1][3]-joint_pose[6][1][3]) +
				  my_SQR(joint_pose[7][2][3]-joint_pose[6][2][3])) *
			 sqrt(my_SQR(joint_pose[7][0][3]-joint_pose[8][0][3]) +
				  my_SQR(joint_pose[7][1][3]-joint_pose[8][1][3]) +
				  my_SQR(joint_pose[7][2][3]-joint_pose[8][2][3]));

	newOutAngs.w = -(acos(elbNum/elbDen) - PI);

	// Shoulder:
	matMultABt(joint_pose[2], joint_pose[6], tmp_rot);

	for (i = 0; i < 3; ++i)
		for (j = 0; j < 4; ++j)
			R[i][j] = tmp_rot[i][j];
	R[3][0] = R[3][1] = R[3][2] = 0.0f; R[3][3] = 1.0f;

	outAngs = Eul_FromHMatrix(R, EulOrdXZXr);

	newOutAngs.x = -(outAngs.x - PI/2);
	newOutAngs.y = -(outAngs.y + PI/2);
	newOutAngs.z = outAngs.z;

	correctAngsRightArm(&newOutAngs.x, &newOutAngs.y, &newOutAngs.z, &newOutAngs.w, &x, &y, &z, &w);

	target_[RA_J1].th = x;		// outAngs.x = first= X
	target_[RA_J2].th = y;	// outAngs.y = 2nd= Z
	target_[RA_J3].th = -z;		// outAngs.z = 3rd= X

	target_[RA_J4].th = w;


	///// LEFT ARM /////////////////////////////////////////////////////

	// Elbow:
	elbNum = (joint_pose[4][0][3]-joint_pose[3][0][3]) * (joint_pose[4][0][3]-joint_pose[5][0][3]) +
			 (joint_pose[4][1][3]-joint_pose[3][1][3]) * (joint_pose[4][1][3]-joint_pose[5][1][3]) +
			 (joint_pose[4][2][3]-joint_pose[3][2][3]) * (joint_pose[4][2][3]-joint_pose[5][2][3]);

	elbDen = sqrt(my_SQR(joint_pose[4][0][3]-joint_pose[3][0][3]) +
				  my_SQR(joint_pose[4][1][3]-joint_pose[3][1][3]) +
				  my_SQR(joint_pose[4][2][3]-joint_pose[3][2][3])) *
			 sqrt(my_SQR(joint_pose[4][0][3]-joint_pose[5][0][3]) +
				  my_SQR(joint_pose[4][1][3]-joint_pose[5][1][3]) +
				  my_SQR(joint_pose[4][2][3]-joint_pose[5][2][3]));

	newOutAngs.w = (acos(elbNum/elbDen) - PI);

	// Shoulder:
	matMultABt(joint_pose[2], joint_pose[3], tmp_rot);
	for (i = 0; i < 3; ++i)
		for (j = 0; j < 4; ++j)
			R[i][j] = tmp_rot[i][j];
	R[3][0] = R[3][1] = R[3][2] = 0.0f; R[3][3] = 1.0f;

	outAngs = Eul_FromHMatrix(R, EulOrdXZXr);

	newOutAngs.x = (outAngs.x + PI/2);
	newOutAngs.y = (outAngs.y + PI/2);
	newOutAngs.z = -(outAngs.z + PI);

	correctAngsLeftArm(&newOutAngs.x, &newOutAngs.y, &newOutAngs.z, &newOutAngs.w, &x, &y, &z, &w);

	//target_[LA_J1].th = x;		// outAngs.x = first= X
	target_[LA_J2].th = y;	// outAngs.y = 2nd= Z
	//target_[LA_J3].th = z;		// outAngs.z = 3rd= X

	target_[LA_J4].th = w;
}

// Here we define which degrees of freedom are active (depending on the control mode)
void KinectPlaybackRobot::control_dofs(void)
{

	int kinect_playback_dofs[] = {17, RL_J1, RL_J2, RL_J3, RL_J4,
                                  RA_J1, RA_J2, RA_J3, RA_J4,
                                  LL_J1, LL_J2, LL_J3, LL_J4,
                                  LA_J1, LA_J2, LA_J3, LA_J4,
                                  B_J};
  	int i;

  	active_legs_dofs[0] = 0;

  	for (i = 0; i <= kinect_playback_dofs[0]; i++)
  		active_dofs[i] = kinect_playback_dofs[i];

  	ndofs = active_dofs[0];
}

// Right leg : correction and limits
static void correctAngsRightLeg(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w)
{
	float degToRadian = PI/180;
  	float radToDegres = 180/PI;

  	static float tab_ub[4] = {30*degToRadian, 81*degToRadian, 0*degToRadian, 129*degToRadian}; // upper limits
  	static float tab_lb[4] = {-90*degToRadian, -70*degToRadian, -30*degToRadian, 0*degToRadian}; // lower limits

  	static float tab_eulerAngles[4];
  	char i;

  	tab_eulerAngles[0] = *pt1;
  	tab_eulerAngles[1] = *pt2;
  	tab_eulerAngles[2] = *pt3;
  	tab_eulerAngles[3] = *pt4;

  	//limits
  	for(i=0; i<4; i++)
  	{
  		if(tab_eulerAngles[i] > tab_ub[i])
  			tab_eulerAngles[i] = tab_ub[i];
  		else if (tab_eulerAngles[i] < tab_lb[i])
  			tab_eulerAngles[i] = tab_lb[i];
  	}

  	*x = tab_eulerAngles[0];
  	*y = tab_eulerAngles[1];
  	*z = tab_eulerAngles[2];
  	*w = tab_eulerAngles[3];
}

// Leftleg : correction and limits
static void correctAngsLeftLeg(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w)
{
  	float degToRadian = PI/180;
  	float radToDegres = 180/PI;

  	static float tab_ub[4] = {90*degToRadian, 70*degToRadian, 30*degToRadian, 0*degToRadian}; // upper limits
  	static float tab_lb[4] = {-30*degToRadian, -81*degToRadian, 0*degToRadian, -129*degToRadian}; // lower limits

  	static float tab_eulerAngles[4];
  	char i;

  	tab_eulerAngles[0] = *pt1;
  	tab_eulerAngles[1] = *pt2;
  	tab_eulerAngles[2] = *pt3;
  	tab_eulerAngles[3] = *pt4;

  	//limits
  	for(i=0; i<4; i++)
  	{
  		if(tab_eulerAngles[i] > tab_ub[i])
  			tab_eulerAngles[i] = tab_ub[i];
  		else if (tab_eulerAngles[i] < tab_lb[i])
  			tab_eulerAngles[i] = tab_lb[i];
  	}

  	*x = tab_eulerAngles[0];
  	*y = tab_eulerAngles[1];
  	*z = tab_eulerAngles[2];
  	*w = tab_eulerAngles[3];
}

// Right arm : correction and limits
static void correctAngsRightArm(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w)
{
	float degToRadian = PI/180;
  	float radToDegres = 180/PI;

  	static float tab_ub[4] = {143*degToRadian, 0*degToRadian,90*degToRadian, 114*degToRadian}; // upper limits
  	static float tab_lb[4] = {-90*degToRadian, -95*degToRadian, -90*degToRadian, 0*degToRadian}; // lower limits

  	static float tab_eulerAngles[4];
  	char i;
  	float x1, y1,z1,w1;

  	//corrections
  	if (*pt1 > PI)
  		x1 = *pt1 - PI;
  	else
  		x1 = *pt1;

  	y1 = *pt2;

  	if (*pt3 < - PI)
  		z1 = *pt3 + PI;
  	else
  		z1 = *pt3;

  	w1 = *pt4;

  	tab_eulerAngles[0] = x1;
  	tab_eulerAngles[1] = y1;
  	tab_eulerAngles[2] = z1;
  	tab_eulerAngles[3] = w1;

  	//limits
  	for(i=0; i<4; i++)
  	{
  		if(tab_eulerAngles[i] > tab_ub[i])
  			tab_eulerAngles[i] = tab_ub[i];
  		else if (tab_eulerAngles[i] < tab_lb[i])
  			tab_eulerAngles[i] = tab_lb[i];
  	}

  	*x = tab_eulerAngles[0];
  	*y = tab_eulerAngles[1];
  	*z = tab_eulerAngles[2];
  	*w = tab_eulerAngles[3];
}

// Left arm : correction and limits
static void correctAngsLeftArm(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w)
{
  	float degToRadian = PI/180;
  	float radToDegres = 180/PI;

  	static float tab_ub[4] = {90*degToRadian, 95*degToRadian, 90*degToRadian, 0*degToRadian}; // upper limits
  	static float tab_lb[4] = {-143*degToRadian, 0*degToRadian, -90*degToRadian, -114*degToRadian}; // lower limits

  	static float tab_eulerAngles[4];
  	char i;
  	float x1, y1,z1,w1;

  	//correction
  	if (*pt1 > PI)
  		x1 = *pt1 - PI;
  	else
  		x1 = *pt1;

  	y1 = *pt2;

  	if (*pt3 < - PI)
  		z1 = *pt3 + PI;
  	else
  		z1 = *pt3;

  	w1 = *pt4;

  	tab_eulerAngles[0] = x1;
  	tab_eulerAngles[1] = y1;
  	tab_eulerAngles[2] = z1;
  	tab_eulerAngles[3] = w1;

  	//limits
  	for(i=0; i<4; i++)
  	{
  		if(tab_eulerAngles[i] > tab_ub[i])
  			tab_eulerAngles[i] = tab_ub[i];
  		else if (tab_eulerAngles[i] < tab_lb[i])
  			tab_eulerAngles[i] = tab_lb[i];
  	}

  	*x = tab_eulerAngles[0];
  	*y = tab_eulerAngles[1];
  	*z = tab_eulerAngles[2];
  	*w = tab_eulerAngles[3];
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
	if (inet_aton(SRV_IP, &si_other.sin_addr)==0)
	{
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

	if (recvfrom(s, buf, RCV_BUFLEN, 0, (struct sockaddr*)&si_other, &slen)==-1)
	{
		error=1;
		diep("recvfrom()");
	}
	//printf("Received packet from %s:%d\nData: %s\n\n",inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port), buf);

	close(s);
	return !error;
}

int KinectPlaybackRobot::hoap_disconnect()
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
	for (int i=0; i<21; ++i)
	{
		memcpy(&buf[buf_count], (void*) &motor_power[i], sizeof(int));
		buf_count+=sizeof(int);
	}
	for (int i=0; i<28; ++i)
	{
		memcpy(&buf[buf_count], (void*) &joint_rotations[i], sizeof(float));
		buf_count+=sizeof(float);
	}
	for (int i=0; i<28; ++i)
	{
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

int KinectPlaybackRobot::hoap_connect(float joint_rot[N_DOFS+1])
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
	for (int i=0; i<21; ++i)
	{
		memcpy(&buf[buf_count], (void*) &motor_power[i], sizeof(int));
		buf_count+=sizeof(int);
	}
	for (int i=0; i<28; ++i)
	{
		memcpy(&buf[buf_count], (void*) &joint_rotations[i], sizeof(float));
		buf_count+=sizeof(float);
	}
	for (int i=0; i<28; ++i)
	{
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



int KinectPlaybackRobot::hoap_run(float joint_rot[N_DOFS+1])
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
	for (int i=0; i<21; ++i)
	{
		memcpy(&buf[buf_count], (void*) &motor_power[i], sizeof(int));
		buf_count+=sizeof(int);
	}
	for (int i=0; i<28; ++i)
	{
		memcpy(&buf[buf_count], (void*) &joint_rotations[i], sizeof(float));
		buf_count+=sizeof(float);
	}
	for (int i=0; i<28; ++i)
	{
		memcpy(&buf[buf_count], (void*) &joint_velocities[i], sizeof(float));
		buf_count+=sizeof(float);
	}
	buf[buf_count++]= user_status;

	success =! mainSend(buf);
	command_counter++;
	return success;
}
