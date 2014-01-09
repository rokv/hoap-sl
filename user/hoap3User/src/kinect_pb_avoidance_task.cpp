/*============================================================================
==============================================================================
							KinectPlayback.cpp
==============================================================================
Remarks:
      Reproduction of human movement estimated by Kinect
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
#include "kinect_pb_avoidance_task.h"
#include <fcntl.h>
#include <arpa/inet.h>
#include <math.h>
#include "avoidanceTask.h"

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

static void correctAngsRightLeg(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w);
static void correctAngsLeftLeg(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w);
static void correctAngsRightArm(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w);
static void correctAngsLeftArm(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w);

avoidanceTask avoidance_task2;

KinectPBavoidance::KinectPBavoidance() : start_time_(0), previous_time_(0)
{
	if (Kinect_pipe != 0)
	{
		close(Kinect_pipe);
		Kinect_pipe = 0;
	}
	task_servo_steps = 0;
}

KinectPBavoidance::~KinectPBavoidance()
{
	if (Kinect_pipe > 0)
    close(Kinect_pipe);
}

int KinectPBavoidance::initialize()
{
	int ans;

	/* check whether any other task is running */
	if (strcmp(current_task_name, NO_TASK) != 0)
	{
		printf("New task can only be run if no other task is running!\n");
		return FALSE;
	}

	// meta parameters
	track = TRUE;              // Reproduce human motion measured by Kinect on a robot
	avoidance = TRUE;			//Use self collision avoidance algorithm before sending to the robot
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
		for (int j = 0; j < 4; j++)
			if (i != j)
				initial_torso_pose[i][j] = 0.0;
			else
				initial_torso_pose[i][j] = 1.0;

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
		if (!get_int(const_cast<char*>("Enter 1 to start or 'q' to abort ..."), ans, &ans))
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
		if (fabs(target_[i].th - joint_des_state[i].th) > 1.0e-3)
		{
			there = false;
			break;
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

int KinectPBavoidance::run()
{
	int i, j;
	double task_time = task_servo_time - start_time_;
	char buffer[MAX_BUFFER];
	double time_step = 1.0 / (double) task_servo_rate;
	double body_TFE, body_TAA;
	float degToRadian = PI/180;

	// Check if new data from Kinect arrived
	if (KinectReader(NULL, buffer) == 720)
	{
		kinect2joint(buffer);

		if (track)
		{
			// Initialize filtering and leg tracking
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
			if (playback && track && !avoidance)
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
					joint_des_state[active_dofs[i]].th = 	target_[active_dofs[i]].th;
				}
				return TRUE;
			}
			else if (avoidance && track)
			{
				double *dq_RL_des;
				double *dq_LL_des;
				double *dq_RA_des;
				double *dq_LA_des;

				int n_used_leg_dofs=6;
				int n_used_arm_dofs=4;

				dq_RA_des = my_vector(1, n_used_arm_dofs);
				dq_LA_des = my_vector(1, n_used_arm_dofs);
				dq_RL_des = my_vector(1, n_used_leg_dofs);
				dq_LL_des = my_vector(1, n_used_leg_dofs);

				get_velocity(task_time);

				dq_RA_des[1]= target_[RA_J1].thd;
				dq_RA_des[2]= target_[RA_J2].thd;
				dq_RA_des[3]= target_[RA_J3].thd;
				dq_RA_des[4]= target_[RA_J4].thd;

				dq_LA_des[1]= target_[LA_J1].thd;
				dq_LA_des[2]= target_[LA_J2].thd;
				dq_LA_des[3]= target_[LA_J3].thd;
				dq_LA_des[4]= target_[LA_J4].thd;

				dq_RL_des[1]= target_[RL_J1].thd;
				dq_RL_des[2]= target_[RL_J2].thd;
				dq_RL_des[3]= target_[RL_J3].thd;
				dq_RL_des[4]= target_[RL_J4].thd;
				dq_RL_des[5]= target_[RL_J5].thd;
				dq_RL_des[6]= target_[RL_J6].thd;

				dq_LL_des[1]= target_[LL_J1].thd;
				dq_LL_des[2]= target_[LL_J2].thd;
				dq_LL_des[3]= target_[LL_J3].thd;
				dq_LL_des[4]= target_[LL_J4].thd;
				dq_LL_des[5]= target_[LL_J5].thd;
				dq_LL_des[6]= target_[LL_J6].thd;

				avoidance_task2.avoidance(&dq_RL_des[0], &dq_LL_des[0], &dq_RA_des[0], &dq_LA_des[0]);

				//kinect_velocities(task_time);
				/*for (i = 1; i <= active_dofs[0]; i++)
				{
					joint_des_state[active_dofs[i]].th = 	target_[active_dofs[i]].th;
				}*/

				my_free_vector(dq_RL_des, 1, n_used_leg_dofs);
				my_free_vector(dq_LL_des, 1, n_used_leg_dofs);
				my_free_vector(dq_LA_des, 1, n_used_arm_dofs);
				my_free_vector(dq_RA_des, 1, n_used_arm_dofs);

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
	    return TRUE;
}

int KinectPBavoidance::changeParameters()
{
	int ivar;
	double dvar;

	get_int(const_cast<char*>("This is how to enter an integer variable"),ivar,&ivar);
	get_double(const_cast<char*>("This is how to enter a double variable"),dvar,&dvar);

	return TRUE;
}

int KinectPBavoidance::KinectReader(const char *fifo_name, char *buf)
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

typedef struct {float x, y, z, w;} Quat; /* Quaternion */
enum QuatPart {X, Y, Z, W};
typedef float HMatrix[4][4]; /* Right-handed, for column vectors _*/
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

void KinectPBavoidance::matMultABt(double A[3][4], double B[3][4], double C[3][4])
{
	int i, j, k;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
		{
			C[i][j] = 0;
			for (k = 0; k < 3; k++)
				C[i][j] += A[i][k]*B[j][k];
		}
}

void KinectPBavoidance::kinect2joint(char buffer[])
{
	int i, j, k;
	char *buffer_pt;

	double dq[3+1];
	double** A;
	double** V;
	double* s;
	A=my_matrix(1, 3 , 1, 3);
	V=my_matrix(1, 3 , 1, 3);
	s=my_vector(1, 3);
	double epsilon;
	double s_max;
	double s_min;
	double rank;


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
			for (j = 0; j < 3; j++)
			{
				joint_pose[i][j][k] = ((float *) buffer_pt)[0];
				buffer_pt += 4;
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

    //target_[B_J].th = -outAngs.x; //x

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

    double elbow = -(acos(elbNum/elbDen) - PI);

    // Shoulder:
    matMultABt(joint_pose[2], joint_pose[6], tmp_rot);

    //switch to 1-indexing
    double R_kin[3+1][3+1];
    for (i=1; i<=3; ++i){
    	for (j=1; j<=3; ++j){
    		R_kin[i][j]=tmp_rot[j-1][i-1]; //T
    	}
    }

    //get kinect (=desired) axis angle between torso and shoulder
    double fi_kin=acos(1/2*(R_kin[1][1]+R_kin[2][2]+R_kin[3][3] - 1));

    double axis_fi_kin[3+1];
    axis_fi_kin[1]= (R_kin[3][2]-R_kin[2][3]) / (2*sin(fi_kin)) * fi_kin;
    axis_fi_kin[2]= (R_kin[1][3]-R_kin[3][1]) / (2*sin(fi_kin)) * fi_kin;
    axis_fi_kin[3]= (R_kin[2][1]-R_kin[1][2]) / (2*sin(fi_kin)) * fi_kin;

    //get robot (=current) axis angle between torso and shoulder. C.s. need to be rotated the same way as kinect's (see ogrinc's report).
    double R_torso[3+1][3+1];
    double R_shoulder[3+1][3+1];
    double R_current[3+1][3+1];
    double R_temp[3+1][3+1];

    for (i=1; i<=3; ++i){
    	for (j=1; j<=3; ++j){
    		R_torso[i][j]=Alink[BASE][i][j];
    	}
    }
    rotX(PI/2, R_torso, R_temp);
    rotY(-PI/2, R_temp, R_torso);

    for (i=1; i<=3; ++i){
    	for (j=1; j<=3; ++j){
    		R_temp[i][j]=Alink[R_UA][i][j];
    	}
    }
    rotZ(joint_state[RA_J4].th, R_temp, R_shoulder); //to fix it to upper arm instead of elbow
    rotY(-PI/2, R_shoulder, R_temp);
    rotZ(PI/2, R_temp, R_shoulder);

    matMultAtB13(R_shoulder, R_torso, R_current);

    double fi_cur=acos(1/2*(R_current[1][1]+R_current[2][2]+R_current[3][3] - 1));

    double axis_fi_cur[3+1];
    axis_fi_cur[1]=(R_current[3][2]-R_current[2][3]) / (2*sin(fi_cur)) * fi_cur;
    axis_fi_cur[2]=(R_current[1][3]-R_current[3][1]) / (2*sin(fi_cur)) * fi_cur;
    axis_fi_cur[3]=(R_current[2][1]-R_current[1][2]) / (2*sin(fi_cur)) * fi_cur;

    //needed change in orientation is the difference between current and desired
    double step = 0.5;
    double dp[3+1];
    dp[1] = step* (axis_fi_kin[1] - axis_fi_cur[1]);
    dp[2] = step* (axis_fi_kin[2] - axis_fi_cur[2]);
    dp[3] = step* (axis_fi_kin[3] - axis_fi_cur[3]);

    //maximum allowed norm of dp
    double max_dp_norm=0.5;
    if (sqrt(dp[1]*dp[1] + dp[2]*dp[2]+ dp[3]*dp[3]) > max_dp_norm){
    	dp[1] *= max_dp_norm;
    	dp[2] *= max_dp_norm;
    	dp[3] *= max_dp_norm;
    }

    double newAxis[3+1];
    double oldAxis[3+1];
    if (  sqrt(dp[1]*dp[1] + dp[2]*dp[2] + dp[3]*dp[3]) < max_dp_norm *0.01  ){
    	//if dp is very low do nothing, rarely happens due to oscilaltions
		target_[RA_J1].th += 0;
		target_[RA_J2].th += 0;
		target_[RA_J3].th += 0;
    }
    else
    {
		//needed change in shoulder dq is given by jacobian, dq=J^-1 dp. Since we are dealing with rotation jacobian columns are equal to unit axes of joints.
    	//the axes need to be in the same cs as the one used to get dp, given by R_torso.
		for (i=1; i<=3; ++i){
			oldAxis[i]=joint_axis_pos[RA_J1][i];
		}
		matTmultVec(R_torso, oldAxis, newAxis);
		for (i=1; i<=3; ++i){
			A[i][1]= - newAxis[i];
		}

		for (i=1; i<=3; ++i){
			oldAxis[i]=joint_axis_pos[RA_J2][i];
		}
		matTmultVec(R_torso, oldAxis, newAxis);
		for (i=1; i<=3; ++i){
			A[i][2]= - newAxis[i];
		}

		for (i=1; i<=3; ++i){
			oldAxis[i]=joint_axis_pos[RA_J3][i];
		}
		matTmultVec(R_torso, oldAxis, newAxis);
		for (i=1; i<=3; ++i){
			A[i][3]= - newAxis[i];
		}

		my_svdcmp(A, 3 , 3, s, V);
		epsilon=0.01;
		s_max = 0.0;  rank = 0;
		for (j = 1; j <= 3; j++)
			if (s[j] > s_max)
				s_max = s[j];
		s_min = s_max*epsilon;
		for (j = 1; j <= 3; j++)
			if (s[j] < s_min)
				s[j] = 0.0;
			else
				rank++;
		my_svbksb(A, s, V, 3, 3, dp, dq);


		target_[RA_J1].th += dq[1];
		target_[RA_J2].th += dq[2];
		target_[RA_J3].th += dq[3];

		target_[RA_J1].th = fmod(target_[RA_J1].th, (2*PI));
		target_[RA_J2].th = fmod(target_[RA_J2].th, (2*PI));
		target_[RA_J3].th = fmod(target_[RA_J3].th, (2*PI));


    }

    target_[RA_J4].th = elbow;

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

    elbow = (acos(elbNum/elbDen) - PI);

    // Shoulder:
    matMultABt(joint_pose[2], joint_pose[3], tmp_rot);
    //switch to 1-indexing
    R_kin[3+1][3+1];
    for (i=1; i<=3; ++i){
    	for (j=1; j<=3; ++j){
    		R_kin[i][j]=tmp_rot[j-1][i-1]; //T
    	}
    }

    //get kinect (=desired) axis angle between torso and shoulder
    fi_kin=acos(1/2*(R_kin[1][1]+R_kin[2][2]+R_kin[3][3] - 1));

    axis_fi_kin[3+1];
    axis_fi_kin[1]= (R_kin[3][2]-R_kin[2][3]) / (2*sin(fi_kin)) * fi_kin;
    axis_fi_kin[2]= (R_kin[1][3]-R_kin[3][1]) / (2*sin(fi_kin)) * fi_kin;
    axis_fi_kin[3]= (R_kin[2][1]-R_kin[1][2]) / (2*sin(fi_kin)) * fi_kin;

    //get robot (=current) axis angle between torso and shoulder. C.s. need to be rotated the same way as kinect's (see ogrinc's report).
    for (i=1; i<=3; ++i){
    	for (j=1; j<=3; ++j){
    		R_torso[i][j]=Alink[BASE][i][j];
    	}
    }
    rotX(PI/2, R_torso, R_temp);
    rotY(-PI/2, R_temp, R_torso);

    for (i=1; i<=3; ++i){
    	for (j=1; j<=3; ++j){
    		R_temp[i][j]=Alink[L_UA][i][j];
    	}
    }
    rotZ(-joint_state[LA_J4].th, R_temp, R_shoulder); //to fix it to upper arm instead of elbow
    rotY(-PI/2, R_shoulder, R_temp);
    rotZ(-PI/2, R_temp, R_shoulder);

    matMultAtB13(R_shoulder, R_torso, R_current);

    fi_cur=acos(1/2*(R_current[1][1]+R_current[2][2]+R_current[3][3] - 1));

    axis_fi_cur[1]=(R_current[3][2]-R_current[2][3]) / (2*sin(fi_cur)) * fi_cur;
    axis_fi_cur[2]=(R_current[1][3]-R_current[3][1]) / (2*sin(fi_cur)) * fi_cur;
    axis_fi_cur[3]=(R_current[2][1]-R_current[1][2]) / (2*sin(fi_cur)) * fi_cur;

    //needed change in orientation is the difference between current and desired
    step = 0.1; //previously defined for right arm
    dp[1] = step* (axis_fi_kin[1] - axis_fi_cur[1]);
    dp[2] = step* (axis_fi_kin[2] - axis_fi_cur[2]);
    dp[3] = step* (axis_fi_kin[3] - axis_fi_cur[3]);

    //maximum allowed norm of dp
    max_dp_norm=0.5; //previously defined for right arm
    if (sqrt(dp[1]*dp[1] + dp[2]*dp[2]+ dp[3]*dp[3]) > max_dp_norm){
    	dp[1] *= max_dp_norm;
    	dp[2] *= max_dp_norm;
    	dp[3] *= max_dp_norm;
    }

    if (  sqrt(dp[1]*dp[1] + dp[2]*dp[2] + dp[3]*dp[3]) < max_dp_norm *0.01  ){
    	//if dp is very low do nothing, rarely happens due to oscilaltions
    	target_[RA_J1].th += 0;
    	target_[RA_J2].th += 0;
    	target_[RA_J3].th += 0;
    }
    else
    {
    	//needed change in shoulder dq is given by jacobian, dq=J^-1 dp. Since we are dealing with rotation jacobian columns are equal to unit axes of joints.
    	//the axes need to be in the same cs as the one used to get dp, given by R_torso.

    	for (i=1; i<=3; ++i){
    		oldAxis[i]=joint_axis_pos[LA_J1][i];
    	}
    	matTmultVec(R_torso, oldAxis, newAxis);
    	for (i=1; i<=3; ++i){
    		A[i][1]= - newAxis[i];
    	}

    	for (i=1; i<=3; ++i){
    		oldAxis[i]=joint_axis_pos[LA_J2][i];
    	}
    	matTmultVec(R_torso, oldAxis, newAxis);
    	for (i=1; i<=3; ++i){
    		A[i][2]= - newAxis[i];
    	}

    	for (i=1; i<=3; ++i){
    		oldAxis[i]=joint_axis_pos[LA_J3][i];
    	}
    	matTmultVec(R_torso, oldAxis, newAxis);
    	for (i=1; i<=3; ++i){
    		A[i][3]= - newAxis[i];
    	}

    	my_svdcmp(A, 3 , 3, s, V);
    	epsilon=0.01;
    	s_max = 0.0;  rank = 0;
    	for (j = 1; j <= 3; j++)
    		if (s[j] > s_max)
    			s_max = s[j];
    	s_min = s_max*epsilon;
    	for (j = 1; j <= 3; j++)
    		if (s[j] < s_min)
    			s[j] = 0.0;
    		else
    			rank++;
    	my_svbksb(A, s, V, 3, 3, dp, dq);


    	target_[LA_J1].th += dq[1];
    	target_[LA_J2].th += dq[2];
    	target_[LA_J3].th += dq[3];

    	target_[LA_J1].th = drem(target_[LA_J1].th, (2*PI));
    	target_[LA_J2].th = drem(target_[LA_J2].th, (2*PI));
    	target_[LA_J3].th = drem(target_[LA_J3].th, (2*PI));

    }


    target_[LA_J4].th = elbow;



    my_free_matrix(A, 1, 3, 1, 3);
    my_free_matrix(V, 1, 3, 1, 3);
    my_free_vector(s, 1, 3);
}

void KinectPBavoidance::matMultVec(double A[3+1][3+1], double B[3+1], double C[3+1])
{//matrix multiplication A*b=c, index starts at 1.
	int i, j;
	for (i = 1; i <= 3; i++){
		C[i] = 0;
		for (j = 1; j <= 3; j++)
		{
			C[i] += A[i][j]*B[j];
		}
	}
}

void KinectPBavoidance::matTmultVec(double A[3+1][3+1], double B[3+1], double C[3+1])
{//matrix multiplication A'*b=c, index starts at 1.
	int i, j;
	for (i = 1; i <= 3; i++){
		C[i] = 0;
		for (j = 1; j <= 3; j++)
		{
			C[i] += A[j][i]*B[j];
		}
	}
}

void KinectPBavoidance::matMultAtB13(double A[3+1][3+1], double B[3+1][3+1], double C[3+1][3+1])
{//matrix multiplication A'*B=C, index starts at 1.
int i, j, k;
  for (i = 1; i <= 3; i++)
    for (j = 1; j <= 3; j++)
    {
      C[i][j] = 0;
      for (k = 1; k <= 3; k++)
    	C[i][j] += A[k][i]*B[k][j];
    }
}

void KinectPBavoidance::matMultABt13(double A[3+1][3+1], double B[3+1][3+1], double C[3+1][3+1])
{//matrix multiplication A*B'=C, index starts at 1.
int i, j, k;
  for (i = 1; i <= 3; i++)
    for (j = 1; j <= 3; j++)
    {
      C[i][j] = 0;
      for (k = 1; k <= 3; k++)
    	C[i][j] += A[i][k]*B[j][k];
    }
}
void KinectPBavoidance::matMultAB13(double A[3+1][3+1], double B[3+1][3+1], double C[3+1][3+1])
{//matrix multiplication A*B=C, index starts at 1.
int i, j, k;
  for (i = 1; i <= 3; i++)
    for (j = 1; j <= 3; j++)
    {
      C[i][j] = 0;
      for (k = 1; k <= 3; k++)
    	C[i][j] += A[i][k]*B[k][j];
    }
}

void KinectPBavoidance::rotZ(double fi, double rotin[3+1][3+1], double rotout[3+1][3+1])
{//rotates rotin by fi around z, index starts at 1
	double rotM[3+1][3+1];
	rotM[1][1] = cos(fi); rotM[1][2] = -sin(fi); rotM[1][3] = 0.0;
	rotM[2][1] = sin(fi); rotM[2][2] =  cos(fi); rotM[2][3] = 0.0;
	rotM[3][1] = 0.0;     rotM[3][2] = 0.0;      rotM[3][3] = 1.0;

	matMultAB13(rotin, rotM, rotout);
}
void KinectPBavoidance::rotY(double fi, double rotin[3+1][3+1], double rotout[3+1][3+1])
{//rotates rotin by fi around y, index starts at 1
	double rotM[3+1][3+1];
	rotM[1][1] = cos(fi);	rotM[1][2] = 0.0; 	 rotM[1][3] = sin(fi);
	rotM[2][1] = 0.0;		rotM[2][2] = 1.0;	 rotM[2][3] = 0.0;
	rotM[3][1] =-sin(fi);	rotM[3][2] = 0.0;    rotM[3][3] = cos(fi);

	matMultAB13(rotin, rotM, rotout);
}
void KinectPBavoidance::rotX(double fi, double rotin[3+1][3+1], double rotout[3+1][3+1])
{//rotates rotin by fi around x, index starts at 1
	double rotM[3+1][3+1];
	rotM[1][1] = 1.0; rotM[1][2] = 0.0;  		rotM[1][3] = 0.0;
	rotM[2][1] = 0.0; rotM[2][2] = cos(fi);		rotM[2][3] =-sin(fi);
	rotM[3][1] = 0.0; rotM[3][2] = sin(fi);  	rotM[3][3] = cos(fi);

	matMultAB13(rotin, rotM, rotout);
}


// Here we define which degrees of freedom are active (depending on the control mode)
void KinectPBavoidance::control_dofs(void)
{

	int kinect_playback_dofs[] = {17, B_J, RL_J1, RL_J2, RL_J3, RL_J4,
                                  RA_J1, RA_J2, RA_J3, RA_J4,
                                  LL_J1, LL_J2, LL_J3, LL_J4,
                                  LA_J1, LA_J2, LA_J3, LA_J4,
                                 };
  	int i;

  	active_legs_dofs[0] = 0;

  	for (i = 0; i <= kinect_playback_dofs[0]; i++)
  		active_dofs[i] = kinect_playback_dofs[i];

  	ndofs = active_dofs[0];
}

static void correctAngsRightLeg(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w) // Right leg : correction and limits
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

static void correctAngsLeftLeg(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w) // Left leg : correction and limits
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

static void correctAngsRightArm(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w) // Right arm : correction and limits
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

static void correctAngsLeftArm(float *pt1, float *pt2, float *pt3, float *pt4, float *x, float *y, float *z, float *w) // Left arm : correction and limits
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

void KinectPBavoidance::get_velocity(double timestamp)
{
double dt;
static double target_old[N_DOFS];
static double timestamp_old;
static int first_time=1;

        if (first_time==1){
                for (int i=1; i<=N_DOFS; ++i){
                        target_old[i]= target_[i].th;
                }
                timestamp_old= timestamp;
                first_time= 0;
        }
        else{
                dt = timestamp - timestamp_old;

                for (int i=1; i<=N_DOFS; ++i){
                        if (0 && prehod > 8000){
                                target_[i].thd = (target_[i].th - target_old[i]) /dt;

                                target_old[i]= target_[i].th;
                        }
                        else{
                                target_[i].thd = (target_[i].th - joint_des_state[i].th);// /dt;
                        }
                }
                timestamp_old= timestamp;
        }

        time_kinect1=time_kinect2=1;//bl
}
