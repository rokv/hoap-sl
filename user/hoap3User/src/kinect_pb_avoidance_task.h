/*=============================================================================
  =============================================================================
  \file    KinectPlayback.h
  \author  Gaetan LAVOUE
  \date    July 2013
  =============================================================================
  \remarks
  ============================================================================*/
#ifndef KinectPBavoidance_H_
#define KinectPBavoidance_H_

#include "SL_filters.h"

class KinectPBavoidance
{
	public:

	KinectPBavoidance();
	~KinectPBavoidance();

	int initialize();
	int run();
	int changeParameters();

	private:

	double start_time_, previous_time_;
	SL_DJstate target_[N_DOFS+1];

	double joint_pose[15][3][4], initial_torso_pose[3][4];
	Filter fth[N_DOFS+1];
	int filters_initialized;
	double time1, time2, time_kinect1, time_kinect2, last_kinect_time;

	int active_dofs[N_DOFS+1], active_legs_dofs[14+1];
	int leg_index;

	int track;
	int task_servo_steps;
	int count_Kinect_frames, waiting_for_data;
	int ndofs;
	int save_active_dofs, use_cop, cog_printing, desired;
	int track_legs, track_torso, balance_with_legs, playback;
	int prehod;
	int avoidance;

	double knee_kinect_initial, knee_initial, offset_hip, offset_ankle;
	double body_tilt, ankle_tilt;

	void matMultABt(double A[3][4], double B[3][4], double C[3][4]);
	void matMultAtB13(double A[3+1][3+1], double B[3+1][3+1], double C[3+1][3+1]);
	void matMultABt13(double A[3+1][3+1], double B[3+1][3+1], double C[3+1][3+1]);
	void matMultAB13(double A[3+1][3+1], double B[3+1][3+1], double C[3+1][3+1]);
	void matMultVec(double A[3+1][3+1], double B[3+1], double C[3+1]);
	void matTmultVec(double A[3+1][3+1], double B[3+1], double C[3+1]);
	void rotZ(double fi, double rotin[3+1][3+1], double rotout[3+1][3+1]);
	void rotY(double fi, double rotin[3+1][3+1], double rotout[3+1][3+1]);
	void rotX(double fi, double rotin[3+1][3+1], double rotout[3+1][3+1]);

	int KinectReader(const char *fifo_name, char buffer[]);
	void kinect2joint(char buffer[]);
	void control_dofs(void);
	void get_velocity(double timestamp);

	int Kinect_collection_idx, CBi_collection_idx, Kinect_DOFs;

};

#endif // KinectPBavoidance_H_
