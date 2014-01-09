/*!=============================================================================
  ==============================================================================

  \file    avoidanceTask.h

  \author  Erwan LEGROS manage by Rok VUGA
  \date    DEC. 2013

  ==============================================================================
  \remarks

  avoidance task that is written in C++

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
#include "avoidanceTask.h"



avoidanceTask::avoidanceTask() : start_time_(0), freq_(0), amp_(0), proximity_factor_LA(0), proximity_factor_RA(0), proximity_factor_RL(0), proximity_factor_LL(0) {
	n_used_leg_dofs=6;
	n_used_arm_dofs=4;

	max_dp_RL=0.1;
	max_dp_LL=0.1;
	max_dp_RA=0.1;
	max_dp_LA=0.1;
}

avoidanceTask::~avoidanceTask() {

}


int avoidanceTask::initialize()
{
  start_time_ = 0.0;

  static int firsttime = TRUE;


  if (firsttime){
    firsttime = FALSE;
    freq_ = 0.1; // frequency
    amp_  = 0.5; // amplitude
  }

  // prepare going to the default posture
  bzero((char *)&(target_[1]),N_DOFS*sizeof(target_[1]));
  for (int i=1; i<=N_DOFS; i++)
    {
      target_[i] = joint_default_state[i];
    }


  /*target_[LA_J1].th = 0;//0.2;
    target_[LA_J2].th = 0;//-0.7;
   target_[LA_J3].th = PI/2;//0;//PI/2;
    target_[LA_J4].th = 0;//-1.8;//-1;//-1.6;

    target_[RA_J1].th = 0;PI/4;
    target_[RA_J2].th = 0;//0;
    target_[RA_J3].th = PI/2;//PI/2;
    target_[RA_J4].th = 0;PI/2;*/

    target_[LL_J1].th = 0;
    target_[LL_J2].th = 0;
   target_[LL_J3].th = -PI/3;
  target_[LL_J4].th = -PI/4;
  target_[LL_J5].th = 0;
    target_[LL_J6].th =0 ;

   target_[RL_J1].th = 0;//-PI/2;
   target_[RL_J2].th = 0;
 target_[RL_J3].th = PI/3;
  target_[RL_J4].th = PI/4;
  target_[RL_J5].th = 0;
      target_[RL_J6].th =0;
  //go to the target using inverse dynamics (ID)
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

int avoidanceTask::run()
{
	double *dq_RL_des;
	double *dq_LL_des;
	double *dq_RA_des;
	double *dq_LA_des;
	double task_time = task_servo_time - start_time_;
	double omega = 2.0*PI*freq_;

	dq_RA_des = my_vector(1, n_used_arm_dofs);
	dq_LA_des = my_vector(1, n_used_arm_dofs);
	dq_RL_des = my_vector(1, n_used_leg_dofs);
	dq_LL_des = my_vector(1, n_used_leg_dofs);

  // NOTE: all array indices start with 1 in SL
  /*for (int i=1; i<=N_DOFS; ++i) {
    target_[i].th   = joint_default_state[i].th + amp_*sin(omega*task_time);
  }*/

  // the following variables need to be assigned
  for (int i=1; i<=N_DOFS; ++i) {
    //joint_des_state[i].th = target_[i].th;

  }



  for (int i=1; i<=n_used_arm_dofs; i++)
  {
	  dq_LA_des[i]= -0.0005*sin(omega*task_time) ;
	  dq_RA_des[i]= 0.0005*sin(omega*task_time);
  }

  for (int i=1; i<=n_used_leg_dofs; i++)
    {
  	  dq_LL_des[i]= 0.0005*sin(omega*task_time);
  	  dq_RL_des[i]= -0.0005*sin(omega*task_time);
    }
  avoidance(dq_RL_des, dq_LL_des, dq_RA_des,dq_LA_des);

  my_free_vector(dq_RL_des, 1, n_used_leg_dofs);
  my_free_vector(dq_LL_des, 1, n_used_leg_dofs);
  my_free_vector(dq_LA_des, 1, n_used_arm_dofs);
  my_free_vector(dq_RA_des, 1, n_used_arm_dofs);

  return TRUE;
}
void avoidanceTask::avoidance(double *dq_RL_des,double *dq_LL_des,double *dq_RA_des,double *dq_LA_des)
{

	  get_all_distances();
	  get_all_jacobian_coordinates();

	  /* ----------- Task priority control ----------- */
	  /* ----------- declarations for the legs ----------- */

	  double dq_leg[6+1];
	  double **A_leg;
	  double *s_leg;
	  double **V_leg;
	  double **N_leg;
	  double *dq_constrained_leg;
      A_leg = my_matrix(1, 2*N_CART, 1, n_used_leg_dofs);
      V_leg = my_matrix(1, n_used_leg_dofs, 1, n_used_leg_dofs);
      N_leg = my_matrix(1, n_used_leg_dofs, 1, n_used_leg_dofs);
      dq_constrained_leg=  my_vector(1, n_used_leg_dofs);
      s_leg = my_vector(1, n_used_leg_dofs);

      /* ----------- declarations for the arms ----------- */

      double dq_arm[4+1];
      double **A_arm;
      double *s_arm;
      double **V_arm;
      double **N_arm;
      double *dq_constrained_arm;
      A_arm = my_matrix(1, 2*N_CART, 1, n_used_arm_dofs);
      V_arm = my_matrix(1, n_used_arm_dofs, 1, n_used_arm_dofs);
      N_arm = my_matrix(1, n_used_arm_dofs, 1, n_used_arm_dofs);

      dq_constrained_arm=  my_vector(1, n_used_arm_dofs);
      s_arm = my_vector(1, n_used_arm_dofs);

      //dp is the avoidance vector which is proportional to the proximity factor. It points outside of the body.
      double dp[6+1];
      	  	 dp[4]=0;
      	  	 dp[5]=0;
      	  	 dp[6]=0;

      double epsilon=0.01;
      double s_max = 0.0;
      double rank = 0;

                  /* ----------- Right Leg -----------*/
                  dp[_X_]= avoidance_vector_RL[_X_]*proximity_factor_RL *max_dp_RL;
                  dp[_Y_]= avoidance_vector_RL[_Y_]*proximity_factor_RL *max_dp_RL;
                  dp[_Z_]= avoidance_vector_RL[_Z_]*proximity_factor_RL *max_dp_RL;

                  /*for (i=1; i<=N_CART; ++i){
                	  if (dp[i] > max_dp_RL)
                		  dp[i]=max_dp_RL;
                  }*/


                  for(int i= 1; i<= 2*N_CART; i++)
                  {
                	  for(int j= 1; j<= n_used_leg_dofs; j++)
                		  A_leg[i][j]= J_RL[i][j];
                  }

                  my_svdcmp(A_leg, 2*N_CART, n_used_leg_dofs, s_leg, V_leg);

                  for (int j = 1; j <= n_used_leg_dofs; j++)
                	  if (s_leg[j] > s_max)
                		  s_max = s_leg[j];

                  double s_min = s_max*epsilon;
                  for (int j = 1; j <= n_used_leg_dofs; j++)
                	  if (s_leg[j] < s_min)
                		  s_leg[j] = 0.0;
                	  else
                		  rank++;

                  my_svbksb(A_leg, s_leg, V_leg, 2*N_CART, n_used_leg_dofs, dp, dq_leg);

                  for (int j = 1; j <= n_used_leg_dofs; j++)
                	  if (s_leg[j] < s_min)
                		  for (int i = 1; i <= n_used_leg_dofs; i++)
                			  V_leg[i][j] = 0;

                  mat_mult_normal_transpose_size(V_leg, n_used_leg_dofs, n_used_leg_dofs, V_leg, n_used_leg_dofs, n_used_leg_dofs, N_leg);

                  for (int i = 1; i <= n_used_leg_dofs; i++)
                	  for (int j = 1; j <= n_used_leg_dofs; j++)
                		  N_leg[i][j] = -proximity_factor_RL*N_leg[i][j];

                  for (int i = 1; i <= n_used_leg_dofs; i++)
                	  N_leg[i][i] += 1.0;

                  mat_vec_mult_size(N_leg, n_used_leg_dofs, n_used_leg_dofs, dq_RL_des, n_used_leg_dofs, dq_constrained_leg);

                  joint_des_state[RL_J1].th = joint_des_state[RL_J1].th + dq_leg[1] + dq_constrained_leg[1];
                  joint_des_state[RL_J2].th = joint_des_state[RL_J2].th + dq_leg[2] + dq_constrained_leg[2];
                  joint_des_state[RL_J3].th = joint_des_state[RL_J3].th + dq_leg[3] + dq_constrained_leg[3];
                  joint_des_state[RL_J4].th = joint_des_state[RL_J4].th + dq_leg[4] + dq_constrained_leg[4];
                  joint_des_state[RL_J5].th = joint_des_state[RL_J5].th + dq_leg[5] + dq_constrained_leg[5];
                  joint_des_state[RL_J6].th = joint_des_state[RL_J6].th + dq_leg[6] + dq_constrained_leg[6];

                   /*----------- Left Leg ----------- */
                  dp[_X_]= avoidance_vector_LL[_X_]*proximity_factor_LL *max_dp_LL;
                  dp[_Y_]= avoidance_vector_LL[_Y_]*proximity_factor_LL *max_dp_LL;
                  dp[_Z_]= avoidance_vector_LL[_Z_]*proximity_factor_LL *max_dp_LL;
                  for(int i= 1; i<= 2*N_CART; i++)
                  {
                	  for(int j= 1; j<= n_used_leg_dofs; j++)
                		  A_leg[i][j]= J_LL[i][j];
                  }

                  my_svdcmp(A_leg, 2*N_CART, n_used_leg_dofs, s_leg, V_leg);

                  for (int j = 1; j <= n_used_leg_dofs; j++)
                	  if (s_leg[j] > s_max)
                		  s_max = s_leg[j];
                  s_min = s_max*epsilon;
                  for (int j = 1; j <= n_used_leg_dofs; j++)
                	  if (s_leg[j] < s_min)
                		  s_leg[j] = 0.0;
                	  else
                		  rank++;

                  my_svbksb(A_leg, s_leg, V_leg, 2*N_CART, n_used_leg_dofs, dp, dq_leg);

                  for (int j = 1; j <= n_used_leg_dofs; j++)
                	  if (s_leg[j] < s_min)
                		  for (int i = 1; i <= n_used_leg_dofs; i++)
                			  V_leg[i][j] = 0;

                  mat_mult_normal_transpose_size(V_leg, n_used_leg_dofs, n_used_leg_dofs, V_leg, n_used_leg_dofs, n_used_leg_dofs, N_leg);

                  for (int i = 1; i <= n_used_leg_dofs; i++)
                	  for (int j = 1; j <= n_used_leg_dofs; j++)
                		  N_leg[i][j] = -proximity_factor_LL*N_leg[i][j];
                  for (int i = 1; i <= n_used_leg_dofs; i++)
                	  N_leg[i][i] += 1.0;

                  mat_vec_mult_size(N_leg, n_used_leg_dofs, n_used_leg_dofs, dq_LL_des, n_used_leg_dofs, dq_constrained_leg);

                  joint_des_state[LL_J1].th = joint_des_state[LL_J1].th + dq_leg[1] + dq_constrained_leg[1];
                  joint_des_state[LL_J2].th = joint_des_state[LL_J2].th + dq_leg[2] + dq_constrained_leg[2];
                  joint_des_state[LL_J3].th = joint_des_state[LL_J3].th + dq_leg[3] + dq_constrained_leg[3];
                  joint_des_state[LL_J4].th = joint_des_state[LL_J4].th + dq_leg[4] + dq_constrained_leg[4];
                  joint_des_state[LL_J5].th = joint_des_state[LL_J5].th + dq_leg[5] + dq_constrained_leg[5];
                  joint_des_state[LL_J6].th = joint_des_state[LL_J6].th + dq_leg[6] + dq_constrained_leg[6];

                  my_free_matrix(A_leg, 1,  2*N_CART, 1, n_used_leg_dofs);
                  my_free_matrix(V_leg, 1, n_used_leg_dofs, 1, n_used_leg_dofs);
                  my_free_vector(s_leg, 1, n_used_leg_dofs);
                  my_free_matrix(N_leg, 1, n_used_leg_dofs, 1, n_used_leg_dofs);

                  /* ----------- Left arm ----------- */
                  dp[_X_]= avoidance_vector_LA[_X_]*proximity_factor_LA *max_dp_LA;
                  dp[_Y_]= avoidance_vector_LA[_Y_]*proximity_factor_LA *max_dp_LA;
                  dp[_Z_]= avoidance_vector_LA[_Z_]*proximity_factor_LA *max_dp_LA;
                  for(int i= 1; i<= 2*N_CART; i++)
                  {
                	  for(int j= 1; j<= n_used_arm_dofs; j++)
                		  A_arm[i][j]= J_LA[i][j];
                  }

                  my_svdcmp(A_arm, 2*N_CART, n_used_arm_dofs, s_arm, V_arm);

                  for (int j = 1; j <= n_used_arm_dofs; j++)
                	  if (s_arm[j] > s_max)
                		  s_max = s_arm[j];
                  	  	  s_min = s_max*epsilon;
                  for (int j = 1; j <= n_used_arm_dofs; j++)
                	  if (s_arm[j] < s_min)
                		  s_arm[j] = 0.0;
                	  else
                		  rank++;

                  my_svbksb(A_arm, s_arm, V_arm, 2*N_CART, n_used_arm_dofs, dp, dq_arm);

                  for (int j = 1; j <= n_used_arm_dofs; j++)
                	  if (s_arm[j] < s_min)
                		  for (int i = 1; i <= n_used_arm_dofs; i++)
                			  V_arm[i][j] = 0;

                  // Modified null space matrix of center of mass motion
                  mat_mult_normal_transpose_size(V_arm, n_used_arm_dofs, n_used_arm_dofs, V_arm, n_used_arm_dofs, n_used_arm_dofs, N_arm);

                  for (int i = 1; i <= n_used_arm_dofs; i++)
                	  for (int j = 1; j <= n_used_arm_dofs; j++)
                		  N_arm[i][j] = -proximity_factor_LA*N_arm[i][j];
                  for (int i = 1; i <= n_used_arm_dofs; i++)
                	  N_arm[i][i] += 1.0;

                  mat_vec_mult_size(N_arm, n_used_arm_dofs, n_used_arm_dofs, dq_LA_des, n_used_arm_dofs, dq_constrained_arm);

                  joint_des_state[LA_J1].th = joint_des_state[LA_J1].th + dq_arm[1] + dq_constrained_arm[1];
                  joint_des_state[LA_J2].th = joint_des_state[LA_J2].th + dq_arm[2] + dq_constrained_arm[2];
                  joint_des_state[LA_J3].th = joint_des_state[LA_J3].th + dq_arm[3] + dq_constrained_arm[3];
                  joint_des_state[LA_J4].th = joint_des_state[LA_J4].th + dq_arm[4] + dq_constrained_arm[4];

                  /* ----------- Right arm ----------- */
                  dp[_X_]= avoidance_vector_RA[_X_]*proximity_factor_RA *max_dp_RA;
                  dp[_Y_]= avoidance_vector_RA[_Y_]*proximity_factor_RA *max_dp_RA;
                  dp[_Z_]= avoidance_vector_RA[_Z_]*proximity_factor_RA *max_dp_RA;

                  for(int i= 1; i<= 2*N_CART; i++)
                  {
                	  for(int j= 1; j<= n_used_arm_dofs; j++)
                		  A_arm[i][j]= J_RA[i][j];
                  }

                  my_svdcmp(A_arm, 2*N_CART, n_used_arm_dofs, s_arm, V_arm);

                  for (int j = 1; j <= n_used_arm_dofs; j++)
                	  if (s_arm[j] > s_max)
                		  s_max = s_arm[j];
                  	  	  s_min = s_max*epsilon;

                  for (int j = 1; j <= n_used_arm_dofs; j++)
                	  if (s_arm[j] < s_min)
                		  s_arm[j] = 0.0;
                	  else
                		  rank++;

                  my_svbksb(A_arm, s_arm, V_arm, 2*N_CART, n_used_arm_dofs, dp, dq_arm);

                  for (int j = 1; j <= n_used_arm_dofs; j++)
                	  if (s_arm[j] < s_min)
                		  for (int i = 1; i <= n_used_arm_dofs; i++)
                			  V_arm[i][j] = 0;

                  mat_mult_normal_transpose_size(V_arm, n_used_arm_dofs, n_used_arm_dofs, V_arm, n_used_arm_dofs, n_used_arm_dofs, N_arm);

                  for (int i = 1; i <= n_used_arm_dofs; i++)
                	  for (int j = 1; j <= n_used_arm_dofs; j++)
                		  N_arm[i][j] = -proximity_factor_RA*N_arm[i][j];
                  for (int i = 1; i <= n_used_arm_dofs; i++)
                	  	  N_arm[i][i] += 1.0;

                  mat_vec_mult_size(N_arm, n_used_arm_dofs, n_used_arm_dofs, dq_RA_des, n_used_arm_dofs, dq_constrained_arm);

                  joint_des_state[RA_J1].th = joint_des_state[RA_J1].th + dq_arm[1] + dq_constrained_arm[1];
                  joint_des_state[RA_J2].th = joint_des_state[RA_J2].th + dq_arm[2] + dq_constrained_arm[2];
                  joint_des_state[RA_J3].th = joint_des_state[RA_J3].th + dq_arm[3] + dq_constrained_arm[3];
                  joint_des_state[RA_J4].th = joint_des_state[RA_J4].th + dq_arm[4] + dq_constrained_arm[4];

                  my_free_matrix(A_arm, 1,  2*N_CART, 1, n_used_arm_dofs);
                  my_free_matrix(V_arm, 1, n_used_arm_dofs, 1, n_used_arm_dofs);
                  my_free_vector(s_arm, 1, n_used_arm_dofs);
                  my_free_matrix(N_arm, 1, n_used_arm_dofs, 1, n_used_arm_dofs);
  }

double avoidanceTask::get_all_distances()
{
    double dist;
    double av_temp[3+1];
    double av_temp_head[3+1];
    double critic_point_temp[3+1];
    double critic_point_temp_head[3+1];
    double start_segment[N_LINKS+1][3+1];
    double end_segment[N_LINKS+1][3+1];
    double dist_head;
    double dist_point;
    double point_H_1[3+1];
    double point_LA_J3[3+1];
    double point_RA_J3[3+1];
    double point_LL_J4[3+1];
    double point_RL_J4[3+1];
    double shortest_dist_head[12];
    double shortest_dist_segment[12];
    double avoidance_vector_head[3+1];
    double avoidance_vector_segment[3+1];
    double avoidance_vector_point[3+1];
    double critical_point_segment[3+1];
    double critical_point_head[3+1];
    double Alink_body_low[4+1][4+1];
    double Alink_body_up[4+1][4+1];
    double T[4+1][4+1];
    double Alink_back_low[4+1][4+1];
    double Alink_back_up[4+1][4+1];

    for (int i=1; i<=4; ++i)
  	{
  	  	  for (int j=1; j<=4; ++j)
  		  	  Alink_body_low[i][j]=Alink[BASE][i][j];
    }

    for (int i=1; i<=4 ; i++)
    {
  	  for (int j=1; j<=4; j++)
  		  T[i][j]= 0;
    }
    T[1][1]= 1;
    T[2][2]= 1;
    T[3][3]= 1;
    T[4][4]= 1;
    T[1][4]= -0.15;

    matMultAB( Alink_body_low,T, Alink_back_low);

    for (int i=1; i<=4; ++i)
    {
    	for (int j=1; j<=4; ++j)
    		Alink_body_up[i][j]=Alink[B_LINK1][i][j];
    }
    matMultAB( Alink_body_up,T, Alink_back_up);

    	start_segment[1][_X_]=link_pos[R_PT][_X_];			//
        start_segment[1][_Y_]=link_pos[R_PT][_Y_];			//
        start_segment[1][_Z_]=link_pos[R_PT][_Z_];			//
        													//  Right Forearm
        end_segment[1][_X_]=link_pos[R_UA][_X_];			//
        end_segment[1][_Y_]=link_pos[R_UA][_Y_];			//
        end_segment[1][_Z_]=link_pos[R_UA][_Z_];			//


        start_segment[2][_X_]=link_pos[R_SH][_X_];			//
        start_segment[2][_Y_]=link_pos[R_SH][_Y_];			//
        start_segment[2][_Z_]=link_pos[R_SH][_Z_];			//
        													//	Right Arm
        end_segment[2][_X_]=link_pos[R_UA][_X_];			//
        end_segment[2][_Y_]=link_pos[R_UA][_Y_];			//
        end_segment[2][_Z_]=link_pos[R_UA][_Z_];			//


        start_segment[3][_X_]=link_pos[L_PT][_X_];			//
        start_segment[3][_Y_]=link_pos[L_PT][_Y_];			//
        start_segment[3][_Z_]=link_pos[L_PT][_Z_];			//
        													//	Left Forearm
        end_segment[3][_X_]=link_pos[L_UA][_X_];			//
        end_segment[3][_Y_]=link_pos[L_UA][_Y_];			//
        end_segment[3][_Z_]=link_pos[L_UA][_Z_];			//


        start_segment[4][_X_]=link_pos[L_SH][_X_];			//
        start_segment[4][_Y_]=link_pos[L_SH][_Y_];			//
        start_segment[4][_Z_]=link_pos[L_SH][_Z_];			//
        													//	Left Arm
        end_segment[4][_X_]=link_pos[L_UA][_X_];			//
        end_segment[4][_Y_]=link_pos[L_UA][_Y_];			//
        end_segment[4][_Z_]=link_pos[L_UA][_Z_];			//


        start_segment[5][_X_]=link_pos[L_HIP][_X_];			//
        start_segment[5][_Y_]=link_pos[L_HIP][_Y_];			//
        start_segment[5][_Z_]=link_pos[L_HIP][_Z_];			//
        													//	Left High leg
        end_segment[5][_X_]=link_pos[L_KNEE][_X_];			//
        end_segment[5][_Y_]=link_pos[L_KNEE][_Y_];			//
        end_segment[5][_Z_]=link_pos[L_KNEE][_Z_];			//


        start_segment[6][_X_]=link_pos[L_FD][_X_];			//
        start_segment[6][_Y_]=link_pos[L_FD][_Y_];			//
        start_segment[6][_Z_]=link_pos[L_FD][_Z_];			//
        													//	Left Low leg
        end_segment[6][_X_]=link_pos[L_KNEE][_X_];			//
        end_segment[6][_Y_]=link_pos[L_KNEE][_Y_];			//
        end_segment[6][_Z_]=link_pos[L_KNEE][_Z_];			//


        start_segment[7][_X_]=link_pos[R_HIP][_X_];			//
        start_segment[7][_Y_]=link_pos[R_HIP][_Y_];			//
        start_segment[7][_Z_]=link_pos[R_HIP][_Z_];			//
        													//	Right High leg
        end_segment[7][_X_]=link_pos[R_KNEE][_X_];			//
        end_segment[7][_Y_]=link_pos[R_KNEE][_Y_];			//
        end_segment[7][_Z_]=link_pos[R_KNEE][_Z_];			//


        start_segment[8][_X_]=link_pos[R_FD][_X_];			//
        start_segment[8][_Y_]=link_pos[R_FD][_Y_];			//
        start_segment[8][_Z_]=link_pos[R_FD][_Z_];			//
        													//	Right Low leg
        end_segment[8][_X_]=link_pos[R_KNEE][_X_];			//
        end_segment[8][_Y_]=link_pos[R_KNEE][_Y_];			//
        end_segment[8][_Z_]=link_pos[R_KNEE][_Z_];			//


        /* ----------- Right Foot ----------- */
        start_segment[9][_X_]= (link_pos[R_OUT_TOE][_X_] + link_pos[R_IN_TOE][_X_]) / 2;
        start_segment[9][_Y_]= (link_pos[R_OUT_TOE][_Y_] + link_pos[R_IN_TOE][_Y_]) / 2;
        start_segment[9][_Z_]= (link_pos[R_OUT_TOE][_Z_] + link_pos[R_IN_TOE][_Z_]) / 2;

        end_segment[9][_X_]= (link_pos[R_OUT_HEEL][_X_] + link_pos[R_IN_HEEL][_X_]) / 2;
        end_segment[9][_Y_]= (link_pos[R_OUT_HEEL][_Y_] + link_pos[R_IN_HEEL][_Y_]) / 2;
        end_segment[9][_Z_]= (link_pos[R_OUT_HEEL][_Z_] + link_pos[R_IN_HEEL][_Z_]) / 2;


        /* ----------- Left Foot ----------- */
        start_segment[10][_X_]= (link_pos[L_OUT_TOE][_X_] + link_pos[L_IN_TOE][_X_]) / 2;
        start_segment[10][_Y_]= (link_pos[L_OUT_TOE][_Y_] + link_pos[L_IN_TOE][_Y_]) / 2;
        start_segment[10][_Z_]= (link_pos[L_OUT_TOE][_Z_] + link_pos[L_IN_TOE][_Z_]) / 2;

        end_segment[10][_X_]= (link_pos[L_OUT_HEEL][_X_] + link_pos[L_IN_HEEL][_X_]) / 2;
        end_segment[10][_Y_]= (link_pos[L_OUT_HEEL][_Y_] + link_pos[L_IN_HEEL][_Y_]) / 2;
        end_segment[10][_Z_]= (link_pos[L_OUT_HEEL][_Z_] + link_pos[L_IN_HEEL][_Z_]) / 2;


        /* ----------- Body ----------- */
        start_segment[11][_X_]= (link_pos[L_SH][_X_] + link_pos[R_SH][_X_]) / 2;
        start_segment[11][_Y_]= (link_pos[L_SH][_Y_] + link_pos[R_SH][_Y_]) / 2;
        start_segment[11][_Z_]= (link_pos[L_SH][_Z_] + link_pos[R_SH][_Z_]) / 2;

        end_segment[11][_X_]= (link_pos[R_HIP][_X_] + link_pos[L_HIP][_X_]) / 2;
        end_segment[11][_Y_]= (link_pos[R_HIP][_Y_] + link_pos[L_HIP][_Y_]) / 2;
        end_segment[11][_Z_]= (link_pos[R_HIP][_Z_] + link_pos[L_HIP][_Z_]) / 2;


        /* ----------- Segment between upper back and lower back ----------- */
        start_segment[12][_X_]= Alink_back_low[1][4];
        start_segment[12][_Y_]= Alink_back_low[2][4];
        start_segment[12][_Z_]= Alink_back_low[3][4];

        end_segment[12][_X_]= Alink_back_up[1][4];
        end_segment[12][_Y_]= Alink_back_up[2][4];
        end_segment[12][_Z_]= Alink_back_up[3][4];

        /* ----------- Point of the HEAD ----------- */
        point_H_1[_X_]= link_pos[H_1][_X_];
        point_H_1[_Y_]= link_pos[H_1][_Y_];
        point_H_1[_Z_]= link_pos[H_1][_Z_];

        /* ----------- Point of the Left Shoulder ----------- */
        point_LA_J3[_X_]= link_pos[L_UA][_X_];
        point_LA_J3[_Y_]= link_pos[L_UA][_Y_];
        point_LA_J3[_Z_]= link_pos[L_UA][_Z_];

        /* ----------- Point of the Right Shoulder ----------- */
        point_RA_J3[_X_]= link_pos[R_UA][_X_];
        point_RA_J3[_Y_]= link_pos[R_UA][_Y_];
        point_RA_J3[_Z_]= link_pos[R_UA][_Z_];

        /* ----------- Point of the Left knee ----------- */
        point_LL_J4[_X_]= link_pos[L_KNEE][_X_];
        point_LL_J4[_Y_]= link_pos[L_KNEE][_Y_];
        point_LL_J4[_Z_]= link_pos[L_KNEE][_Z_];

        /* ----------- Point of the Right knee ----------- */
        point_RL_J4[_X_]= link_pos[R_KNEE][_X_];
        point_RL_J4[_Y_]= link_pos[R_KNEE][_Y_];
        point_RL_J4[_Z_]= link_pos[R_KNEE][_Z_];



    /* ----------- to get all the combinations  ----------- */
/*
-----------------------------k==R_FA----------------------------------------------------
k==R_FA && q==L_FA   ==> distance between Right Forearm and Left Forearm
k==R_FA && q==L_UARM   ==> distance between Right Forearm and Left Arm
k==R_FA && q==L_TL   ==> distance between Right Forearm and Left High Leg
k==R_FA && q==L_LL   ==> distance between Right Forearm and Left Low Leg
k==R_FA && q==R_TL   ==> distance between Right Forearm and Right High Leg
k==R_FA && q==R_LL   ==> distance between Right Forearm and Right Low Leg
k==R_FA && q==R_FOOT   ==> distance between Right Forearm and Right Foot
k==R_FA && q==L_FOOT  ==> distance between Right Forearm and Left Foot
k==R_FA && q==BODY  ==> distance between Right Forearm and Body
k==R_FA && q==BACK_PACK  ==> distance between Right Forearm and Backpack
k==R_FA ==> distance between Right Forearm and Head

------------------------------k==R_UARM----------------------------------------------------
k==R_UARM && q==L_FA   ==> distance between Right Arm and Left Forearm
k==R_UARM && q==BODY  ==> distance between Right Arm and Body
k==R_UARM ==> distance between Right Arm and Head

------------------------------k==L_FA----------------------------------------------------
k==L_FA && q==L_TL   ==> distance between Left Forearm and Left High Leg
k==L_FA && q==L_LL   ==> distance between Left Forearm and Left Low Leg
k==L_FA && q==R_TL   ==> distance between Left Forearm and Right High Leg
k==L_FA && q==R_LL   ==> distance between Left Forearm and Right Low Leg
k==L_FA && q==R_FOOT   ==> distance between Left Forearm and Right Foot
k==L_FA && q==L_FOOT  ==> distance between Left Forearm and Left Foot
k==L_FA && q==BODY  ==> distance between Left Forearm and Body
k==L_FA && q==BACK_PACK  ==> distance between Left Forearm and Backpack
k==L_FA ==> distance between Left Forearm and Head

------------------------------k==L_UARM----------------------------------------------------
k==L_UARM && q==BODY ==> distance between Left Arm and Body
k==L_UARM ==> distance between Left Arm and Head

------------------------------k==L_TL----------------------------------------------------
k==L_TL && q==R_TL   ==> distance between Left High Leg and Right High Leg
k==L_TL && q==R_LL   ==> distance between Left High Leg and Right Low Leg
k==L_TL && q==R_FOOT   ==> distance between Left High Leg and Right Foot

------------------------------k==L_LL----------------------------------------------------
k==L_LL && q==R_TL   ==> distance between Left Low Leg and Right High Leg
k==L_LL && q==R_LL   ==> distance between Left Low Leg and Right Low Leg
k==L_LL && q==R_FOOT   ==> distance between Left Low Leg and Right Foot
k==L_LL && q==BACK_PACK   ==> distance between Left Low Leg and Backpack

------------------------------k==R_LL----------------------------------------------------
k==R_LL && q==L_FOOT  ==> distance between Right Low Leg and Left Foot
k==R_LL && q==BACK_PACK  ==> distance between Right Low Leg and Backpack

------------------------------k==R_FOOT----------------------------------------------------
k==R_FOOT && q==L_FOOT  ==> distance between Right Foot and Left Foot
k==R_FOOT && q==BACK_PACK  ==> distance between Right Foot and Backpack

------------------------------k==L_FOOT----------------------------------------------------
k==L_FOOT && q==BACK_PACK  ==> distance between Left Foot and Backpack

1: Right Forearm
2: Right Arm
3: Left Forearm
4: Left Arm
5: Left High Leg
6: Left Low Leg
7: Right High Leg
8: Right Low Leg
9: Right Foot
10: Left Foot
11: Body
12: Segment between upper back and lower back
point_H_1: Head
*/

#define R_FA 		1	//Right Forearm
#define R_UARM 		2	//Right Upper Arm
#define L_FA 		3	//Left Forearm
#define L_UARM 		4	//Left Upper Arm
#define L_TL 		5	//Left Thigh Leg
#define L_LL 		6	//Left Lower Leg
#define R_TL 		7	//Right Thigh Leg
#define R_LL 		8	//Right Lower Leg
#define R_FOOT 		9	//Right Foot
#define L_FOOT 		10	//Left Foot
#define BODY 		11	//Body
#define BACK_PACK 	12	//Back

#define N_ACTIVE_SEG 10 //segments which avoid
#define N_SEG 12 		//All the segments used

    for (int k=R_FA; k <= N_ACTIVE_SEG ; k++)
    {
    	shortest_dist_segment[k] = INFINITY;
    	dist=INFINITY;
    	for (int q = R_FA; q <= N_SEG ; q ++)
    	{
    		//calculate distance between body segment pairs, for which a collision is possible. see also table above
    		if (   	(k==R_FA 	&& 	q==L_FA)  		||  	(k==L_FA  		&& 		q==R_FA) 		||
    				(k==R_FA 	&& 	q==L_UARM) 		||		(k==L_UARM  	&& 		q==R_FA) 		||
    				(k==R_FA 	&& 	q==L_TL)  		||		(k==L_TL  		&& 		q==R_FA) 		||
    				(k==R_FA 	&& 	q==L_LL)  		||		(k==L_LL  		&& 		q==R_FA) 		||
    				(k==R_FA 	&& 	q==R_TL)  		||		(k==R_TL  		&& 		q==R_FA)		||
    				(k==R_FA 	&& 	q==R_LL)  		||		(k==R_LL  		&& 		q==R_FA)		||
    				(k==R_FA 	&& 	q==R_FOOT) 		||		(k==R_FOOT  	&& 		q==R_FA)		||
    				(k==R_FA 	&& 	q==L_FOOT) 		||		(k==L_FOOT 		&& 		q==R_FA)		||
    				(k==R_FA 	&& 	q==BODY) 		||		(k==BODY 		&& 		q==R_FA)		||
    				(k==R_FA 	&& 	q==BACK_PACK)	||
    				(k==R_UARM 	&& 	q==L_FA)  		||		(k==L_FA  		&& 		q==R_UARM)		||
    				(k==L_FA 	&& 	q==L_TL)  		||		(k==L_TL  		&& 		q==L_FA)		||
    				(k==L_FA 	&& 	q==L_LL)  		||		(k==L_LL  		&& 		q==L_FA)		||
    				(k==L_FA 	&& 	q==R_TL)  		||		(k==R_TL 		&& 		q==L_FA)		||
    				(k==L_FA 	&& 	q==R_LL)  		||		(k==R_LL  		&& 		q==L_FA)		||
    				(k==L_FA 	&& 	q==R_FOOT)  	||		(k==R_FOOT  	&& 		q==L_FA)		||
    				(k==L_FA 	&& 	q==L_FOOT) 		||		(k==L_FOOT 		&& 		q==L_FA)		||
    				(k==L_FA 	&& 	q==BODY) 		||		(k==BODY 		&& 		q==L_FA)		||
    				(k==L_FA 	&& 	q==BACK_PACK)	||
    				(k==L_TL 	&& 	q==R_TL) 		||		(k==R_TL  		&& 		q==L_TL)		||
    				(k==L_TL 	&& 	q==R_LL) 		||		(k==R_LL  		&& 		q==L_TL)		||
    				(k==L_TL 	&& 	q==R_FOOT) 		||		(k==R_FOOT  	&& 		q==L_TL)		||
    				(k==L_LL 	&& 	q==R_TL)  		||		(k==R_TL  		&& 		q==L_LL)		||
    				(k==L_LL 	&& 	q==R_LL)  		||		(k==R_LL  		&& 		q==L_LL)		||
    				(k==L_LL 	&& 	q==R_FOOT)  	||		(k==R_FOOT  	&& 		q==L_LL)		||
    				(k==L_LL 	&& 	q==BACK_PACK)	||
    				(k==R_LL 	&& 	q==L_FOOT) 		||		(k==L_FOOT 		&& 		q==R_LL)		||
    				(k==R_LL 	&& 	q==BACK_PACK)	||
    				(k==R_FOOT 	&& 	q==L_FOOT) 		||		(k==L_FOOT 		&& 		q==R_FOOT)		||
    				(k==R_FOOT 	&& 	q==BACK_PACK)	||
    				(k==L_FOOT 	&& 	q==BACK_PACK)
    		)
    		{
    			dist = dist3D_segment_to_segment(  start_segment[k],  end_segment[k],  start_segment[q],  end_segment[q], av_temp, critic_point_temp);

    			//For the feet, backpack is a cylinder with 5cm width
    			if ((k==BACK_PACK && q==L_FOOT) || (k==BACK_PACK && q==R_FOOT) || (k==L_FOOT && q==BACK_PACK) || (k==R_FOOT && q==BACK_PACK)){
    				dist -=0.05;
    			}
    		}

    		//when comparing body with something, substract 10 cm to model it as a cylinder. When comparing it with upper arm,
    		//use point_to_segment function, comparing just the distance body-elbow.
    		if (k==BODY || q==BODY )
    		{
    			if (k==L_UARM && q==BODY)
    			{
    				dist= dist_Point_to_Segment( start_segment[q], end_segment[q], point_LA_J3, av_temp, critic_point_temp );

    				av_temp[_X_]= -av_temp[_X_];
    				av_temp[_Y_]= -av_temp[_Y_];
    				av_temp[_Z_]= -av_temp[_Z_];

    				critic_point_temp[_X_]= point_LA_J3[_X_];
    				critic_point_temp[_Y_]= point_LA_J3[_Y_];
    				critic_point_temp[_Z_]= point_LA_J3[_Z_];

    			}
    			else if (k==BODY && q==L_UARM)
    			{
    				dist= dist_Point_to_Segment(start_segment[k], end_segment[k], point_LA_J3, av_temp, critic_point_temp );

    			}
    			else if (k==R_UARM && q==BODY)
    			{
    				dist= dist_Point_to_Segment( start_segment[q], end_segment[q], point_RA_J3, av_temp, critic_point_temp );

    				av_temp[_X_]= -av_temp[_X_];
    				av_temp[_Y_]= -av_temp[_Y_];
    				av_temp[_Z_]= -av_temp[_Z_];

    				critic_point_temp[_X_]= point_RA_J3[_X_];
    				critic_point_temp[_Y_]= point_RA_J3[_Y_];
    				critic_point_temp[_Z_]= point_RA_J3[_Z_];

    			}
    			else if (k==BODY && q==R_UARM)
    			{
    				dist= dist_Point_to_Segment(start_segment[k], end_segment[k], point_RA_J3, av_temp, critic_point_temp );
    			}

    			//For the arms, body is a cylinder with 10cm width. for the legs, this is skipped, as we just consider it a line segment.
    			if (k==R_FA || k==R_UARM || k==L_FA || k==L_UARM || q==R_FA || q==R_UARM || q==L_FA || q==L_UARM )
    				{
    				dist -= 0.15;
    				}

    		}

    		//when comparing thighs to each other, use point (knee) to segment (thigh)
    		if ((k == L_TL && q == R_TL) || (q == L_TL && k == R_TL)){
    			if (k == L_TL && q == R_TL){
    				dist = dist_Point_to_Segment(start_segment[q], end_segment[q], point_LL_J4, av_temp, critic_point_temp );
    				av_temp[_X_]= -av_temp[_X_];
    				av_temp[_Y_]= -av_temp[_Y_];
    				av_temp[_Z_]= -av_temp[_Z_];
    				critic_point_temp[_X_]= point_LL_J4[_X_];
    				critic_point_temp[_Y_]= point_LL_J4[_Y_];
    				critic_point_temp[_Z_]= point_LL_J4[_Z_];
    			}
    			else if (q == L_TL && k == R_TL){
    				dist = dist_Point_to_Segment(start_segment[q], end_segment[q], point_RL_J4, av_temp, critic_point_temp );
    				av_temp[_X_]= -av_temp[_X_];
    				av_temp[_Y_]= -av_temp[_Y_];
    				av_temp[_Z_]= -av_temp[_Z_];
    				critic_point_temp[_X_]= point_RL_J4[_X_];
    				critic_point_temp[_Y_]= point_RL_J4[_Y_];
    				critic_point_temp[_Z_]= point_RL_J4[_Z_];
    			}
    		}

    		if (shortest_dist_segment[k] > dist)	//to get THE shortest distance of all the segments
    		{
				shortest_dist_segment[k] = dist;

				avoidance_vector_segment[_X_]= av_temp[_X_];
				avoidance_vector_segment[_Y_]= av_temp[_Y_];
				avoidance_vector_segment[_Z_]= av_temp[_Z_];

				critical_point_segment[_X_]= critic_point_temp[_X_];
				critical_point_segment[_Y_]= critic_point_temp[_Y_];
				critical_point_segment[_Z_]= critic_point_temp[_Z_];
    		}


    	}

    	/* ----------- Shortest distance to the head ----------- */
    	shortest_dist_head[k] = INFINITY;
    	dist_head=INFINITY;

    	if ((k==R_FA) || (k==L_FA) || (k==R_UARM) || (k==L_UARM)) 	//Only for the arms
    	{
    		if ((k==R_FA) || (k==L_FA))
    			dist_head= dist_Point_to_Segment( start_segment[k], end_segment[k], point_H_1, av_temp_head, critic_point_temp_head);
    		else if (k==R_UARM)
    		{
    			dist_head= dist_point_to_point(point_H_1, point_RA_J3, av_temp_head);
    			av_temp_head[_X_]= -av_temp_head[_X_];
    			av_temp_head[_Y_]= -av_temp_head[_Y_];
    			av_temp_head[_Z_]= -av_temp_head[_Z_];
    		}
    		else if (k==L_UARM)
    		{
    			dist_head= dist_point_to_point(point_H_1, point_LA_J3, av_temp_head);
    			av_temp_head[_X_]= -av_temp_head[_X_];
    			av_temp_head[_Y_]= -av_temp_head[_Y_];
    			av_temp_head[_Z_]= -av_temp_head[_Z_];
    		}
    		dist_head -= 0.075;	//head is a sphere with 10cm radius

    		if (shortest_dist_head[k] > dist_head)		//to get THE shortest distance of all the segments which can touch the head
    		{
    			shortest_dist_head[k] = dist_head;

    			avoidance_vector_head[_X_]= av_temp_head[_X_];
    			avoidance_vector_head[_Y_]= av_temp_head[_Y_];
    			avoidance_vector_head[_Z_]= av_temp_head[_Z_];

    			critical_point_head[_X_]= critic_point_temp_head[_X_];
    			critical_point_head[_Y_]= critic_point_temp_head[_Y_];
    			critical_point_head[_Z_]= critic_point_temp_head[_Z_];


    		}
    	 }

    	if (shortest_dist_head[k] > shortest_dist_segment[k] )
    		{
    			shortest_dist[k] = shortest_dist_segment[k];

        		/* ----------- Direction of the vector ----------- */
    			avoidance_vector[k][_X_] = avoidance_vector_segment[_X_];
    			avoidance_vector[k][_Y_] = avoidance_vector_segment[_Y_];
    			avoidance_vector[k][_Z_] = avoidance_vector_segment[_Z_];

    			critical_point[k][_X_]= critical_point_segment[_X_];
    			critical_point[k][_Y_]= critical_point_segment[_Y_];
    			critical_point[k][_Z_]= critical_point_segment[_Z_];
    		}
    	else
    		{
    			shortest_dist[k] = shortest_dist_head[k];

        		/* ----------- Direction of the vector ----------- */
    			avoidance_vector[k][_X_] = avoidance_vector_head[_X_];
    			avoidance_vector[k][_Y_] = avoidance_vector_head[_Y_];
    			avoidance_vector[k][_Z_] = avoidance_vector_head[_Z_];

    			critical_point[k][_X_]= critical_point_head[_X_];
    			critical_point[k][_Y_]= critical_point_head[_Y_];
    			critical_point[k][_Z_]= critical_point_head[_Z_];
    		}

    }

    int i;
    for (i=L_TL; i<=L_FOOT; ++i)
    	printf("k=%d,  short.dist=%f\n",i,shortest_dist[i]);

    for(int k=R_FA; k<= N_SEG; k++)		// Shortest distance is never negative
    	{
    	    	if (shortest_dist[k]<0)
    				shortest_dist[k]=0;
    	}

    //Call the function to have all the proximity factors
    get_all_proximity_factors();

    return TRUE;
}

/* ----------- Calculation of the distance between each segments which can cross.
						Get the 3D minimum distance between 2 segments ----------- */
#define DOT(u,v) 		((u)[1] * (v)[1] + (u)[2] * (v)[2] + (u)[3] * (v)[3])
#define SMALL_NUM 		0.00000001
#define MY_ABS(x)   	((x) >= 0 ? (x) : -(x))   //  absolute value
#define NORM(v)			sqrt(DOT(v,v)) 			  // distance
//#define MY_DIST(u,v)	NORM(u - v)

/* ----------- Input:  two 3D line segments vector1 and vector2
  	  	  	Return: the shortest distance between segment vector1 and vector2 ----------- */

double avoidanceTask::dist3D_segment_to_segment(double start_segment1[3+1], double end_segment1[3+1], double start_segment2[3+1], double end_segment2[3+1], double avoidance_vector_segment[3+1], double critical_point_segment[3+1])
{
	//this function has been modified from http://geomalgorithms.com/a07-_distance.html

	double start_segment[N_LINKS+1][3+1];
	double end_segment[N_LINKS+1][3+1];

	/* ----------- Making segments ----------- */

	double vector1[3+1];
 	 	 	 vector1[_X_]= end_segment1[_X_] - start_segment1[_X_];
	 	 	 vector1[_Y_]= end_segment1[_Y_] - start_segment1[_Y_];
	 	 	 vector1[_Z_]= end_segment1[_Z_] - start_segment1[_Z_];

	double vector2[3+1];
 	 	 	 vector2[_X_]= end_segment2[_X_] - start_segment2[_X_];
 	 	 	 vector2[_Y_]= end_segment2[_Y_] - start_segment2[_Y_];
 	 	 	 vector2[_Z_]= end_segment2[_Z_] - start_segment2[_Z_];

 	double vector_w[3+1];
 	 	 	 vector_w[_X_]= start_segment1[_X_] - start_segment2[_X_];
   	  	 	 vector_w[_Y_]= start_segment1[_Y_] - start_segment2[_Y_];
   	  	 	 vector_w[_Z_]= start_segment1[_Z_] - start_segment2[_Z_];

   	double a= DOT (vector1, vector1);
   	double b= DOT (vector1, vector2);
   	double c= DOT (vector2, vector2);
   	double d= DOT (vector1, vector_w);
   	double e= DOT (vector2, vector_w);
   	double D= a*c - b*b;
   	double sc, sN, sD = D;
   	double tc, tN, tD = D;
   	float P;

if (D < SMALL_NUM)
	 {
		sN = 0.0;         // force using point P0 on segment S1
		sD = 1.0;         // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	 }
else
	{
		sN = (b*e - c*d);
	    tN = (a*e - b*d);
	    	if (sN < 0.0)        // sc < 0 => the s=0 edge is visible
	            {
	        		sN = 0.0;
	        		tN = e;
	        		tD = c;
	            }
	        else if (sN > sD)   // sc > 1  => the s=1 edge is visible
	            {	sN = sD;
	            	tN = e + b;
	            	tD = c;
	            }
	}

if (tN < 0.0)             // tc < 0 => the t=0 edge is visible
    {
		tN = 0.0;
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else
        {
			sN = -d;
			sD = a;
        }
    }

else if (tN > tD)       // tc > 1  => the t=1 edge is visible
    {
		tN = tD;
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else
        {
			sN = (-d + b);
			sD = a;
        }
    }

/* ----------- finally do the division to get sc and tc  ----------- */
    sc = (MY_ABS(sN) < SMALL_NUM ? 0.0 : sN / sD);
    tc = (MY_ABS(tN) < SMALL_NUM ? 0.0 : tN / tD);

    		/* ----------- get the difference of the two closest points  ----------- */
    double vector_dP[3+1];
    	   vector_dP[_X_] = vector_w[_X_] + (sc * vector1[_X_]) - (tc * vector2[_X_]);
    	   vector_dP[_Y_] = vector_w[_Y_] + (sc * vector1[_Y_]) - (tc * vector2[_Y_]);
    	   vector_dP[_Z_] = vector_w[_Z_] + (sc * vector1[_Z_]) - (tc * vector2[_Z_]);

    		/* ----------- Direction of the vector ----------- */
    	   avoidance_vector_segment[_X_]= vector_dP[_X_] / NORM(vector_dP);
    	   avoidance_vector_segment[_Y_]= vector_dP[_Y_] / NORM(vector_dP);
    	   avoidance_vector_segment[_Z_]= vector_dP[_Z_] / NORM(vector_dP);

        	/* ----------- Critical Point Segment----------- */
    	   critical_point_segment[_X_]= start_segment1[_X_]+(sc * vector1[_X_]);
    	   critical_point_segment[_Y_]= start_segment1[_Y_]+(sc * vector1[_Y_]);
    	   critical_point_segment[_Z_]= start_segment1[_Z_]+(sc * vector1[_Z_]);

    return NORM(vector_dP); // distance
}
/* ----------- get the 3D minimum distance between 2 lines
 	 	 	 Input:  two 3D line vector1 and vector2
  	  	Return: the shortest distance between line vector1 and vector2 ----------- */

double avoidanceTask::dist3D_line_to_line(double start_segment1[3+1], double end_segment1[3+1], double start_segment2[3+1], double end_segment2[3+1], double avoidance_vector_line[3+1])
{
	double start_segment[N_LINKS+1][3+1];
	double end_segment[N_LINKS+1][3+1];

	double vector1[3+1];
 	 	 	 vector1[_X_]=end_segment1[_X_] - start_segment1[_X_] ;
	 	 	 vector1[_Y_]=end_segment1[_Y_] - start_segment1[_Y_];
	 	 	 vector1[_Z_]=end_segment1[_Z_] - start_segment1[_Z_];

	double vector2[3+1];
 	 	 	 vector2[_X_]=end_segment2[_X_] - start_segment2[_X_];
 	 	 	 vector2[_Y_]=end_segment2[_Y_] - start_segment2[_Y_];
 	 	 	 vector2[_Z_]=end_segment2[_Z_] - start_segment2[_Z_];

 	double vector_w[3+1];
  	  	 	 vector_w[_X_]= start_segment1[_X_] - start_segment2[_X_];
  	  	 	 vector_w[_Y_]= start_segment1[_Y_] - start_segment2[_Y_];
  	  	 	 vector_w[_Z_]= start_segment1[_Z_] - start_segment2[_Z_];

  	double a= DOT (vector1, vector1);
  	double b= DOT (vector1, vector2);
  	double c= DOT (vector2, vector2);
  	double d= DOT (vector1, vector_w);
  	double e= DOT (vector2, vector_w);
  	double D= a*c - b*b;
  	double sc,tc;

  	sc = (b*e - c*d) / D;
  	tc = (a*e - b*d) / D;

  	double vector_dP[3+1];
  		   vector_dP[_X_] = vector_w[_X_] + (sc * vector1[_X_]) - (tc * vector2[_X_]);
  		   vector_dP[_Y_] = vector_w[_Y_] + (sc * vector1[_Y_]) - (tc * vector2[_Y_]);
  		   vector_dP[_Z_] = vector_w[_Z_] + (sc * vector1[_Z_]) - (tc * vector2[_Z_]);

  		/* ----------- Direction of the vector ----------- */
  		   avoidance_vector_line[_X_]= vector_dP[_X_] / NORM(vector_dP);
  		   avoidance_vector_line[_Y_]= vector_dP[_Y_] / NORM(vector_dP);
  		   avoidance_vector_line[_Z_]= vector_dP[_Z_] / NORM(vector_dP);

  	return NORM(vector_dP); // distance
}

/* ----------- get the 3D minimum distance between 2 points
 	 	 	 Input:  two points : point_1 and point_2
  	  	Return: the shortest distance between point_1 and point_2  ----------- */

double avoidanceTask::dist_point_to_point(double point_1[3+1], double point_2[3+1], double avoidance_vector_point[3+1])
{
	double distance_point_to_point= sqrt((point_2[_X_] - point_1[_X_])*(point_2[_X_] - point_1[_X_]) + (point_2[_Y_] - point_1[_Y_])*(point_2[_Y_] - point_1[_Y_]) + (point_2[_Z_] - point_1[_Z_])*(point_2[_Z_] - point_1[_Z_]));

	double vector_between_2_points[3+1];
		   vector_between_2_points[_X_]= point_1[_X_] - point_2[_X_];
		   vector_between_2_points[_Y_]= point_1[_Y_] - point_2[_Y_];
		   vector_between_2_points[_Z_]= point_1[_Z_] - point_2[_Z_];

		   /* ----------- Direction of the vector ----------- */
	       avoidance_vector_point[_X_]= vector_between_2_points[_X_] / NORM(vector_between_2_points);
	       avoidance_vector_point[_Y_]= vector_between_2_points[_Y_] / NORM(vector_between_2_points);
	       avoidance_vector_point[_Z_]= vector_between_2_points[_Z_] / NORM(vector_between_2_points);

	       //avoidance vector is pointing away from point 2

return distance_point_to_point;
}

/* ----------- get the distance of a point to a segment
       Input:  a Point point_H_1 and a Segment vector1 (in any dimension)
    Return: the shortest distance from point_H_1 to vector1 ----------- */

double avoidanceTask::dist_Point_to_Segment( double start_segment1[3+1], double end_segment1[3+1], double point1[3+1], double avoidance_from_point[3+1], double critical_point_head[3+1])
{
	double start_segment[N_LINKS+1][3+1];
	double end_segment[N_LINKS+1][3+1];

	double vector1[3+1];
	 	   vector1[1]=end_segment1[_X_] - start_segment1[_X_];
		   vector1[2]=end_segment1[_Y_] - start_segment1[_Y_];
		   vector1[3]=end_segment1[_Z_] - start_segment1[_Z_];

	double vector_z[3+1];
		   vector_z[_X_]= point1[_X_] - start_segment1[_X_];
		   vector_z[_Y_]= point1[_Y_] - start_segment1[_Y_];
		   vector_z[_Z_]= point1[_Z_] - start_segment1[_Z_];

     double c1 = DOT(vector_z, vector1);
     if (c1 <= 0)
		{
			double vector_m[3 + 1];
				   vector_m[_X_] = start_segment1[_X_] - point1[_X_];
				   vector_m[_Y_] = start_segment1[_Y_] - point1[_Y_];
				   vector_m[_Z_] = start_segment1[_Z_] - point1[_Z_];

				   /* ----------- Direction of the vector ----------- */
				   avoidance_from_point[_X_] = vector_m[_X_] / NORM(vector_m);
				   avoidance_from_point[_Y_] = vector_m[_Y_] / NORM(vector_m);
				   avoidance_from_point[_Z_] = vector_m[_Z_] / NORM(vector_m);

				   /* ----------- Critical Point Head ----------- */
				   critical_point_head[_X_]= start_segment1[_X_];
				   critical_point_head[_Y_]= start_segment1[_Y_];
				   critical_point_head[_Z_]= start_segment1[_Z_];

    	 	return NORM(vector_m); // distance
    	}

     double c2 = DOT(vector1,vector1);

     if ( c2 <= c1 )
          {
    	 	 	 double vector_n[3+1];
    	 	 	 	 vector_n[_X_] = end_segment1[_X_] - point1[_X_];
    	 	 	 	 vector_n[_Y_] = end_segment1[_Y_] - point1[_Y_];
    	 	 	 	 vector_n[_Z_] = end_segment1[_Z_] - point1[_Z_];

    	 	 	 	/* ----------- Direction of the vector ----------- */
    	     	  	 avoidance_from_point[_X_] = vector_n[_X_] / NORM(vector_n);
    	     	  	 avoidance_from_point[_Y_] = vector_n[_Y_] / NORM(vector_n);
    	     	  	 avoidance_from_point[_Z_] = vector_n[_Z_] / NORM(vector_n);

    	          	/* ----------- Critical Point Head ----------- */
    	     	  	critical_point_head[_X_]= end_segment1[_X_];
    	     	  	critical_point_head[_Y_]= end_segment1[_Y_];
    	     	  	critical_point_head[_Z_]= end_segment1[_Z_];

    	 	 	 return NORM(vector_n); // distance
          }

     double b = c1 / c2;
     double Pb[3+1];
     	    Pb[_X_] = start_segment1[_X_] + b * vector1[_X_];
     	    Pb[_Y_] = start_segment1[_Y_] + b * vector1[_Y_];
     	    Pb[_Z_] = start_segment1[_Z_] + b * vector1[_Z_];

     double vector_s[3+1];
         	vector_s[_X_]= Pb[_X_] - point1[_X_];
         	vector_s[_Y_]= Pb[_Y_] - point1[_Y_];
         	vector_s[_Z_]= Pb[_Z_] - point1[_Z_];

         	/* ----------- Direction of the vector ----------- */
         	avoidance_from_point[_X_]= vector_s[_X_] / NORM(vector_s);
         	avoidance_from_point[_Y_]= vector_s[_Y_] / NORM(vector_s);
         	avoidance_from_point[_Z_]= vector_s[_Z_] / NORM(vector_s);

         	/* ----------- Critical Point Head ----------- */
         	critical_point_head[_X_]= Pb[_X_];
         	critical_point_head[_Y_]= Pb[_Y_];
         	critical_point_head[_Z_]= Pb[_Z_];

         	return NORM(vector_s); // distance
}

double avoidanceTask::get_all_proximity_factors()
{
	double dist_dangerous[12];
	dist_dangerous[1]= 0.02;
	dist_dangerous[2]= 0.02;
	dist_dangerous[3]= 0.02;
	dist_dangerous[4]= 0.02;
	dist_dangerous[5]= 0.07;
	dist_dangerous[6]= 0.07;
	dist_dangerous[7]= 0.07;
	dist_dangerous[8]= 0.07;
	dist_dangerous[9]= 0.07;
	dist_dangerous[10]= 0.07;

	/*---- Caclul of the Proximity_Factor ----*/
	for (int i=1; i<=10; i++)
	{
		proximity_factor[i]= exp(-70*(shortest_dist[i]-dist_dangerous[i]));

		printf("i=%d, sh.dist=%f, prox.f. =%f\n",i, shortest_dist[i], proximity_factor[i]);

		// 0.001 < proximity_factor < 1
		if (proximity_factor[i] > 1)
			proximity_factor[i] = 1;
		else if (proximity_factor[i] < 0.001)
			proximity_factor[i] = 0;
	}
	return TRUE;
}

double avoidanceTask::get_all_jacobian_coordinates()
{
	/*---- J ----*/
	double J1[3+1];
	double J2[3+1];
	double J3[3+1];
	double J4[3+1];
	double J5[3+1];
	double J6[3+1];
	/*---- X ----*/
	double X1[3+1];
	double X2[3+1];
	double X3[3+1];
	double X4[3+1];
	double X5[3+1];
	double X6[3+1];
	/*---- Z ----*/
    double Z1[3+1];
    double Z2[3+1];
    double Z3[3+1];
    double Z4[3+1];
    double Z5[3+1];
    double Z6[3+1];

	/* ----------- Left arm ----------- */
	if (proximity_factor[L_FA] < proximity_factor[L_UARM])
	{
		proximity_factor_LA = proximity_factor[L_UARM];
		avoidance_vector_LA[_X_]= avoidance_vector[L_UARM][_X_];
		avoidance_vector_LA[_Y_]= avoidance_vector[L_UARM][_Y_];
		avoidance_vector_LA[_Z_]= avoidance_vector[L_UARM][_Z_];

		critical_point_LA[_X_]= critical_point[L_UARM][_X_];
		critical_point_LA[_Y_]= critical_point[L_UARM][_Y_];
		critical_point_LA[_Z_]= critical_point[L_UARM][_Z_];
	}

	else
	{
		proximity_factor_LA = proximity_factor[L_FA];
		avoidance_vector_LA[_X_]= avoidance_vector[L_FA][_X_];
		avoidance_vector_LA[_Y_]= avoidance_vector[L_FA][_Y_];
		avoidance_vector_LA[_Z_]= avoidance_vector[L_FA][_Z_];

		critical_point_LA[_X_]= critical_point[L_FA][_X_];
		critical_point_LA[_Y_]= critical_point[L_FA][_Y_];
		critical_point_LA[_Z_]= critical_point[L_FA][_Z_];
	}

	/* ----------- Right arm ----------- */
	if (proximity_factor[R_FA] < proximity_factor[R_UARM])
	{
		proximity_factor_RA = proximity_factor[R_UARM];
		avoidance_vector_RA[_X_]= avoidance_vector[R_UARM][_X_];
		avoidance_vector_RA[_Y_]= avoidance_vector[R_UARM][_Y_];
		avoidance_vector_RA[_Z_]= avoidance_vector[R_UARM][_Z_];

		critical_point_RA[_X_]= critical_point[R_UARM][_X_];
		critical_point_RA[_Y_]= critical_point[R_UARM][_Y_];
		critical_point_RA[_Z_]= critical_point[R_UARM][_Z_];
	}

	else
	{
		proximity_factor_RA = proximity_factor[R_FA];
		avoidance_vector_RA[_X_]= avoidance_vector[R_FA][_X_];
		avoidance_vector_RA[_Y_]= avoidance_vector[R_FA][_Y_];
		avoidance_vector_RA[_Z_]= avoidance_vector[R_FA][_Z_];

		critical_point_RA[_X_]= critical_point[R_FA][_X_];
		critical_point_RA[_Y_]= critical_point[R_FA][_Y_];
		critical_point_RA[_Z_]= critical_point[R_FA][_Z_];
	}

	/* ----------- Right Leg ----------- */
	if ((proximity_factor[R_LL] < proximity_factor[R_TL]) && (proximity_factor[R_FOOT] < proximity_factor[R_TL]))
	{
		proximity_factor_RL = proximity_factor[R_TL];
		avoidance_vector_RL[_X_]= avoidance_vector[R_TL][_X_];
		avoidance_vector_RL[_Y_]= avoidance_vector[R_TL][_Y_];
		avoidance_vector_RL[_Z_]= avoidance_vector[R_TL][_Z_];

		critical_point_RL[_X_]= critical_point[R_TL][_X_];
		critical_point_RL[_Y_]= critical_point[R_TL][_Y_];
		critical_point_RL[_Z_]= critical_point[R_TL][_Z_];
	}

	else if ((proximity_factor[R_LL] > proximity_factor[R_TL]) && (proximity_factor[R_FOOT] < proximity_factor[R_LL]))
	{
		proximity_factor_RL = proximity_factor[R_LL];
		avoidance_vector_RL[_X_]= avoidance_vector[R_LL][_X_];
		avoidance_vector_RL[_Y_]= avoidance_vector[R_LL][_Y_];
		avoidance_vector_RL[_Z_]= avoidance_vector[R_LL][_Z_];

		critical_point_RL[_X_]= critical_point[R_LL][_X_];
		critical_point_RL[_Y_]= critical_point[R_LL][_Y_];
		critical_point_RL[_Z_]= critical_point[R_LL][_Z_];
	}

	else
	{
		proximity_factor_RL = proximity_factor[R_FOOT];
		avoidance_vector_RL[_X_]= avoidance_vector[R_FOOT][_X_];
		avoidance_vector_RL[_Y_]= avoidance_vector[R_FOOT][_Y_];
		avoidance_vector_RL[_Z_]= avoidance_vector[R_FOOT][_Z_];

		critical_point_RL[_X_]= critical_point[R_FOOT][_X_];
		critical_point_RL[_Y_]= critical_point[R_FOOT][_Y_];
		critical_point_RL[_Z_]= critical_point[R_FOOT][_Z_];
	}

		/* ----------- Left Leg ----------- */
	if ((proximity_factor[L_LL] < proximity_factor[L_TL]) && (proximity_factor[L_FOOT] < proximity_factor[L_TL]))
	{
		proximity_factor_LL = proximity_factor[L_TL];
		avoidance_vector_LL[_X_]= avoidance_vector[L_TL][_X_];
		avoidance_vector_LL[_Y_]= avoidance_vector[L_TL][_Y_];
		avoidance_vector_LL[_Z_]= avoidance_vector[L_TL][_Z_];

		critical_point_LL[_X_]= critical_point[L_TL][_X_];
		critical_point_LL[_Y_]= critical_point[L_TL][_Y_];
		critical_point_LL[_Z_]= critical_point[L_TL][_Z_];
	}

	else if ((proximity_factor[L_LL] > proximity_factor[L_TL]) && (proximity_factor[L_FOOT] < proximity_factor[L_LL]))
	{
		proximity_factor_LL = proximity_factor[L_LL];
		avoidance_vector_LL[_X_]= avoidance_vector[L_LL][_X_];
		avoidance_vector_LL[_Y_]= avoidance_vector[L_LL][_Y_];
		avoidance_vector_LL[_Z_]= avoidance_vector[L_LL][_Z_];

		critical_point_LL[_X_]= critical_point[L_LL][_X_];
		critical_point_LL[_Y_]= critical_point[L_LL][_Y_];
		critical_point_LL[_Z_]= critical_point[L_LL][_Z_];
	}

	else
	{
		proximity_factor_LL = proximity_factor[L_FOOT];
		avoidance_vector_LL[_X_]= avoidance_vector[L_FOOT][_X_];
		avoidance_vector_LL[_Y_]= avoidance_vector[L_FOOT][_Y_];
		avoidance_vector_LL[_Z_]= avoidance_vector[L_FOOT][_Z_];

		critical_point_LL[_X_]= critical_point[L_FOOT][_X_];
		critical_point_LL[_Y_]= critical_point[L_FOOT][_Y_];
		critical_point_LL[_Z_]= critical_point[L_FOOT][_Z_];
	}


	double myball[N_CART+1];

	    			    			myball[_X_]=critical_point_RL[_X_];
	    			    			myball[_Y_]=critical_point_RL[_Y_];
	    			    			myball[_Z_]=critical_point_RL[_Z_];

	    			    			sendUserGraphics((char *) "ball2", myball, (N_CART+1)*sizeof(double));

	    			    			double myball1[N_CART+1];

	    			    			    			myball1[_X_]=avoidance_vector_RL[_X_];
	    			    			    			myball1[_Y_]=avoidance_vector_RL[_Y_];
	    			    			    			myball1[_Z_]=avoidance_vector_RL[_Z_];

	    			    			    			sendUserGraphics((char *) "ball1", myball1, (N_CART+1)*sizeof(double));

			printf("prox.f. RL=%f\n",proximity_factor_RL);

							/* ----------- Left arm ----------- */
	/* ----------- x1 is a vector starting at joint1 and ending at critical point ----------- */

		   X1[_X_]= critical_point_LA[_X_] - joint_origin_pos[LA_J1][_X_];
		   X1[_Y_]= critical_point_LA[_Y_] - joint_origin_pos[LA_J1][_Y_];
		   X1[_Z_]= critical_point_LA[_Z_] - joint_origin_pos[LA_J1][_Z_];

		   X2[_X_]= critical_point_LA[_X_] - joint_origin_pos[LA_J2][_X_];
		   X2[_Y_]= critical_point_LA[_Y_] - joint_origin_pos[LA_J2][_Y_];
		   X2[_Z_]= critical_point_LA[_Z_] - joint_origin_pos[LA_J2][_Z_];

		   X3[_X_]= critical_point_LA[_X_] - joint_origin_pos[LA_J3][_X_];
		   X3[_Y_]= critical_point_LA[_Y_] - joint_origin_pos[LA_J3][_Y_];
		   X3[_Z_]= critical_point_LA[_Z_] - joint_origin_pos[LA_J3][_Z_];

		   X4[_X_]= critical_point_LA[_X_] - joint_origin_pos[LA_J4][_X_];
		   X4[_Y_]= critical_point_LA[_Y_] - joint_origin_pos[LA_J4][_Y_];
		   X4[_Z_]= critical_point_LA[_Z_] - joint_origin_pos[LA_J4][_Z_];

		   /* ----------- z1 is unit vector of joint 1 axis ----------- */
  		   Z1[_X_]= joint_axis_pos[LA_J1][_X_];
   		   Z1[_Y_]= joint_axis_pos[LA_J1][_Y_];
   		   Z1[_Z_]= joint_axis_pos[LA_J1][_Z_];

		   Z2[_X_]= joint_axis_pos[LA_J2][_X_];
		   Z2[_Y_]= joint_axis_pos[LA_J2][_Y_];
		   Z2[_Z_]= joint_axis_pos[LA_J2][_Z_];

		   Z3[_X_]= joint_axis_pos[LA_J3][_X_];
		   Z3[_Y_]= joint_axis_pos[LA_J3][_Y_];
		   Z3[_Z_]= joint_axis_pos[LA_J3][_Z_];

		   Z4[_X_]= joint_axis_pos[LA_J4][_X_];
		   Z4[_Y_]= joint_axis_pos[LA_J4][_Y_];
		   Z4[_Z_]= joint_axis_pos[LA_J4][_Z_];

		   /* ----------- Cross Product ----------- */
			crossProd (Z1, X1, J1);
			crossProd (Z2, X2, J2);
			crossProd (Z3, X3, J3);
			crossProd (Z4, X4, J4);

			/* ----------- Jacobian Matrix for Left Arm ----------- */
			J_LA[1][1]= J1[_X_];
			J_LA[1][2]= J2[_X_];
			J_LA[1][3]= J3[_X_];
			J_LA[1][4]= J4[_X_];

			J_LA[2][1]= J1[_Y_];
			J_LA[2][2]= J2[_Y_];
			J_LA[2][3]= J3[_Y_];
			J_LA[2][4]= J4[_Y_];

			J_LA[3][1]= J1[_Z_];
			J_LA[3][2]= J2[_Z_];
			J_LA[3][3]= J3[_Z_];
			J_LA[3][4]= J4[_Z_];

			J_LA[4][1]= Z1[_X_];
			J_LA[4][2]= Z2[_X_];
			J_LA[4][3]= Z3[_X_];
			J_LA[4][4]= Z4[_X_];

			J_LA[5][1]= Z1[_Y_];
			J_LA[5][2]= Z2[_Y_];
			J_LA[5][3]= Z3[_Y_];
			J_LA[5][4]= Z4[_Y_];

			J_LA[6][1]= Z1[_Z_];
			J_LA[6][2]= Z2[_Z_];
			J_LA[6][3]= Z3[_Z_];
			J_LA[6][4]= Z4[_Z_];

	if( proximity_factor[L_FA] < proximity_factor[L_UARM])
			{
		// if critical point is before joint_3 and joint_4, we don't use joint_3 and joint_4 to avoid
				J_LA[1][4]= 0;
				J_LA[2][4]= 0;
				J_LA[3][4]= 0;
				J_LA[4][4]= 0;
				J_LA[5][4]= 0;
				J_LA[6][4]= 0;

				J_LA[1][3]= 0;
				J_LA[2][3]= 0;
				J_LA[3][3]= 0;
				J_LA[4][3]= 0;
				J_LA[5][3]= 0;
				J_LA[6][3]= 0;
			}

							/* ----------- Right arm ----------- */
	/* ----------- x1 is a vector starting at joint1 and ending at critical point ----------- */
			   X1[_X_]= critical_point_RA[_X_] - joint_origin_pos[RA_J1][_X_];
			   X1[_Y_]= critical_point_RA[_Y_] - joint_origin_pos[RA_J1][_Y_];
			   X1[_Z_]= critical_point_RA[_Z_] - joint_origin_pos[RA_J1][_Z_];

			   X2[_X_]= critical_point_RA[_X_] - joint_origin_pos[RA_J2][_X_];
			   X2[_Y_]= critical_point_RA[_Y_] - joint_origin_pos[RA_J2][_Y_];
			   X2[_Z_]= critical_point_RA[_Z_] - joint_origin_pos[RA_J2][_Z_];

			   X3[_X_]= critical_point_RA[_X_] - joint_origin_pos[RA_J3][_X_];
			   X3[_Y_]= critical_point_RA[_Y_] - joint_origin_pos[RA_J3][_Y_];
			   X3[_Z_]= critical_point_RA[_Z_] - joint_origin_pos[RA_J3][_Z_];

			   X4[_X_]= critical_point_RA[_X_] - joint_origin_pos[RA_J4][_X_];
			   X4[_Y_]= critical_point_RA[_Y_] - joint_origin_pos[RA_J4][_Y_];
			   X4[_Z_]= critical_point_RA[_Z_] - joint_origin_pos[RA_J4][_Z_];

			   /* ----------- z1 is unit vector of joint 1 axis ----------- */
	  		   Z1[_X_]= joint_axis_pos[RA_J1][_X_];
	   		   Z1[_Y_]= joint_axis_pos[RA_J1][_Y_];
	   		   Z1[_Z_]= joint_axis_pos[RA_J1][_Z_];

			   Z2[_X_]= joint_axis_pos[RA_J2][_X_];
			   Z2[_Y_]= joint_axis_pos[RA_J2][_Y_];
			   Z2[_Z_]= joint_axis_pos[RA_J2][_Z_];

			   Z3[_X_]= joint_axis_pos[RA_J3][_X_];
			   Z3[_Y_]= joint_axis_pos[RA_J3][_Y_];
			   Z3[_Z_]= joint_axis_pos[RA_J3][_Z_];

			   Z4[_X_]= joint_axis_pos[RA_J4][_X_];
			   Z4[_Y_]= joint_axis_pos[RA_J4][_Y_];
			   Z4[_Z_]= joint_axis_pos[RA_J4][_Z_];

			   crossProd (Z1, X1, J1);
			   crossProd (Z2, X2, J2);
			   crossProd (Z3, X3, J3);
			   crossProd (Z4, X4, J4);

			   /* ----------- Jacobian Matrix for Right Arm ----------- */
			   	J_RA[1][1]= J1[_X_];
			   	J_RA[1][2]= J2[_X_];
			   	J_RA[1][3]= J3[_X_];
			   	J_RA[1][4]= J4[_X_];

			   	J_RA[2][1]= J1[_Y_];
			   	J_RA[2][2]= J2[_Y_];
			   	J_RA[2][3]= J3[_Y_];
			   	J_RA[2][4]= J4[_Y_];

			   	J_RA[3][1]= J1[_Z_];
			   	J_RA[3][2]= J2[_Z_];
			   	J_RA[3][3]= J3[_Z_];
			   	J_RA[3][4]= J4[_Z_];

			   	J_RA[4][1]= Z1[_X_];
			   	J_RA[4][2]= Z2[_X_];
			   	J_RA[4][3]= Z3[_X_];
			   	J_RA[4][4]= Z4[_X_];

			   	J_RA[5][1]= Z1[_Y_];
			   	J_RA[5][2]= Z2[_Y_];
			   	J_RA[5][3]= Z3[_Y_];
			   	J_RA[5][4]= Z4[_Y_];

			   	J_RA[6][1]= Z1[_Z_];
			   	J_RA[6][2]= Z2[_Z_];
			   	J_RA[6][3]= Z3[_Z_];
			   	J_RA[6][4]= Z4[_Z_];

		if( proximity_factor[R_FA] < proximity_factor[R_UARM])
				{
			// if critical point is before joint_3 and joint_4, we don't use joint_3 and joint_4 to avoid
					J_RA[1][4]= 0;
					J_RA[2][4]= 0;
					J_RA[3][4]= 0;
					J_RA[4][4]= 0;
					J_RA[5][4]= 0;
					J_RA[6][4]= 0;

					J_RA[1][3]= 0;
					J_RA[2][3]= 0;
					J_RA[3][3]= 0;
					J_RA[4][3]= 0;
					J_RA[5][3]= 0;
					J_RA[6][3]= 0;
				}

								/* ----------- Right Leg ----------- */
		/* ----------- x1 is a vector starting at joint1 and ending at critical point ----------- */
		X1[_X_]= critical_point_RL[_X_] - joint_origin_pos[RL_J1][_X_];
		X1[_Y_]= critical_point_RL[_Y_] - joint_origin_pos[RL_J1][_Y_];
		X1[_Z_]= critical_point_RL[_Z_] - joint_origin_pos[RL_J1][_Z_];

		X2[_X_]= critical_point_RL[_X_] - joint_origin_pos[RL_J2][_X_];
		X2[_Y_]= critical_point_RL[_Y_] - joint_origin_pos[RL_J2][_Y_];
		X2[_Z_]= critical_point_RL[_Z_] - joint_origin_pos[RL_J2][_Z_];

		X3[_X_]= critical_point_RL[_X_] - joint_origin_pos[RL_J3][_X_];
		X3[_Y_]= critical_point_RL[_Y_] - joint_origin_pos[RL_J3][_Y_];
		X3[_Z_]= critical_point_RL[_Z_] - joint_origin_pos[RL_J3][_Z_];

		X4[_X_]= critical_point_RL[_X_] - joint_origin_pos[RL_J4][_X_];
		X4[_Y_]= critical_point_RL[_Y_] - joint_origin_pos[RL_J4][_Y_];
		X4[_Z_]= critical_point_RL[_Z_] - joint_origin_pos[RL_J4][_Z_];

		X5[_X_]= critical_point_RL[_X_] - joint_origin_pos[RL_J5][_X_];
		X5[_Y_]= critical_point_RL[_Y_] - joint_origin_pos[RL_J5][_Y_];
		X5[_Z_]= critical_point_RL[_Z_] - joint_origin_pos[RL_J5][_Z_];

		X6[_X_]= critical_point_RL[_X_] - joint_origin_pos[RL_J6][_X_];
		X6[_Y_]= critical_point_RL[_Y_] - joint_origin_pos[RL_J6][_Y_];
		X6[_Z_]= critical_point_RL[_Z_] - joint_origin_pos[RL_J6][_Z_];

		/* ----------- z1 is unit vector of joint 1 axis ----------- */
		Z1[_X_]= joint_axis_pos[RL_J1][_X_];
		Z1[_Y_]= joint_axis_pos[RL_J1][_Y_];
		Z1[_Z_]= joint_axis_pos[RL_J1][_Z_];

		Z2[_X_]= joint_axis_pos[RL_J2][_X_];
		Z2[_Y_]= joint_axis_pos[RL_J2][_Y_];
		Z2[_Z_]= joint_axis_pos[RL_J2][_Z_];

		Z3[_X_]= joint_axis_pos[RL_J3][_X_];
		Z3[_Y_]= joint_axis_pos[RL_J3][_Y_];
		Z3[_Z_]= joint_axis_pos[RL_J3][_Z_];

		Z4[_X_]= joint_axis_pos[RL_J4][_X_];
		Z4[_Y_]= joint_axis_pos[RL_J4][_Y_];
		Z4[_Z_]= joint_axis_pos[RL_J4][_Z_];

		Z5[_X_]= joint_axis_pos[RL_J5][_X_];
		Z5[_Y_]= joint_axis_pos[RL_J5][_Y_];
		Z5[_Z_]= joint_axis_pos[RL_J5][_Z_];

		Z6[_X_]= joint_axis_pos[RL_J6][_X_];
		Z6[_Y_]= joint_axis_pos[RL_J6][_Y_];
		Z6[_Z_]= joint_axis_pos[RL_J6][_Z_];

		crossProd (Z1, X1, J1);
		crossProd (Z2, X2, J2);
		crossProd (Z3, X3, J3);
		crossProd (Z4, X4, J4);
		crossProd (Z5, X5, J5);
		crossProd (Z6, X6, J6);

		/* ----------- Jacobian Matrix for Right Leg ----------- */
		J_RL[1][1]= J1[_X_];
		J_RL[1][2]= J2[_X_];
		J_RL[1][3]= J3[_X_];
		J_RL[1][4]= J4[_X_];
		J_RL[1][5]= J5[_X_];
		J_RL[1][6]= J6[_X_];

		J_RL[2][1]= J1[_Y_];
		J_RL[2][2]= J2[_Y_];
		J_RL[2][3]= J3[_Y_];
		J_RL[2][4]= J4[_Y_];
		J_RL[2][5]= J5[_Y_];
		J_RL[2][6]= J6[_Y_];

		J_RL[3][1]= J1[_Z_];
		J_RL[3][2]= J2[_Z_];
		J_RL[3][3]= J3[_Z_];
		J_RL[3][4]= J4[_Z_];
		J_RL[3][5]= J5[_Z_];
		J_RL[3][6]= J6[_Z_];

		J_RL[4][1]= Z1[_X_];
		J_RL[4][2]= Z2[_X_];
		J_RL[4][3]= Z3[_X_];
		J_RL[4][4]= Z4[_X_];
		J_RL[4][5]= Z5[_X_];
		J_RL[4][6]= Z6[_X_];

		J_RL[5][1]= Z1[_Y_];
		J_RL[5][2]= Z2[_Y_];
		J_RL[5][3]= Z3[_Y_];
		J_RL[5][4]= Z4[_Y_];
		J_RL[5][5]= Z5[_Y_];
		J_RL[5][6]= Z6[_Y_];

		J_RL[6][1]= Z1[_Z_];
		J_RL[6][2]= Z2[_Z_];
		J_RL[6][3]= Z3[_Z_];
		J_RL[6][4]= Z4[_Z_];
		J_RL[6][5]= Z5[_Z_];
		J_RL[6][6]= Z6[_Z_];

		if( (proximity_factor[R_LL] > proximity_factor[R_TL]) && (proximity_factor[R_LL] > proximity_factor[R_FOOT]))
		{
			// if critical point is before joint_5 and joint_6, we don't use joint_5 and joint_6 to avoid
			J_RL[1][5]= 0;
			J_RL[2][5]= 0;
			J_RL[3][5]= 0;
			J_RL[4][5]= 0;
			J_RL[5][5]= 0;
			J_RL[6][5]= 0;

			J_RL[1][6]= 0;
			J_RL[2][6]= 0;
			J_RL[3][6]= 0;
			J_RL[4][6]= 0;
			J_RL[5][6]= 0;
			J_RL[6][6]= 0;
		}

		else if( (proximity_factor[R_LL] < proximity_factor[R_TL]) && (proximity_factor[R_TL] > proximity_factor[R_FOOT]))
		{
			// if critical point is before joint_4, joint_5 and joint_6, we don't use joint_4, joint_5 and joint_6 to avoid
			J_RL[1][4]= 0;
			J_RL[2][4]= 0;
			J_RL[3][4]= 0;
			J_RL[4][4]= 0;
			J_RL[5][4]= 0;
			J_RL[6][4]= 0;

			J_RL[1][5]= 0;
			J_RL[2][5]= 0;
			J_RL[3][5]= 0;
			J_RL[4][5]= 0;
			J_RL[5][5]= 0;
			J_RL[6][5]= 0;

			J_RL[1][6]= 0;
			J_RL[2][6]= 0;
			J_RL[3][6]= 0;
			J_RL[4][6]= 0;
			J_RL[5][6]= 0;
			J_RL[6][6]= 0;
		}

									/* ----------- Left Leg ----------- */
		/* ----------- x1 is a vector starting at joint1 and ending at critical point ----------- */
				X1[_X_]= critical_point_LL[_X_] - joint_origin_pos[LL_J1][_X_];
				X1[_Y_]= critical_point_LL[_Y_] - joint_origin_pos[LL_J1][_Y_];
				X1[_Z_]= critical_point_LL[_Z_] - joint_origin_pos[LL_J1][_Z_];

				X2[_X_]= critical_point_LL[_X_] - joint_origin_pos[LL_J2][_X_];
				X2[_Y_]= critical_point_LL[_Y_] - joint_origin_pos[LL_J2][_Y_];
				X2[_Z_]= critical_point_LL[_Z_] - joint_origin_pos[LL_J2][_Z_];

				X3[_X_]= critical_point_LL[_X_] - joint_origin_pos[LL_J3][_X_];
				X3[_Y_]= critical_point_LL[_Y_] - joint_origin_pos[LL_J3][_Y_];
				X3[_Z_]= critical_point_LL[_Z_] - joint_origin_pos[LL_J3][_Z_];

				X4[_X_]= critical_point_LL[_X_] - joint_origin_pos[LL_J4][_X_];
				X4[_Y_]= critical_point_LL[_Y_] - joint_origin_pos[LL_J4][_Y_];
				X4[_Z_]= critical_point_LL[_Z_] - joint_origin_pos[LL_J4][_Z_];

				X5[_X_]= critical_point_LL[_X_] - joint_origin_pos[LL_J5][_X_];
				X5[_Y_]= critical_point_LL[_Y_] - joint_origin_pos[LL_J5][_Y_];
				X5[_Z_]= critical_point_LL[_Z_] - joint_origin_pos[LL_J5][_Z_];

				X6[_X_]= critical_point_LL[_X_] - joint_origin_pos[LL_J6][_X_];
				X6[_Y_]= critical_point_LL[_Y_] - joint_origin_pos[LL_J6][_Y_];
				X6[_Z_]= critical_point_LL[_Z_] - joint_origin_pos[LL_J6][_Z_];

				/* ----------- z1 is unit vector of joint 1 axis ----------- */
				Z1[_X_]= joint_axis_pos[LL_J1][_X_];
				Z1[_Y_]= joint_axis_pos[LL_J1][_Y_];
				Z1[_Z_]= joint_axis_pos[LL_J1][_Z_];

				Z2[_X_]= joint_axis_pos[LL_J2][_X_];
				Z2[_Y_]= joint_axis_pos[LL_J2][_Y_];
				Z2[_Z_]= joint_axis_pos[LL_J2][_Z_];

				Z3[_X_]= joint_axis_pos[LL_J3][_X_];
				Z3[_Y_]= joint_axis_pos[LL_J3][_Y_];
				Z3[_Z_]= joint_axis_pos[LL_J3][_Z_];

				Z4[_X_]= joint_axis_pos[LL_J4][_X_];
				Z4[_Y_]= joint_axis_pos[LL_J4][_Y_];
				Z4[_Z_]= joint_axis_pos[LL_J4][_Z_];

				Z5[_X_]= joint_axis_pos[LL_J5][_X_];
				Z5[_Y_]= joint_axis_pos[LL_J5][_Y_];
				Z5[_Z_]= joint_axis_pos[LL_J5][_Z_];

				Z6[_X_]= joint_axis_pos[LL_J6][_X_];
				Z6[_Y_]= joint_axis_pos[LL_J6][_Y_];
				Z6[_Z_]= joint_axis_pos[LL_J6][_Z_];

				crossProd (Z1, X1, J1);
				crossProd (Z2, X2, J2);
				crossProd (Z3, X3, J3);
				crossProd (Z4, X4, J4);
				crossProd (Z5, X5, J5);
				crossProd (Z6, X6, J6);

				/* ----------- Jacobian Matrix for Left Leg ----------- */
				J_LL[1][1]= J1[_X_];
				J_LL[1][2]= J2[_X_];
				J_LL[1][3]= J3[_X_];
				J_LL[1][4]= J4[_X_];
				J_LL[1][5]= J5[_X_];
				J_LL[1][6]= J6[_X_];

				J_LL[2][1]= J1[_Y_];
				J_LL[2][2]= J2[_Y_];
				J_LL[2][3]= J3[_Y_];
				J_LL[2][4]= J4[_Y_];
				J_LL[2][5]= J5[_Y_];
				J_LL[2][6]= J6[_Y_];

				J_LL[3][1]= J1[_Z_];
				J_LL[3][2]= J2[_Z_];
				J_LL[3][3]= J3[_Z_];
				J_LL[3][4]= J4[_Z_];
				J_LL[3][5]= J5[_Z_];
				J_LL[3][6]= J6[_Z_];

				J_LL[4][1]= Z1[_X_];
				J_LL[4][2]= Z2[_X_];
				J_LL[4][3]= Z3[_X_];
				J_LL[4][4]= Z4[_X_];
				J_LL[4][5]= Z5[_X_];
				J_LL[4][6]= Z6[_X_];

				J_LL[5][1]= Z1[_Y_];
				J_LL[5][2]= Z2[_Y_];
				J_LL[5][3]= Z3[_Y_];
				J_LL[5][4]= Z4[_Y_];
				J_LL[5][5]= Z5[_Y_];
				J_LL[5][6]= Z6[_Y_];

				J_LL[6][1]= Z1[_Z_];
				J_LL[6][2]= Z2[_Z_];
				J_LL[6][3]= Z3[_Z_];
				J_LL[6][4]= Z4[_Z_];
				J_LL[6][5]= Z5[_Z_];
				J_LL[6][6]= Z6[_Z_];

				if( (proximity_factor[L_LL] > proximity_factor[L_TL]) && (proximity_factor[L_LL] > proximity_factor[L_FOOT]))
				{
					// if critical point is before joint_5 and joint_6, we don't use joint_5 and joint_6 to avoid
					J_LL[1][5]= 0;
					J_LL[2][5]= 0;
					J_LL[3][5]= 0;
					J_LL[4][5]= 0;
					J_LL[5][5]= 0;
					J_LL[6][5]= 0;

					J_LL[1][6]= 0;
					J_LL[2][6]= 0;
					J_LL[3][6]= 0;
					J_LL[4][6]= 0;
					J_LL[5][6]= 0;
					J_LL[6][6]= 0;
				}

				else if( (proximity_factor[L_LL] < proximity_factor[L_TL]) && (proximity_factor[L_TL] > proximity_factor[L_FOOT]))
				{
					// if critical point is before joint_4, joint_5 and joint_6, we don't use joint_4, joint_5 and joint_6 to avoid
					J_LL[1][4]= 0;
					J_LL[2][4]= 0;
					J_LL[3][4]= 0;
					J_LL[4][4]= 0;
					J_LL[5][4]= 0;
					J_LL[6][4]= 0;

					J_LL[1][5]= 0;
					J_LL[2][5]= 0;
					J_LL[3][5]= 0;
					J_LL[4][5]= 0;
					J_LL[5][5]= 0;
					J_LL[6][5]= 0;

					J_LL[1][6]= 0;
					J_LL[2][6]= 0;
					J_LL[3][6]= 0;
					J_LL[4][6]= 0;
					J_LL[5][6]= 0;
					J_LL[6][6]= 0;
				}
	return TRUE;
}

void avoidanceTask::crossProd(double a[], double b[], double result[])
		{
	/* ----------- cross product of 3d vectors, indices start at 1 ----------- */

		      result[1] = a[2] * b[3] - a[3] * b[2];

		      result[2] = a[3] * b[1] - a[1] * b[3];

		      result[3] = a[1] * b[2] - a[2] * b[1];

		}

void avoidanceTask::matMultAB(double A[4+1][4+1], double B[4+1][4+1], double C[4+1][4+1])
/* ----------- matrix multiplication C=A*B for 4x4 matrices, indeces start at 1. ----------- */
{
	int i, j, k;
	for (i = 1; i <= 4; i++)
	{
		for (j = 1; j <= 4; j++)
		{
			C[i][j] = 0;
			for (k = 1; k <= 4; k++)
				C[i][j] += A[i][k]*B[k][j];
		}
	}
}

int avoidanceTask::changeParameters()
{
  int ivar;
  double dvar;

  get_int(const_cast<char*>("This is how to enter an integer variable"),ivar,&ivar);
  get_double(const_cast<char*>("This is how to enter a double variable"),dvar,&dvar);

  return TRUE;
}

