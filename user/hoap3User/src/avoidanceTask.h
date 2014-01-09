/*!=============================================================================
  ==============================================================================

  \file    SampleTask.h

  \author  Peter Pastor
  \date    Jan. 2010

  ==============================================================================
  \remarks

  sample task that is written in C++

  ============================================================================*/

#ifndef AVOIDANCETASK_H_
#define AVOIDANCETASK_H_

class avoidanceTask {

public:

  /*!
   */
	avoidanceTask();
  ~avoidanceTask();

  /*!
   */
  int initialize();
  int run();
  int changeParameters();
  double dist3D_segment_to_segment(double point1[3+1], double point2[3+1], double point3[3+1], double point4[3+1], double avoidance_vector[3+1], double critical_point[3+1]);
  double dist3D_line_to_line(double point1[3+1], double point2[3+1], double point3[3+1], double point4[3+1], double avoidance_vector[3+1]);
  double dist_point_to_point(double point_1[3+1], double point_2[3+1], double avoidance_vector_point[3+1]);
  double dist_Point_to_Segment( double point1[3+1], double point2[3+1], double point3[3+1], double avoidance_vector[3+1], double critical_point[3+1]);
  double get_all_distances();
  double get_all_proximity_factors();
  double get_all_jacobian_coordinates();
  void crossProd(double a[], double b[], double result[]);
  void matMultAB(double A[4+1][4+1], double B[4+1][4+1], double C[4+1][4+1]);
  void avoidance(double *dq_RL_des, double *dq_LL_des, double *dq_RA_des, double *dq_LA_des);

  int n_used_leg_dofs;
  int n_used_arm_dofs;

  double max_dp_RL;
  double max_dp_LL;
  double max_dp_LA;
  double max_dp_RA;

private:

  /*!
   */
 double start_time_;
 double freq_;
 double amp_;
 SL_DJstate target_[N_DOFS+1];

 //double all_distances[];
 double shortest_dist[11+1];
 double avoidance_vector[13][3+1];
 double critical_point[13][3+1];
 double proximity_factor[13];
 double J_LA[6+1][4+1];
 double J_RA[6+1][4+1];
 double proximity_factor_LA;
 double avoidance_vector_LA[3+1];
 double critical_point_LA[3+1];
 double avoidance_vector_RA[3+1];
 double critical_point_RA[3+1];
 double proximity_factor_RA;
 double J_RL[6+1][6+1];
 double proximity_factor_RL;
 double avoidance_vector_RL[3+1];
 double critical_point_RL[3+1];
 double critical_point_LL[3+1];
 double J_LL[6+1][6+1];
 double proximity_factor_LL;
 double avoidance_vector_LL[3+1];


};

#endif // AVOIDANCETASK_H_
