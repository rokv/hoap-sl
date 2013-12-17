/*!=============================================================================
  ==============================================================================

  \file    hoapSendTask.h

  \author  Rok Vuga
  \date    Jan. 2013

  ==============================================================================
  \remarks

  task for sending hoap data over udp

  ============================================================================*/

#ifndef HOAPSENDTASK_H_
#define HOAPSENDTASK_H_

class HoapSendTask {

public:


	HoapSendTask();
	~HoapSendTask();

	int initialize();
	int run();
	int changeParameters();



private:

	int hoap_connect(float joint_rot[N_DOFS+1]);
	int hoap_run(float joint_rot[N_DOFS+1]);
	int hoap_disconnect();

	//int hoap_read();

	double start_time_;

	double freq_, amp_;

	SL_DJstate target_[N_DOFS+1];

};

#endif // HOAPSENDTASK_H_
