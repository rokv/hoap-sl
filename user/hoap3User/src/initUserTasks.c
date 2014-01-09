/*============================================================================
==============================================================================
                      
                              initUserTasks.c
 
==============================================================================
Remarks:

         Functions needed to initialize and link user tasks for the
         simulation

============================================================================*/


#include "SL.h"
#include "SL_user.h"
#include "SL_integrate.h"
#include "SL_common.h"
#include "SL_task_servo.h"

/* global variables */

/* local variables */
static int user_tasks_initialized = FALSE;


/*****************************************************************************
******************************************************************************
Function Name	: initUserTasks
Date		: June 1999
   
Remarks:

      initialize tasks that are not permanently linked in the simulation
      This replaces the <ltask facility in vxworks -- just that we cannot
      do on-line linking in C.

******************************************************************************
Paramters:  (i/o = input/output)

  none   

*****************************************************************************/
void
initUserTasks(void)

{

   //extern void add_test_task();
	extern void add_sample_task();
	add_sample_task();
	extern void add_sample_task_cpp();
	add_sample_task_cpp();
	extern void add_kinect_playback_task_cpp();
	add_kinect_playback_task_cpp();
	
	extern void add_kinect_playback_r_task_cpp();
	add_kinect_playback_r_task_cpp();
	
	extern void add_kinect_pb_avoidance_task_cpp();
	add_kinect_pb_avoidance_task_cpp();
	
	 extern void add_hoapSend_task_cpp();
	 add_hoapSend_task_cpp();

	 extern void add_avoidance_task_cpp();
	 add_avoidance_task_cpp();

   //add_test_task();
   freezeBase(TRUE);
   changeRealTime(TRUE);
   
  // setG();

  sprintf(initial_user_command,"go0");

}
