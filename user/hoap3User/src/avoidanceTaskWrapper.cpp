/*============================================================================
==============================================================================
                      
                              SampleTaskWrapper.cpp
 
==============================================================================
Remarks:

  sekeleton that wrappes a sample task written in C++

============================================================================*/

// SL system headers
#include "SL_system_headers.h"

// SL includes
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"

// local includes
#include "avoidanceTask.h"

extern "C" {
  // global function
  void add_avoidance_task_cpp();

  // local functions
  static int change_avoidance_task_cpp(void);
  static int init_avoidance_task_cpp(void);
  static int run_avoidance_task_cpp(void);
}

static avoidanceTask avoidance_task;

/*****************************************************************************
Function Name	: add_sample_task_cpp
Date		: Jan. 2010
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_avoidance_task_cpp( void )
{
  char task_name[200];
  strcpy(task_name, "avoidance Task example");
  addTask(task_name, init_avoidance_task_cpp, run_avoidance_task_cpp, change_avoidance_task_cpp);
}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_sample_task_cpp
  Date		: Jan. 2010

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_avoidance_task_cpp(void)
{
  return avoidance_task.initialize();
}

/*****************************************************************************
******************************************************************************
  Function Name	: run_sample_task_cpp
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_avoidance_task_cpp(void)
{
  return avoidance_task.run();
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_sample_task_cpp
  Date		: Jan. 2010

  Remarks:

  changes the task parameters

 ******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_avoidance_task_cpp(void)
{
  return avoidance_task.changeParameters();
}
