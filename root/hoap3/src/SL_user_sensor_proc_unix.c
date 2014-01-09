/*!=============================================================================
  ==============================================================================

  \file    SL_user_sensor_proc_unix.c

  \author  Stefan Schaal
  \date    July 2010

  ==============================================================================
  \remarks

  performs reading of sensors, translation to units, and sending out
  motor commands. This verion of the functions is just a simulation
  interface

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// private includes
#include "utility.h"
#include "SL.h"
#include "SL_common.h"
#include "SL_user.h"
#include "SL_sensor_proc.h"
#include "SL_shared_memory.h"
#include "SL_motor_servo.h"
#include "control_matlab.h"

#include <math.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <netdb.h>
#include <string.h>



#define TIME_OUT_NS  1000000000

// local variables



 



// external variables
extern int           motor_servo_errors;

//local variables
double joint_limits[2][21] = {
//lower limits
{-1.5708, -0.5236, -1.2217, 0.0, -1.0472, -0.4189,  //RIGHT LEG
-1.5700, -1.6581, -1.5700, 0.0, //RIGHT ARM
-0.5236, -0.3491, -1.4137, -2.2515, -1.0472, -0.4189, //LEFT LEG
-2.4958, 0.0, -1.5708, -1.9897, //LEFT ARM
0.0}, //HIP
//upper limits
{0.5236, 0.3491, 1.4137, 2.2515, 1.0472, 0.4189, //RIGHT LEG
2.4958, 0.0, 1.5700, 1.9897, //RIGHT ARM
1.5700, 0.5236, 1.2217, 0.0, 1.0472, 0.4189, //LEFT LEG
1.5700, 1.6581, 1.5700, 0.0,
1.5533} //HIP
};

//global functions


// local functions
static int receive_sim_state(void);
static int receive_misc_sensors(void);
static int send_des_command(void);
static int send_sim_state(void);
int limit_joint_range();

 

/*!*****************************************************************************
 *******************************************************************************
\note  init_user_sensor_processing
\date  Nov. 2007
   
\remarks 

          Initializes all user specific sensory processing

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int
init_user_sensor_processing(void) //robot go into initial position in simulation
{
  int count=0;
  int  i;

  
 
  
  return TRUE;
}
/*!*****************************************************************************
 *******************************************************************************
\note  read_user_sensors
\date  Dec 1997
   
\remarks 

    gets sensor readings from the robot and converts them into standard
    units

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[out]    raw :     the raw sensor readings
 \param[out]    misc_raw: the raw miscelleneous sensor readings

 ******************************************************************************/
int
read_user_sensors(SL_Jstate *raw,double *misc_raw)
{
  int i,j;

  double d_zero = 0.0;

  // get simulated sensory data 
  if (!receive_sim_state())
    printf("Time out on receive_sim_state\n");

  if (!receive_misc_sensors())
    printf("Time out on receive_misc_sensors\n");

  // for simulation, the simulation state is just copied to the 
  // raw states 


  limit_joint_range();
  for (i=1; i<=N_DOFS; ++i) {
    raw[i].th   = joint_sim_state[i].th;
    raw[i].thd  = joint_sim_state[i].thd;
    raw[i].thdd = joint_sim_state[i].thdd;
    raw[i].load = joint_sim_state[i].u;
  }
  
  
  

  for (i=1; i<=N_MISC_SENSORS; ++i)
    misc_raw[i] = misc_sim_sensor[i];

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  send_user_commands
\date  April 1999
   
\remarks 

    translates the commands into raw and sends them to the robot

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     commands : the structure containing the commands

 ******************************************************************************/
int
send_user_commands(SL_Jstate *command)

{
  int i,j;


  for (i=1; i<=N_DOFS; ++i)
  {
    joint_sim_state[i].u = command[i].u;
  }

  

  // send commands to simulation
  send_des_command();



  return TRUE;
}
/*!*****************************************************************************
 *******************************************************************************
\note  limit_joint_range
\date 
   
\remarks 

        limit joint range
        only 21*2 limits
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
int limit_joint_range()
{
   int i;
   
   for(i=1; i<=21; i++) //only for 21 joints
   {
      //restriction on lower limit
      if(joint_sim_state[i].th < joint_limits[0][i-1])
      {
         joint_sim_state[i].th = joint_limits[0][i-1];
         printf("joint %d lower lim.\n", i);
      }  
      //restriction on upper limit
      if(joint_sim_state[i].th > joint_limits[1][i-1])
      {
         joint_sim_state[i].th = joint_limits[1][i-1];
         printf("joint %d upper lim.\n", i);
      } 
   }
   return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  receive_sim_state
\date  Nov. 2007
   
\remarks 

        recieves the entire joint_sim_state from shared memory
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
receive_sim_state(void)
{
  
  int i;

  if (semTake(sm_joint_sim_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++motor_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_dofs; ++i)
    sm_joint_sim_state_data[i] = sm_joint_sim_state->joint_sim_state[i];
  
  cSL_Jstate(joint_sim_state,sm_joint_sim_state_data,n_dofs,FLOAT2DOUBLE);

  // get time stamp and check for synchronization errors
  motor_servo_time = servo_time = sm_joint_sim_state->ts;

  semGive(sm_joint_sim_state_sem);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  receive_misc_sensors
\date  Nov. 2007   
\remarks 

        receives the entire misc_sim_sensors from shared memory
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
receive_misc_sensors(void)
{
  
  int i;

  if (n_misc_sensors <= 0)
    return TRUE;

  if (semTake(sm_misc_sim_sensor_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++motor_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_misc_sensors; ++i)
    misc_sim_sensor[i] = sm_misc_sim_sensor->value[i];
  
  semGive(sm_misc_sim_sensor_sem);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  send_des_command
\date  Nov. 2007
   
\remarks 

        send the commands from the joint_sim_state shared memory
        structure
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static int 
send_des_command(void)
{
  
  int i;
  extern double *upd;

  if (semTake(sm_des_commands_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++motor_servo_errors;
    return FALSE;

  } 

  for (i=1; i<=n_dofs; ++i) {
    sm_des_commands->des_command[i].th  = (float) joint_des_state[i].th;
    sm_des_commands->des_command[i].thd = (float) joint_des_state[i].thd;
    sm_des_commands->des_command[i].uff = (float) joint_des_state[i].uff;
    sm_des_commands->des_command[i].u   = (float) joint_sim_state[i].u;
    sm_des_commands->des_command[i].upd = (float) upd[i];
  }

  sm_des_commands->ts = motor_servo_time;
  
  semGive(sm_des_commands_sem);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  user_controller
\date  Feb. 2009
   
\remarks 

          allows the user to modify the commands currently computed by 
          the active controller. Note that only u is actually used in
          the final send out of the commands to the robot, and ufb is only
          to kept as feedback to the user for data collection.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] u     : the current total command
 \param[in,out] ufb   : the current feedback command

 ******************************************************************************/
void
user_controller(double *u, double *uf)
{
  int i,j;

}

/*!*****************************************************************************
 *******************************************************************************
\note  userCheckForMessage
\date  Feb. 2009
   
\remarks 

          this function allows the user to intercept a message send to the
          motor servo and use this information in sensor_user_proc

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name : message identifying name
 \param[in]     k    : index of message in shared memory

 ******************************************************************************/
void
userCheckForMessage(char *name, int k)
{
  int i,j;

}
/*!*****************************************************************************
*******************************************************************************
\note  send_sim_state
\date  Nov. 2007
   
\remarks 

sends the entire joint_sim_state to shared memory


*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int 
send_sim_state(void)
{
  
  int i;

  // joint state
  if (semTake(sm_joint_sim_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
    
    ++motor_servo_errors;
    return FALSE;

  } 

  cSL_Jstate(joint_sim_state,sm_joint_sim_state_data,n_dofs,DOUBLE2FLOAT);
    
  for (i=1; i<=n_dofs; ++i)
      sm_joint_sim_state->joint_sim_state[i] = sm_joint_sim_state_data[i];
  
  semGive(sm_joint_sim_state_sem);

  // no need to send the base for a fixed-base system

  return TRUE;
}

