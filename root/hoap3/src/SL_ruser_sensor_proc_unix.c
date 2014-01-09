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


#define M_PII 3.141592653589793
#define DEG_TO_RAD_MULT (M_PII / 180.0)
#define SND_BUFF_SIZE       ((int) sizeof(Command_structure)) // set the buffer sizes
#define RCV_BUFF_SIZE       ((int) sizeof(Report_structure))
#define SRV_IP "178.172.42.110"
#define HOAP_PORT 15005





#define TIME_OUT_NS  1000000000

// local variables
struct sockaddr_in si_other;
int sock;
float old_time = 0.0;
float dT;
float old_velocity[28] = {
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0 
}; 
int stevec=0;


 



// external variables
extern int           motor_servo_errors;

//local variables
union hoap_matlab_reply receive_buf;
union hoap_matlab_command command_buf;

//global functions


// local functions
static int receive_sim_state(void);
static int receive_misc_sensors(void);
static int send_des_command(void);
static int send_sim_state(void);
int hoapConnect(void);
int hoapRecv(void);
int hoapSend(float *joint_position);
int hoapDiss(void);



/*
 * init_joint_position: Put the desire initial positions for every joint in radians.
 * Function writes data in command_buf and sends it to hoap.
 * Function receives data and writes it in receive_buff.
 * Function returns 0 if ok.
*/
int hoapConnect(void) //return 0 if ok
{

	double initial_position[21] = {
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // right leg
	0.0, 0.0, 0.0, 0.0, // right arm
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // left leg
	0.0, 0.0, 0.0, 0.0, // left arm
	0.0 // waist
	}; 
	

	int count;
	struct sockaddr_in si_me;
	socklen_t len_RCV=sizeof(si_me);
	int len_SND=sizeof(si_other);
	int err;
	int opt=0;
	int k;


	command_buf.as_struct.command_frequency=(float)(1.0/120.0); // period 1/120
	command_buf.as_struct.command_counter=0;
	command_buf.as_struct.filter='f';
	command_buf.as_struct.command_type = 'j'; //'j' ... joint control
	command_buf.as_struct.user_status = 'b';

	for(k = 0; k < 21; k++)//power motors, only 21, you can use 28
	{
		command_buf.as_struct.motor_power[k] = 1;
	}
	for(k = 0; k < 28; k++)
	{
		command_buf.as_struct.joint_velocities[k] = 0.0f;
	}
	for(k = 0; k < 28; k++)
	{
		command_buf.as_struct.joint_rotations[k] = initial_position[k];
	}

	//open connection
	sock=socket(PF_INET, SOCK_DGRAM, 0);
	if(sock < 0)
	{
		printf("Socket error.\n");
		return -1;
	}
	si_other.sin_family = PF_INET;
	si_other.sin_port = htons(HOAP_PORT);
	if (inet_aton(SRV_IP, &si_other.sin_addr)==0)
	{
		fprintf(stderr, "inet_aton() failed\n");
		return -1;
	}
	//send buffer, begin connection
	count = sendto(sock, command_buf.as_array, SND_BUFF_SIZE, 0, (struct sockaddr*)&si_other, len_SND);
	if(count < 0)
	{
		printf("Send error.\n");
		return -1;
	}


	printf("Send data.\n");

	while(1) //wait until robot_status = 'c'
	{
		count = recvfrom(sock,receive_buf.as_array , RCV_BUFF_SIZE, 0, (struct sockaddr*)&si_me, &len_RCV);

		if(count < 0)
		{
			printf("Receive error.\n");
			return -1;
			break;
		}
		if(receive_buf.as_struct.robot_status == 'c')
		{
			break;
		}
	}

        command_buf.as_struct.command_counter = 1;
	return 0;
}
/*
 * Function receives data and write it in receive_buff
 * Function returns 0 if ok.
*/
int hoapRecv(void)
{
	int count;
	struct sockaddr_in si_me;
	socklen_t len_RCV = sizeof(si_me);

	count = recvfrom(sock,receive_buf.as_array , RCV_BUFF_SIZE, 0, (struct sockaddr*)&si_me, &len_RCV);
	if(count < 0)
	{ //send error
		printf("Receive error.\n");
		return -1;
	}
	else
	{
		return 0;
	}
}
/*
 * joint_position: Put the desire positions for every join.
 * Function writes data in command_buf and sends it to hoap.
 * Function returns 0 if ok.
*/
int hoapSend(float *joint_position)
{
	int len_SND=sizeof(si_other);
	int count;
        int k;

	command_buf.as_struct.user_status = 'c';  //'c' - continue connection

	for(k = 0; k < 21; k++)
	{
		command_buf.as_struct.motor_power[k] = 1;
	}
	for(k = 0; k < 28; k++)
	{
		command_buf.as_struct.joint_velocities[k] = 0.0f;
	}
	for(k = 0; k < 28; k++)
	{
		command_buf.as_struct.joint_rotations[k] = joint_position[k];

	}

	count = sendto(sock, command_buf.as_array, SND_BUFF_SIZE, 0, (struct sockaddr*)&si_other, len_SND);
	if(count < 0)
	{
		printf("Send error.\n");
		return -1;
	}
	return 0;
}
/*
 * Function writes data in command_buf and sends it to hoap.
 * Function returns 0 if ok.
*/
int hoapDiss(void)
{
	int count;
	int len_SND=sizeof(si_other);


	command_buf.as_struct.command_type = 'e';
    count = sendto(sock, command_buf.as_array, SND_BUFF_SIZE, 0, (struct sockaddr*)&si_other, len_SND);
    if(count < 0)
	{ //send error
		printf("Send error.\n");
		return -1;
	}
	//close opened ports
	if(count >= 0)
	{
		close(sock);
		printf("Disconnect");
		printf("\n");
		return 0;
	}
}
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
  
  count = hoapConnect(); //real robot go into initial position
   
  old_time = receive_buf.as_struct.time_in_seconds;

  if(count != 0)
  {
    printf("\nHoapConnect error.\n");
    return FALSE;
  }
  else
  {
    printf("\nHoap is connected.\n");
    return TRUE;
  }
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
  float f_zero = 0.0;  


  // read the sensors
  hoapRecv(); 

  
  

  stevec++;

  dT = receive_buf.as_struct.time_in_seconds - old_time; 
  
  // note the difference between receive_buf and raw indexing, add one in raw  
  //printf("NOV CIKEL\n");
  for (i=1; i<=N_DOFS; ++i) {
    raw[i].th   =(double)receive_buf.as_struct.motor_position[i-1];
    raw[i].thd  =(double)receive_buf.as_struct.motor_velocity[i-1];
    raw[i].thdd =(double)(receive_buf.as_struct.motor_velocity[i-1]-old_velocity[i-1])/dT;
    raw[i].load = 0.0;//joint_sim_state[i].u;

    old_velocity[i-1] = receive_buf.as_struct.motor_velocity[i-1];
    if(stevec == 1 || stevec == 10000)
    {
      printf("motor_positions = %f .\n", receive_buf.as_struct.motor_position[i-1]);
    }
  }

  old_time = receive_buf.as_struct.time_in_seconds;
  

  //(!) invert arm joint 3 after receiving
  raw[RA_J3].th = -raw[RA_J3].th;
  raw[RA_J3].thd = -raw[RA_J3].thd;
  raw[LA_J3].th = -raw[LA_J3].th;
  raw[LA_J3].thd = -raw[LA_J3].thd;

  // send the old sensor readings (which are processed by now) to
  // shared memory for the graphics visualization	
  for (i=1; i<=N_DOFS; ++i)
  {
    joint_sim_state[i] = joint_state[i];
  }








  //for (i=1; i<=N_MISC_SENSORS; ++i)
   //misc_sim_sensor[i] = misc_sensor[i];
  send_sim_state();
  //send_misc_sensors();



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
  float joint_des_position[28];


  //note the difference between joint_des_position and command indexing, 
  //add one in joint_des_position
  for (i=1; i<=N_DOFS; ++i)
  { 
    //round on 3 decimal places, because of the sensors offset
    if(command[i].th >= 0.0)
    {
      joint_des_position[i-1] = (floor((float)joint_des_state[i].th*1000))/1000;
    }
    else
    {
       joint_des_position[i-1] = (ceil((float)joint_des_state[i].th*1000))/1000;  
    }


    //joint_des_position[i-1] = (float)command[i].th; no roudning
    
    if(stevec == 1 || stevec == 10000)
    {
    	printf("joint:%d, command = %f .\n",i-1, joint_des_position[i-1]);
    }
  
  }
  
  //(!) invert arm joint 3 before sending
  joint_des_position[RA_J3-1] = -joint_des_position[RA_J3-1];
  joint_des_position[LA_J3-1] = -joint_des_position[LA_J3-1];

  hoapSend(joint_des_position);

  // send commands to simulation
  //send_des_command();

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

  return TRUE;

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

  return TRUE;

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

