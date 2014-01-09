// reply
typedef struct
{
	float motor_position[28];  //contains joint angles in radians
	// right_leg: 0,1,2,3,4,5,
	// right_arm: 6,7,8,9,
	// left_leg: 10,11,12,13,14,15,
	// left_arm: 16,17,18,19,
	// waist: 20,
	// head: roll 21,pitch(tilt) 22,yaw(pan) 23,
	// wrist: left 24,right 25
	// gripper: left 26,right 27

	short fsr[8]; //contains foot sensor data
	short acc_gyro[6]; //contains data from acelerometer and gyroscope
	float motor_velocity[28]; //contains joint velocities in radians per second
	int time_counter; //contains time from real time robot control
	float time_in_seconds;
	float endeffector_positions[42]; // 6 (arms, legs, head, body) * 7 (position, quaternion orientation)
									 //contains the positions and orientations of end-effectors
	short gripper_force[2]; // left, right
							// contains data from force sensors on the grippers
	short distance_sensor; //contains data from the distance sensor
	short battery_voltage; //contains data from the distance sensor
	char robot_status; //contains the data that indicates the current state of the robot and its UDP connection with the user
						// 'c' - continue connection
						// 'b' - begin connection
						// 'e' - end connection
						// 'i' - info
						// 'i' - not yet received
						// 't' - user timeout
}Report_structure;

union hoap_matlab_reply
{
	char as_array[sizeof(Report_structure)];
	Report_structure as_struct;
};

//command
typedef struct
{
  float command_frequency; // period of the users controller (1/f)
						   // a multiplier of 60, for example 60 Hz, 120 Hz, 180 Hz...
  int command_counter; //contains the number of the command we send to the robot, set to 0
  char filter; //'f' - low pass filter, 
               //'i' - interpolation, 
               //other - nothing         
  char command_type; // 'j' - joint angle control,
                     // 'v' - joint velocity control

  int motor_power[21]; // contains the motors that should be powered 
					   // Note that the first 21 motors can be powered off, but the rest of the motors can not be.
  float joint_rotations[28];  //radians
  float joint_velocities[28]; // radian per second
  char user_status;	// 'c' - continue connection
					// 'b' - begin connection
					// 'e' - end connection
					// 'i' - info
					// 'i' - not yet received
					// 't' - user timeout
					//contains the current status of the user. 
					//This information together with the robot_status is needed to establish and maintain a connection between the user and the robot.
}Command_structure;

union hoap_matlab_command
{
	char as_array[sizeof(Command_structure)];
	Command_structure as_struct;
};
