#pragma once
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>


#define DEVICENAME						"COM3"
#define PROTOCOL_VERSION				2

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_GOAL_CURRENT			102
#define ADDR_PRO_PRESENT_CURRENT		126
#define ADDR_PRO_PRESENT_LOAD			126

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

#define BAUDRATE                        57600

#define DXL_OPEN_GRIPPER_VALUE          1500                 //and open the gripper
#define fingerLength					11.0


class RobotControl
{
public:
	RobotControl(); 
	~RobotControl();
	
	//Open a DXL Port, reboot each DXL motor, Enable torque, specify maximum velocity and move the robot into its starting position
	void runDynamixelSetup();
	//Reboots each DXL motor, Enable torque and specify velocity
	void restartSetup();
	//Move the robot into its starting position
	void moveDynamixelStartPos();
	//Send a different movement command to 3 different motors
	void groupMoveDynamixel(int motorID1, int goalPos1, int motorID2, int goalPos2, int motorID3, int goalPos3);
	//Send a different movement command to 2 different motors
	void groupMoveDynamixel(int motorID1, int goalPos1, int motorID2, int goalPos2);
	//Send a movement command to 1 motor
	void writeSinglePos(int motorID, int posVal);
	//Calculates inverse kinematics and moves each motor to the position
	void moveGripperToObject(double xPos, double yPos, double zPos, double objectSize);
	//Makes the gripper have 'gabValue' cm between the 2 fingers.
	bool openCloseGripper(double gabValue);
	//Pings DXL motor to control if it is still online.
	bool pingDXL(int motorID);
	//Pings the DXL motor and restarts it if needed.
	void testAndRestartDXL(int motorID);
	//Reads the position of a DXL motor, and returns it.
	int readDXLPos(int motorID);

	int motorAmount;
	int maxVelocity;
	int startpos;


private:
	void _setupDynamixelPort();
	void _enableTorque();
	void _specifyVelocity();
	void _specifyControlMode();
	void _rebootAll();
	//Calculates the length of the gripper for a given gabValue
	double _calcGripperLength(double gabValue);
	void _inverseKinematicCalc(double xPos, double yPos, double zPos, double gripperLength);

	void _alocatePosVal(int PosVal);

	int comm_result;					// Communication result
	bool addparam_result;               // addParam result
	bool getdata_result;                // GetParam result
	uint8_t dxl_error;
	uint8_t param_goal_position[4];
	int16_t dxl_present_current;               // Present position
	int16_t dxl_present_load;
	int32_t dxl_present_position;


	dynamixel::PortHandler* portHandler;
	dynamixel::PacketHandler* packetHandler;
};

