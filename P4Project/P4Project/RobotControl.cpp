#include "RobotControl.h"

RobotControl::RobotControl()
{
	motorAmount = 5;
	maxVelocity = 50;
	startpos = 2024;
	comm_result = COMM_TX_FAIL;
	addparam_result = false;
	getdata_result = false;
	dxl_error = 0;
	dxl_present_current = 0;
}

RobotControl::~RobotControl()
{

}

void RobotControl::runDynamixelSetup()
{
	RobotControl::_setupDynamixelPort();
	RobotControl::_rebootAll();
	RobotControl::_specifyControlMode();
	RobotControl::_enableTorque();
	RobotControl::_specifyVelocity();
	RobotControl::moveDynamixelStartPos();
}

void RobotControl::restartSetup()
{
	RobotControl::_rebootAll();
	RobotControl::_specifyControlMode();
	RobotControl::_enableTorque();
	RobotControl::_specifyVelocity();
}

void RobotControl::_setupDynamixelPort()
{
	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	dynamixel::GroupSyncWrite groupSyncWritePos(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// Initialize GroupSyncWrite instance
	dynamixel::GroupSyncRead groupSyncReadPos(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);	// Initialize Groupsyncread instance for Present Position
	// Open port
	if (portHandler->openPort())
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
	}
	// Set baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
	}
}

void RobotControl::_rebootAll()
{
	for (int i = 0; i < motorAmount; i++)
	{
		comm_result = packetHandler->reboot(portHandler, i + 1, &dxl_error);
		if (comm_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(comm_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		}
		else
		{
			printf("[ID:%03d] reboot Succeeded\n", (i + 1));
		}
	}
}

void RobotControl::_enableTorque()
{
	for (int i = 0; i < motorAmount; i++)
	{
		// Enable Dynamixel#i+1 Torque
		comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_PRO_TORQUE_ENABLE, true, &dxl_error);
		if (comm_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(comm_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		}
		else
		{
			printf("Dynamixel#%d has been successfully connected \n", i + 1);
		}
	}
}

void RobotControl::_specifyVelocity()
{
	for (int i = 0; i < motorAmount; i++)
	{
		// Enable Dynamixel#i+1 Torque
		comm_result = packetHandler->write4ByteTxRx(portHandler, i + 1, 112, 50, &dxl_error);
		if (comm_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(comm_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		}
		else
		{
			printf("Dynamixel#%d's velocity has succesfully been set \n", i + 1);
		}
	}
}

void RobotControl::_specifyControlMode()
{
	for (int i = 0; i < motorAmount; i++)
	{
		// Enable Dynamixel#i+1 Torque
		comm_result = packetHandler->write2ByteTxRx(portHandler, i + 1, 11, 3, &dxl_error);
		if (comm_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(comm_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		}
		else
		{
			printf("Dynamixel#%d has succesfully been set to position control mode\n", i + 1);
		}
	}
}

void RobotControl::moveDynamixelStartPos()
{
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// Initialize GroupSyncWrite instance

	_alocatePosVal(startpos);	// Allocate starting position value into byte array
	addparam_result = groupSyncWrite.addParam(1, param_goal_position);	// Add Dynamixel#1 goal position value to the Syncwrite storage
	if (addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 1);
	}
	addparam_result = groupSyncWrite.addParam(2, param_goal_position);	// Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	if (addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 2);
	}

	_alocatePosVal(startpos - 1024);	// Allocate starting position value into byte array
	addparam_result = groupSyncWrite.addParam(3, param_goal_position);	// Add Dynamixel#3 goal position value to the Syncwrite storage
	if (addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 3);
	}

	_alocatePosVal(DXL_OPEN_GRIPPER_VALUE);	// Allocate starting position value into byte array
	addparam_result = groupSyncWrite.addParam(4, param_goal_position);	// Add Dynamixel#4 goal position value to the Syncwrite storage
	if (addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 4);
	}
	addparam_result = groupSyncWrite.addParam(5, param_goal_position);	// Add Dynamixel#5 goal position value to the Syncwrite storage
	if (addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 5);
	}

	// Syncwrite goal position
	comm_result = groupSyncWrite.txPacket();
	if (comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(comm_result));
	// Clear syncwrite parameter storage
	groupSyncWrite.clearParam();
}

void RobotControl::_alocatePosVal(int PosVal)
{
	param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(PosVal));
	param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(PosVal));
	param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(PosVal));
	param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(PosVal));
}

void RobotControl::groupMoveDynamixel(int motorID1, int goalPos1, int motorID2, int goalPos2, int motorID3, int goalPos3)
{
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// Initialize GroupSyncWrite instance

	_alocatePosVal(goalPos1);
	addparam_result = groupSyncWrite.addParam(motorID1, param_goal_position);	// Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	if (addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motorID1);
	}

	_alocatePosVal(goalPos2);
	addparam_result = groupSyncWrite.addParam(motorID2, param_goal_position);	// Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	if (addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motorID2);
	}

	_alocatePosVal(goalPos3);
	addparam_result = groupSyncWrite.addParam(motorID3, param_goal_position);	// Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	if (addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motorID3);
	}

	comm_result = groupSyncWrite.txPacket();
	if (comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(comm_result));
	// Clear syncwrite parameter storage
	groupSyncWrite.clearParam();
}

void RobotControl::groupMoveDynamixel(int motorID1, int goalPos1, int motorID2, int goalPos2)
{
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);		// Initialize GroupSyncWrite instance

	_alocatePosVal(goalPos1);
	addparam_result = groupSyncWrite.addParam(motorID1, param_goal_position);	// Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	if (addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motorID1);
	}

	_alocatePosVal(goalPos2);
	addparam_result = groupSyncWrite.addParam(motorID2, param_goal_position);	// Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	if (addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motorID2);
	}

	comm_result = groupSyncWrite.txPacket();
	if (comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(comm_result));
	// Clear syncwrite parameter storage
	groupSyncWrite.clearParam();
}

bool RobotControl::openCloseGripper(double gabValue)
{
	double goalPosRad;
	int goalPos;
	goalPosRad = atan(1.9/11)+asin(((gabValue-3)/2)/(sqrt(pow(11, 2)+pow(1.9, 2))));
	goalPos = 2048 - goalPosRad * 4096 / (2 * M_PI);
	if (0 <= goalPos && goalPos <= 4096)
		groupMoveDynamixel(4, goalPos, 5, goalPos);
	else 
		return 0;
	return 1;
}

void RobotControl::writeSinglePos(int motorID, int posVal)
{
	comm_result = packetHandler->write4ByteTxRx(portHandler, motorID, ADDR_PRO_GOAL_POSITION, posVal, &dxl_error);
	if (comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
}

int RobotControl::readDXLPos(int motorID)
{
	comm_result = packetHandler->read4ByteTxRx(portHandler, motorID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
	if (comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	return dxl_present_position;
}

double RobotControl::_calcGripperLength(double gabValue)
{
	double gripperLength;
	gripperLength = sqrt(pow(11, 2) + pow(1.9, 2) - pow((gabValue-3.8)/2, 2));
	return gripperLength;
}

void RobotControl::_inverseKinematicCalc(double xPos, double yPos, double zPos, double gripperLength)
{
	/*double Theta1 = atan2(xPos, yPos);
	double sigma1 = xPos/(cos(Theta1));	
	double sigma2 = sqrt(pow(zPos, 2) + pow(sigma1, 2));
	double phi1 = atan(zPos/sigma1);
	double phi2 = acos((pow(22, 2) + pow(sigma2, 2) - pow((14.5 + gripperLength), 2)) / (2 * 22 * pow(sigma2,2)));
	double phi3 = acos((pow((14.5 + gripperLength), 2) + pow(22, 2) - pow(sigma2, 2))/(2* (14.5 + gripperLength) * 22));
	double Theta2 = (M_PI / 2) - phi1 - phi2;
	double Theta3 = M_PI - phi3;*/
	std::cout << "Gripper is " << gripperLength << std::endl;
	double  H = 24.5;
	double L1 = 22.0;
	double L2 = 14.5;
	double Cheight = zPos - H;
	double L2Full = L2 + gripperLength;

	double Theta1 = atan2(yPos, xPos);
	if (Theta1 < 0)
	{
		Theta1 = M_PI - Theta1;
	}
	std::cout << "Theta 1 is " << Theta1 << std::endl;

	double  Sigma = sqrt(pow(xPos, 2) + pow(yPos, 2));
	double		C = sqrt(pow(Sigma, 2) + pow(Cheight, 2));
	double Theta2 = 0.5 * M_PI + atan2(Cheight, Sigma) + acos((pow(L1, 2) + pow(C, 2) - pow(L2Full, 2)) / (2 * L1 * C));
	if (Theta2 < 0)
	{
		Theta2 = M_PI - Theta2;
	}
	
	std::cout << "Theta 2 is " << Theta2 << std::endl;

	double Theta3 = acos((pow(L1, 2) + pow(L2Full, 2) - pow(C, 2)) / (2 * L1 * L2Full));
	if (Theta3 < 0)
	{
		Theta3 = M_PI - Theta3;
	}
	std::cout << "Theta 3 is " << Theta3 << std::endl;


	int Pos1 = Theta1 * 4096 / (2 * M_PI);
	int Pos2 = Theta2 * 4096 / (2 * M_PI);
	int Pos3 = Theta3 * 4096 / (2 * M_PI);

	groupMoveDynamixel(1, Pos1, 2, Pos2, 3, Pos3);

	while ((((Pos1 - readDXLPos(1)) > 10) || ((Pos1 - readDXLPos(1)) < -10)) 
		&& (((Pos2 - readDXLPos(2)) > 10) || ((Pos2 - readDXLPos(3)) < -10)) 
		&& (((Pos3 - readDXLPos(3)) > 10) || ((Pos3 - readDXLPos(3)) < -10)))
	{
		std::cout << "moving to position" << std::endl;
		std::cout << "position 1: " + readDXLPos(1) << std::endl;
		std::cout << "position 2: " + readDXLPos(2) << std::endl;
		std::cout << "position 3: " + readDXLPos(3) << std::endl;
	}
}

//Calculates inverse kinematics and moves each motor to the position
void RobotControl::moveGripperToObject(double xPos, double yPos, double zPos, double objectSize)
{
	double gripperLength = _calcGripperLength(objectSize);
	openCloseGripper(objectSize + 2);
	_inverseKinematicCalc(xPos, yPos, zPos + 15, gripperLength);
	_inverseKinematicCalc(xPos, yPos, zPos, gripperLength);
	openCloseGripper(objectSize);
} 