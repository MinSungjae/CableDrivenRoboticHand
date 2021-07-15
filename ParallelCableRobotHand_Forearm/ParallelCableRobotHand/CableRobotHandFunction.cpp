#include "CableRobotHandFunction.h"

using namespace std;

bool initializeRobotHand(CableRobotHand *RobotHand, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, DXL_IDs IDs[])
{
	extern uint8_t dxl_error;
	extern int dxl_comm_result;

	// Torque off to change EEPROM data
	if (!toggleAllDXLTorque(packetHandler, portHandler, IDs, TORQUE_DISABLE))
		return false;
	
	// Change position PID gains
	if (!setAllDXLGains(packetHandler, portHandler, IDs))
		return false;

	// Change operating mode
	if (!changeAllDXLOperatingMode(packetHandler, portHandler, IDs, EXTENDED_POS_CONTROL_MODE))
		return false;
	

	// Setting zero point manually
	DXL_POS TEMP_ABS_POS[DXLS] = { 0 };

	cout << "Point ALL finger to the zero point and press any key to set" << endl;
	_getch();

	if (!syncReadDXLPresentPos(packetHandler, portHandler, IDs, TEMP_ABS_POS))
		return false;
	RobotHand->setHandInitialPos(TEMP_ABS_POS);
	cout << "Dynamixel initialized with ";
	for (int dxl = 0; dxl < DXLS; dxl++)
		cout << TEMP_ABS_POS[dxl] << '\t';
	cout << endl;
	
	// Torque on with setting value
	if (!toggleAllDXLTorque(packetHandler, portHandler, IDs, TORQUE_ENABLE))
		return false;

	// Set robot hand
	if (!syncWriteDXLGoalPos(packetHandler, portHandler, IDs, TEMP_ABS_POS))
		return false;

	return true;
}

void CableRobotHand::viaTrajectoryMaker(char method)
{
	// Method casting
	bool methoded = true;

	if (method == 'X')
		for (int finger = 0; finger < NUM_FINGERS; finger++)
		{
			methoded &= endTrajectory[finger][0];
			methoded &= endTrajectory[finger][2];
			methoded &= endTrajectory[finger][3];
		}
	else if (method == 'Y')
		for (int finger = 0; finger < NUM_FINGERS; finger++)
			methoded &= endTrajectory[finger][1];

	if (methoded)
		method = 'N';

	// Via point generation
#pragma omp parallel
	{
	#pragma omp for
		for (int finger = 0; finger < NUM_FINGERS; finger++)
		{
			switch (method)
			{
			case 'N':
				// MCP ~ DIP joint rotation
				for (int joint = 0; joint < 4; joint++)
				{
					if (!endTrajectory[finger][joint])
					{
						if (GoalAngles[finger][joint] == viaAngles[finger][joint])
						{
							presentAngles[finger][joint] = viaAngles[finger][joint];
							endTrajectory[finger][joint] = true;
						}
						else if (abs(GoalAngles[finger][joint] - viaAngles[finger][joint]) <= TRAJECTORY_RESOLUTION_RAD)
						{
							viaAngles[finger][joint] = GoalAngles[finger][joint];
						}
						else
						{
							presentAngles[finger][joint] = viaAngles[finger][joint];

							if (GoalAngles[finger][joint] - viaAngles[finger][joint] > 0)
								viaAngles[finger][joint] += TRAJECTORY_RESOLUTION_RAD;
							else
								viaAngles[finger][joint] -= TRAJECTORY_RESOLUTION_RAD;
						}
					}
				}
				break;

			case 'X':
				// MCP joint X rotation
				if (!endTrajectory[finger][0])
				{
					if (GoalAngles[finger][0] == viaAngles[finger][0])
					{
						presentAngles[finger][0] = viaAngles[finger][0];
						endTrajectory[finger][0] = true;
					}
					else if (abs(GoalAngles[finger][0] - viaAngles[finger][0]) < TRAJECTORY_RESOLUTION_RAD)
					{
						viaAngles[finger][0] = GoalAngles[finger][0];
					}
					else
					{
						presentAngles[finger][0] = viaAngles[finger][0];

						if (GoalAngles[finger][0] - viaAngles[finger][0] > 0)
							viaAngles[finger][0] += TRAJECTORY_RESOLUTION_RAD;
						else
							viaAngles[finger][0] -= TRAJECTORY_RESOLUTION_RAD;
					}
				}

				// PIP joint
				if (!endTrajectory[finger][2])
				{
					if (GoalAngles[finger][2] == viaAngles[finger][2])
					{
						presentAngles[finger][2] = viaAngles[finger][2];
						endTrajectory[finger][2] = true;
					}
					else if (abs(GoalAngles[finger][2] - viaAngles[finger][2]) < TRAJECTORY_RESOLUTION_RAD)
					{
						viaAngles[finger][2] = GoalAngles[finger][2];
					}
					else
					{
						presentAngles[finger][2] = viaAngles[finger][2];

						if (GoalAngles[finger][2] - viaAngles[finger][2] > 0)
							viaAngles[finger][2] += TRAJECTORY_RESOLUTION_RAD;
						else
							viaAngles[finger][2] -= TRAJECTORY_RESOLUTION_RAD;
					}
				}

				// DIP joint
				if (!endTrajectory[finger][3])
				{
					if (GoalAngles[finger][3] == viaAngles[finger][3])
					{
						presentAngles[finger][3] = viaAngles[finger][3];
						endTrajectory[finger][3] = true;
					}
					else if (abs(GoalAngles[finger][3] - viaAngles[finger][3]) < TRAJECTORY_RESOLUTION_RAD)
					{
						viaAngles[finger][3] = GoalAngles[finger][3];
					}
					else
					{
						presentAngles[finger][3] = viaAngles[finger][3];

						if (GoalAngles[finger][3] - viaAngles[finger][3] > 0)
							viaAngles[finger][3] += TRAJECTORY_RESOLUTION_RAD;
						else
							viaAngles[finger][3] -= TRAJECTORY_RESOLUTION_RAD;
					}
				}
				break;

				// MCP joint Y rotation
			case 'Y':
				if (!endTrajectory[finger][1])
				{
					if (GoalAngles[finger][1] == viaAngles[finger][1])
					{
						presentAngles[finger][1] = viaAngles[finger][1];
						endTrajectory[finger][1] = true;
					}
					else if (abs(GoalAngles[finger][1] - viaAngles[finger][1]) < TRAJECTORY_RESOLUTION_RAD)
					{
						viaAngles[finger][1] = GoalAngles[finger][1];
					}
					else
					{
						presentAngles[finger][1] = viaAngles[finger][1];

						if (GoalAngles[finger][1] - viaAngles[finger][1] > 0)
							viaAngles[finger][1] += TRAJECTORY_RESOLUTION_RAD;
						else
							viaAngles[finger][1] -= TRAJECTORY_RESOLUTION_RAD;
					}
				}
				break;

			case 'L':

				break;

			default:
				cout << "viaTrajectoryMaker method error! Check code..." << endl;
			}
		}
	}
	
}

void CableRobotHand::compareDXLPresentLoad(char ThresholdType, DXL_LOAD LOAD[])
{
	DXL_LOAD MCP_THRESHOLD, DIP_THRESHOLD;
	switch (ThresholdType)
	{
	case 'H':
		MCP_THRESHOLD = HOMING_MCP_THRESHOLD;
		DIP_THRESHOLD = HOMING_DIP_THRESHOLD;
		break;

	case 'F':
		MCP_THRESHOLD = FREESPACE_MCP_THRESHOLD;
		DIP_THRESHOLD = FREESPACE_DIP_THRESHOLD;
		break;

	case 'S':
		MCP_THRESHOLD = SPEED_MCP_THRESHOLD;
		DIP_THRESHOLD = SPEED_DIP_THRESHOLD;
		break;

	case 'G':
		MCP_THRESHOLD = GRASPING_MCP_THRESHOLD;
		DIP_THRESHOLD = GRASPING_DIP_THRESHOLD;
		break;

	case 'T':
		MCP_THRESHOLD = FORCE_MCP_THRESHOLD;
		DIP_THRESHOLD = FORCE_DIP_THRESHOLD;
		break;
	}

#pragma omp parallel
	{
	#pragma omp for
		for (int finger = 0; finger < NUM_FINGERS; finger++)
		{
			if (abs(LOAD[finger * 3 + 0]) > DIP_THRESHOLD)
			{
				endTrajectory[finger][2] = true;
				endTrajectory[finger][3] = true;
				presentAngles[finger][2] = viaAngles[finger][2];
				presentAngles[finger][3] = viaAngles[finger][3];
			}

			if ((abs(LOAD[finger * 3 + 1]) > MCP_THRESHOLD) || (abs(LOAD[finger * 3 + 2]) > MCP_THRESHOLD))
			{
				endTrajectory[finger][0] = true;
				endTrajectory[finger][1] = true;
				presentAngles[finger][0] = viaAngles[finger][0];
				presentAngles[finger][1] = viaAngles[finger][1];
			}
		}
	}
	
}

void moveRobotHand(CableRobotHand *RobotHand, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, DXL_IDs IDs[], DXL_POS GOAL_POS[], char viaMethod, char compareMethod, DXL_POS PRESENT_POS[], DXL_LOAD PRESENT_LOAD[])
{
	RobotHand->checkGoalPositionFeasible();

	cout << "Moving procedure..." << endl;
	bool moving = true;
	
	while (moving)
	{
		cout << '.';
		RobotHand->viaTrajectoryMaker(viaMethod);

		RobotHand->updateHand();

		RobotHand->getUpdatedDXLPos(GOAL_POS);

		if (!syncWriteDXLGoalPos(packetHandler, portHandler, IDs, GOAL_POS))
			cout << "Goal position command ERROR!" << endl;


		if (!syncReadDXLPresentPos(packetHandler, portHandler, IDs, PRESENT_POS))
			cout << "Reading position ERROR!" << endl;

		if (!syncReadDXLPresentLoad(packetHandler, portHandler, IDs, PRESENT_LOAD))
			cout << "Reading load ERROR!" << endl;

		RobotHand->compareDXLPresentLoad(compareMethod, PRESENT_LOAD);

		bool test = true;
		for (int finger = 0; finger < NUM_FINGERS; finger++)
		{
			for (int joint = 0; joint < 4; joint++)
				test &= RobotHand->endTrajectory[finger][joint];
		}

		if (test)
		{
			cout << "Trajectory Finished!!!" << endl;
			break;
		}
	}
	printRobotHandStatus(RobotHand);
	cout << endl << endl;
}

void printRobotHandStatus(CableRobotHand *RobotHand)
{
	cout << "Prsent finger joint angles are: " << endl;
	for (int finger = 0; finger < 5; finger++)
	{
		switch (finger)
		{
		case 0: cout << "Thumb : \t";
			break;
		case 1: cout << "Index : \t";
			break;
		case 2: cout << "Middle: \t";
			break;
		case 3: cout << "Ring  : \t";
			break;
		case 4: cout << "Pinky : \t";
			break;
		}
		for (int angle = 0; angle < 4; angle++)
		{
			cout << rad2deg(RobotHand->presentAngles[finger][angle]) << '\t';
		}
		cout << endl;
	}
	cout << endl << endl;
}