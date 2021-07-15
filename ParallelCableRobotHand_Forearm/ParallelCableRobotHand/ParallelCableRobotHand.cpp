// ParallelCableRobotHand.cpp : This file contains the 'main' function. Program execution begins and ends there.
// Seoul National University of Science and Technology
// Electrical and Information Engineering
// Author: Sungjae-Min || msj5826@gmail.com
// Last update: 11/04/2020

#include "ParallelCableRobotHand.h"

using namespace std;

bool DEBUG = false;

int main()
{
	CableRobotHand RobotHand;

	dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	DXL_IDs DXL_IDs[DXLS] = { 2, 1, 3, 5, 4, 6, 8, 7, 9, 11, 10, 12, 14, 13, 15 };
	DXL_POS DXL_GOAL_POS[DXLS] = { 0 };

	DXL_POS DXL_PRESENT_POS[DXLS] = { 0 };
	DXL_LOAD DXL_PRESENT_LOAD[DXLS] = { 0 };

	extern uint8_t dxl_error;
	extern int dxl_comm_result;

	if (portHandler->openPort())
		cout << "Succeeded to open the port!" << endl;
	else
	{
		cout << "Failed to open the port!" << endl;
		_getch();
		return -1;
	}

	if (portHandler->setBaudRate(BAUDRATE))
		cout << "Set baudrate to " << BAUDRATE << endl;
	else
	{
		cout << "Failed to change the baudrate!" << endl;
		_getch();
		return -1;
	}

	if (initializeRobotHand(&RobotHand, packetHandler, portHandler, DXL_IDs))
		cout << "Robot Hand successfully initialized!!!" << endl;
	else
	{
		cout << "Failed to initializing ROBOT HAND" << endl;
		return -1;
	}
	printRobotHandStatus(&RobotHand);
	Sleep(1000);

	double clench;
	cout << "Input max clench angle: ";
	cin >> clench;
	cout << endl << endl;

	char key = 0;
	while (key != 27)
	{
		cout << "Input key o or c \t[r s p]\t escape to end the program" << endl;
		key = _getch();

		switch (key)
		{
		case 'o':
			for (int finger = 0; finger < NUM_FINGERS; finger++)
			{
				for (int joint = 0; joint < 4; joint++)
				{
					RobotHand.endTrajectory[finger][joint] = false;
					RobotHand.GoalAngles[finger][joint] = deg2rad(0);
				}
			}
			break;

		case 'c':
			for (int finger = 0; finger < NUM_FINGERS; finger++)
			{
				for (int joint = 0; joint < 4; joint++)
				{
					RobotHand.endTrajectory[finger][joint] = false;
					if (joint != 1)
						RobotHand.GoalAngles[finger][joint] = deg2rad(clench);
					else
						RobotHand.GoalAngles[finger][joint] = deg2rad(5);
				}
			}
			RobotHand.GoalAngles[0][0] = deg2rad(-20);
			RobotHand.GoalAngles[0][1] = deg2rad(25);
			RobotHand.GoalAngles[1][1] = deg2rad(10);
			RobotHand.GoalAngles[2][1] = deg2rad(0);
			RobotHand.GoalAngles[3][1] = deg2rad(-5);
			RobotHand.GoalAngles[4][1] = deg2rad(-10);
			break;

		case 'r':
			for (int finger = 0; finger < NUM_FINGERS; finger++)
			{
				for (int joint = 0; joint < 4; joint++)
				{
					RobotHand.endTrajectory[finger][joint] = false;
					if (joint == 0)
						RobotHand.GoalAngles[finger][joint] = deg2rad(65);
					else if (joint == 1)
						RobotHand.GoalAngles[finger][joint] = deg2rad(0);
					else
						RobotHand.GoalAngles[finger][joint] = deg2rad(95);
				}
			}
			RobotHand.GoalAngles[0][0] = deg2rad(-20);
			RobotHand.GoalAngles[0][1] = deg2rad(5);
			break;

		case 's':
			for (int finger = 0; finger < NUM_FINGERS; finger++)
			{
				for (int joint = 0; joint < 4; joint++)
				{
					RobotHand.endTrajectory[finger][joint] = false;
					if (joint == 0)
						RobotHand.GoalAngles[finger][joint] = deg2rad(65);
					else if (joint == 1)
						RobotHand.GoalAngles[finger][joint] = deg2rad(0);
					else
						RobotHand.GoalAngles[finger][joint] = deg2rad(95);
				}
			}

			for (int finger = 1; finger < 3; finger++)
			{
				for (int joint = 0; joint < 4; joint++)
				{
					RobotHand.GoalAngles[finger][joint] = deg2rad(0);
				}
			}
			RobotHand.GoalAngles[0][0] = deg2rad(-20);
			RobotHand.GoalAngles[0][1] = deg2rad(5);
			RobotHand.GoalAngles[1][1] = deg2rad(10);
			RobotHand.GoalAngles[2][1] = deg2rad(-10);
			break;

		case 'p':
			for (int finger = 0; finger < NUM_FINGERS; finger++)
			{
				for (int joint = 0; joint < 4; joint++)
				{
					RobotHand.endTrajectory[finger][joint] = false;
					RobotHand.GoalAngles[finger][joint] = deg2rad(0);
				}
			}
			RobotHand.GoalAngles[0][0] = deg2rad(-15);
			RobotHand.GoalAngles[0][1] = deg2rad(7.5);
			RobotHand.GoalAngles[1][1] = deg2rad(5);
			RobotHand.GoalAngles[2][1] = deg2rad(0);
			RobotHand.GoalAngles[3][1] = deg2rad(-7.5);
			RobotHand.GoalAngles[4][1] = deg2rad(-15);
			break;

		case 'l':
			for (int finger = 0; finger < NUM_FINGERS; finger++)
			{
				for (int joint = 0; joint < 4; joint++)
				{
					RobotHand.endTrajectory[finger][joint] = false;
					if (joint == 0)
						RobotHand.GoalAngles[finger][joint] = deg2rad(65);
					else if (joint == 1)
						RobotHand.GoalAngles[finger][joint] = deg2rad(0);
					else
						RobotHand.GoalAngles[finger][joint] = deg2rad(95);
				}
			}

			for (int finger = 0; finger < NUM_FINGERS; finger++)
			{
				for (int joint = 0; joint < 4; joint++)
				{
					if (finger == 0 || finger == 1 || finger == 4)
						RobotHand.GoalAngles[finger][joint] = deg2rad(0);
				}
			}

			RobotHand.GoalAngles[0][0] = deg2rad(-30);
			RobotHand.GoalAngles[0][1] = deg2rad(30);
			RobotHand.GoalAngles[1][1] = deg2rad(-10);
			RobotHand.GoalAngles[4][1] = deg2rad(30);
			break;
		}
		system("cls");
		moveRobotHand(&RobotHand, packetHandler, portHandler, DXL_IDs, DXL_GOAL_POS, 'N', 'F', DXL_PRESENT_POS, DXL_PRESENT_LOAD);
	}
	system("cls");
	cout << endl << endl << "Moving hand to initial position..." << endl;
	cout << "Program will automatically finished." << endl << endl;
	for (int finger = 0; finger < NUM_FINGERS; finger++)
	{
		for (int joint = 0; joint < 4; joint++)
		{
			RobotHand.endTrajectory[finger][joint] = false;
			RobotHand.GoalAngles[finger][joint] = deg2rad(0);
		}
	}
	moveRobotHand(&RobotHand, packetHandler, portHandler, DXL_IDs, DXL_GOAL_POS, 'N', 'H', DXL_PRESENT_POS, DXL_PRESENT_LOAD);
	Sleep(2000);
	toggleAllDXLTorque(packetHandler, portHandler, DXL_IDs, TORQUE_DISABLE);
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
