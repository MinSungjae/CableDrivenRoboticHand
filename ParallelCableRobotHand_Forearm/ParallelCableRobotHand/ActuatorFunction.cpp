#include "ActuatorFunciton.h"

using namespace std;

extern bool DEBUG;

bool setAllDXLGains(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, DXL_IDs IDs[])
{
	extern uint8_t dxl_error;
	extern int dxl_comm_result;

	cout << "Setting gains..." << endl;

	for (int dxl = 0; dxl < DXLS; dxl++)
	{
		// Set position max PWM change
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, IDs[dxl], ADDR_POS_PWM_LIMIT, POS_PWM_LIMIT, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;
			return false;
		}
		else if (dxl_error != 0)
		{
			cout << packetHandler->getRxPacketError(dxl_error) << endl;
			return false;
		}
		else
			cout << "DXL ID: " << IDs[dxl] << " position max PWM changed" << endl;

		// Set position P gain change
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, IDs[dxl], ADDR_POS_P_GAIN, POS_P_GAIN, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;
			return false;
		}
		else if (dxl_error != 0)
		{
			cout << packetHandler->getRxPacketError(dxl_error) << endl;
			return false;
		}
		else
			cout << "DXL ID: " << IDs[dxl] << " position P GAIN changed" << endl;

		// Set position I gain change
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, IDs[dxl], ADDR_POS_I_GAIN, POS_I_GAIN, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;
			return false;
		}
		else if (dxl_error != 0)
		{
			cout << packetHandler->getRxPacketError(dxl_error) << endl;
			return false;
		}
		else
			cout << "DXL ID: " << IDs[dxl] << " position I GAIN changed" << endl;

		// Set position D gain change
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, IDs[dxl], ADDR_POS_D_GAIN, POS_D_GAIN, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;
			return false;
		}
		else if (dxl_error != 0)
		{
			cout << packetHandler->getRxPacketError(dxl_error) << endl;
			return false;
		}
		else
			cout << "DXL ID: " << IDs[dxl] << " position D GAIN changed" << endl;
	}
	cout << "All gain set!" << endl;
	return true;
}

bool toggleDXLTorque(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, DXL_ID ID, int TORQUE)
{
	extern uint8_t dxl_error;
	extern int dxl_comm_result;

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;
		return false;
	}
	else if (dxl_error != 0)
	{
		cout << packetHandler->getRxPacketError(dxl_error) << endl;
		return false;
	}
	return true;
}

bool toggleAllDXLTorque(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, DXL_IDs IDs[], int TORQUE)
{
	bool result = false;

	for (int dxl = 0; dxl < DXLS; dxl++)
	{
		result = toggleDXLTorque(packetHandler, portHandler, IDs[dxl], TORQUE);
		if (!result)
			return false;
	}
	cout << "All Torque changed" << endl;
	return true;
}

bool changeDXLOperatingMode(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, DXL_ID ID, int MODE)
{
	extern uint8_t dxl_error;
	extern int dxl_comm_result;

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID, ADDR_OPERATING_MODE, MODE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;
		return false;
	}
	else if (dxl_error != 0)
	{
		cout << packetHandler->getRxPacketError(dxl_error) << endl;
		return false;
	}
	return true;
}

bool changeAllDXLOperatingMode(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, DXL_IDs IDs[], int MODE)
{
	bool result = 0;

	for (int dxl = 0; dxl < DXLS; dxl++)
	{
		result = changeDXLOperatingMode(packetHandler, portHandler, IDs[dxl], MODE);
		if (!result)
			return false;
	}

	cout << "All operating mode changed" << endl;
	return true;
}

bool syncReadDXLPresentPos(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, DXL_IDs IDs[], DXL_POS presentPOS[])
{
	extern uint8_t dxl_error;
	extern int dxl_comm_result;

	dynamixel::GroupSyncRead SyncReadPresentPos(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
	bool dxl_addparam_result = false;
	bool dxl_getdata_result = false;

	for (int dxl = 0; dxl < DXLS; dxl++)
	{
		if (DEBUG)
			cout << "Add groupSyncRead parameters.." << endl;
		dxl_addparam_result = SyncReadPresentPos.addParam(IDs[dxl]);
		if (dxl_addparam_result != true)
		{
			if (DEBUG)
				cerr << "ID : " << IDs[dxl] << "groupSyncRead addparam failed!" << endl;
			return false;
		}
	}

	dxl_comm_result = SyncReadPresentPos.txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS)
		if(DEBUG)
			cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;
	else
	{
		for (int dxl = 0; dxl < DXLS; dxl++)
		{
			if (SyncReadPresentPos.getError(IDs[dxl], &dxl_error))
				if (DEBUG)
					cout << "ID: " << IDs[dxl] << " : " << packetHandler->getRxPacketError(dxl_error) << endl;
		}
	}

	for (int dxl = 0; dxl < DXLS; dxl++)
	{
		dxl_getdata_result = SyncReadPresentPos.isAvailable(IDs[dxl], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
		if (dxl_getdata_result != true)
		{
			if (DEBUG)
				cerr << "DXL ID: " << IDs[dxl] << " groupSyncRead getdata failed" << endl;
			return false;
		}
	}

	for (int dxl = 0; dxl < DXLS; dxl++)
	{
		presentPOS[dxl] = SyncReadPresentPos.getData(IDs[dxl], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
		if (DEBUG)
			cout << "ID: " << IDs[dxl] << " Position: " << presentPOS[dxl] << endl;
	}

	SyncReadPresentPos.clearParam();
	return true;
}

bool syncReadDXLPresentLoad(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, DXL_IDs IDs[], DXL_LOAD presentLoad[])
{
	extern uint8_t dxl_error;
	extern int dxl_comm_result;

	dynamixel::GroupSyncRead SyncReadPresentLoad(portHandler, packetHandler, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD);
	bool dxl_addparam_result = false;
	bool dxl_getdata_result = false;

	for (int dxl = 0; dxl < DXLS; dxl++)
	{
		if (DEBUG)
			cout << "Add groupSyncRead parameters.." << endl;
		dxl_addparam_result = SyncReadPresentLoad.addParam(IDs[dxl]);
		if (dxl_addparam_result != true)
		{
			if (DEBUG)
				cerr << "ID : " << IDs[dxl] << "groupSyncRead addparam failed!" << endl;
			return false;
		}
	}

	dxl_comm_result = SyncReadPresentLoad.txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS)
		if (DEBUG)
			cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;
		else
		{
			for (int dxl = 0; dxl < DXLS; dxl++)
			{
				if (SyncReadPresentLoad.getError(IDs[dxl], &dxl_error))
					if (DEBUG)
						cout << "ID: " << IDs[dxl] << " : " << packetHandler->getRxPacketError(dxl_error) << endl;
			}
		}

	for (int dxl = 0; dxl < DXLS; dxl++)
	{
		dxl_getdata_result = SyncReadPresentLoad.isAvailable(IDs[dxl], ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD);
		if (dxl_getdata_result != true)
		{
			if (DEBUG)
				cerr << "DXL ID: " << IDs[dxl] << " groupSyncRead getdata failed" << endl;
			return false;
		}
	}

	for (int dxl = 0; dxl < DXLS; dxl++)
	{
		presentLoad[dxl] = SyncReadPresentLoad.getData(IDs[dxl], ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD);
		if (DEBUG)
			cout << "ID: " << IDs[dxl] << " Position: " << presentLoad[dxl] << endl;
	}

	SyncReadPresentLoad.clearParam();
	return true;
}

bool syncWriteDXLGoalPos(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, DXL_IDs IDs[], DXL_POS goalPOS[])
{
	dynamixel::GroupSyncWrite SyncWriteGoalPos(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

	extern int dxl_comm_result;

	uint8_t param_goal_position[4];
	bool dxl_addparam_result = false;
	bool dxl_getdata_result = false;

	for (int dxl = 0; dxl < DXLS; dxl++)
	{
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goalPOS[dxl]));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goalPOS[dxl]));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goalPOS[dxl]));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goalPOS[dxl]));
		dxl_addparam_result = SyncWriteGoalPos.addParam(IDs[dxl], param_goal_position);
		if (dxl_addparam_result != true)
		{
			if(DEBUG)
				cerr << "DXL ID: " << IDs[dxl] << "groupSyncWrite failed" << endl;
			return false;
		}
	}

	dxl_comm_result = SyncWriteGoalPos.txPacket();
	if (dxl_comm_result != COMM_SUCCESS)
		cout << packetHandler->getTxRxResult(dxl_comm_result) << endl;

	SyncWriteGoalPos.clearParam();
	return true;
}