#include "CableRobotHand.h"

using namespace std;

CableRobotHand::CableRobotHand()
{
	MakeHgTransform('I', 0, HANDORIGIN);
	makeHandHomogeneous();
	setHandPosture();
	updateKinematics();
	updateCableLength();
}

void CableRobotHand::makeHandHomogeneous()
{
	double TEMP1[4][4] = { 0 }, TEMP2[4][4] = { 0 };
	double Pivot[4][4] = { 0 };

	MakeHgTransform('z', -6, TEMP1);
	MatrixMul4_4(HANDORIGIN, TEMP1, Pivot);

	// Making transform to thumb origin from hand origin
	MakeHgTransform('P', deg2rad(-40), TEMP1);
	MatrixMul4_4(Pivot, TEMP1, TEMP2);
	MakeHgTransform('z', 40, TEMP1);
	MatrixMul4_4(TEMP2, TEMP1, ThumbHomogeneous);

	// Making transform to index origin from hand origin
	MakeHgTransform('P', deg2rad(-15), TEMP1);
	MatrixMul4_4(Pivot, TEMP1, TEMP2);
	MakeHgTransform('z', 90, TEMP1);
	MatrixMul4_4(TEMP2, TEMP1, IndexHomogeneous);

	// Making transform to middle origin from hand origin
	MakeHgTransform('P', deg2rad(0), TEMP1);
	MatrixMul4_4(Pivot, TEMP1, TEMP2);
	MakeHgTransform('z', 93, TEMP1);
	MatrixMul4_4(TEMP2, TEMP1, MiddleHomogeneous);

	// Making transform to ring origin from hand origin
	MakeHgTransform('P', deg2rad(15), TEMP1);
	MatrixMul4_4(Pivot, TEMP1, TEMP2);
	MakeHgTransform('z', 87, TEMP1);
	MatrixMul4_4(TEMP2, TEMP1, RingHomogeneous);

	// Making transform to pinky origin from hand origin
	MakeHgTransform('P', deg2rad(30), TEMP1);
	MatrixMul4_4(Pivot, TEMP1, TEMP2);
	MakeHgTransform('z', 80, TEMP1);
	MatrixMul4_4(TEMP2, TEMP1, PinkyHomogeneous);
}

void CableRobotHand::setHandInitialPos(DXL_POS INIT_POS[])
{
	for (int dxl = 0; dxl < 3; dxl++)
	{
		Thumb.ABS_InitialPOS[dxl] = INIT_POS[0 + dxl];
		Index.ABS_InitialPOS[dxl] = INIT_POS[3 + dxl];
		Middle.ABS_InitialPOS[dxl] = INIT_POS[6 + dxl];
		Ring.ABS_InitialPOS[dxl] = INIT_POS[9 + dxl];
		Pinky.ABS_InitialPOS[dxl] = INIT_POS[12 + dxl];
	}
}

void CableRobotHand::updateHand()
{
	setHandPosture();
	updateKinematics();
	updateCableLength();
	updateRELGoalPos();
}

void CableRobotHand::setHandPosture()
{
	Thumb.setJointAngle(viaAngles[0][0], viaAngles[0][1], viaAngles[0][2], viaAngles[0][3]);
	Index.setJointAngle(viaAngles[1][0], viaAngles[1][1], viaAngles[1][2], viaAngles[1][3]);
	Middle.setJointAngle(viaAngles[2][0], viaAngles[2][1], viaAngles[2][2], viaAngles[2][3]);
	Ring.setJointAngle(viaAngles[3][0], viaAngles[3][1], viaAngles[3][2], viaAngles[3][3]);
	Pinky.setJointAngle(viaAngles[4][0], viaAngles[4][1], viaAngles[4][2], viaAngles[4][3]);
}

void CableRobotHand::updateKinematics()
{
	// Updating origin of fingers
	MatrixMul4_4(HANDORIGIN, ThumbHomogeneous, Thumb.originOfFing);
	MatrixMul4_4(HANDORIGIN, IndexHomogeneous, Index.originOfFing);
	MatrixMul4_4(HANDORIGIN, MiddleHomogeneous, Middle.originOfFing);
	MatrixMul4_4(HANDORIGIN, RingHomogeneous, Ring.originOfFing);
	MatrixMul4_4(HANDORIGIN, PinkyHomogeneous, Pinky.originOfFing);

	// Updating kinematic of fingers
	Thumb.calculateKinematics();
	Index.calculateKinematics();
	Middle.calculateKinematics();
	Ring.calculateKinematics();
	Pinky.calculateKinematics();
}

void CableRobotHand::updateCableLength()
{
	// Updating cable length of fingers
	Thumb.calculateCableLength();
	Index.calculateCableLength();
	Middle.calculateCableLength();
	Ring.calculateCableLength();
	Pinky.calculateCableLength();
}

void CableRobotHand::updateRELGoalPos()
{
	Thumb.calculateDXLPos();
	Index.calculateDXLPos();
	Middle.calculateDXLPos();
	Ring.calculateDXLPos();
	Pinky.calculateDXLPos();
}

void CableRobotHand::checkGoalPositionFeasible()
{
#pragma omp parallel
	{
	#pragma omp for
		for (int finger = 0; finger < NUM_FINGERS; finger++)
		{
			if (GoalAngles[finger][0] > MCPX_MAX_RADIAN)
			{
				cout << "Goal position of finger " << finger << " MCP_x over maximum degree" << endl;
				GoalAngles[finger][0] = MCPX_MAX_RADIAN;
			}
			else if (GoalAngles[finger][0] < MCPX_MIN_RADIAN)
			{
				cout << "Goal position of finger " << finger << " MCP_x under minimum degree" << endl;
				GoalAngles[finger][0] = MCPX_MIN_RADIAN;
			}

			if (GoalAngles[finger][1] > MCPY_MAX_RADIAN)
			{
				cout << "Goal position of finger " << finger << " MCP_y over maximum degree" << endl;
				GoalAngles[finger][1] = MCPY_MAX_RADIAN;
			}
			else if (GoalAngles[finger][1] < MCPY_MIN_RADIAN)
			{
				cout << "Goal position of finger " << finger << " MCP_y under minimum degree" << endl;
				GoalAngles[finger][1] = MCPY_MIN_RADIAN;
			}

			if (GoalAngles[finger][2] > PIPDIP_MAX_RADIAN)
			{
				cout << "Goal position of finger " << finger << " PIP over maximum degree" << endl;
				GoalAngles[finger][2] = PIPDIP_MAX_RADIAN;
			}
			else if (GoalAngles[finger][2] < PIPDIP_MIN_RADIAN)
			{
				cout << "Goal position of finger " << finger << " PIP under minimum degree" << endl;
				GoalAngles[finger][2] = PIPDIP_MIN_RADIAN;
			}

			if (GoalAngles[finger][3] > PIPDIP_MAX_RADIAN)
			{
				cout << "Goal position of finger " << finger << " DIP over maximum degree" << endl;
				GoalAngles[finger][3] = PIPDIP_MAX_RADIAN;
			}
			else if (GoalAngles[finger][3] < PIPDIP_MIN_RADIAN)
			{
				cout << "Goal position of finger " << finger << " DIP under minimum degree" << endl;
				GoalAngles[finger][3] = PIPDIP_MIN_RADIAN;
			}
		}
	}
}

void CableRobotHand::getUpdatedDXLPos(DXL_POS RobotHandDXLPos[DXLS])
{
	RobotHandDXLPos[0] = Thumb.ABS_GoalPOS[0];
	RobotHandDXLPos[1] = Thumb.ABS_GoalPOS[1];
	RobotHandDXLPos[2] = Thumb.ABS_GoalPOS[2];

	RobotHandDXLPos[3] = Index.ABS_GoalPOS[0];
	RobotHandDXLPos[4] = Index.ABS_GoalPOS[1];
	RobotHandDXLPos[5] = Index.ABS_GoalPOS[2];

	RobotHandDXLPos[6] = Middle.ABS_GoalPOS[0];
	RobotHandDXLPos[7] = Middle.ABS_GoalPOS[1];
	RobotHandDXLPos[8] = Middle.ABS_GoalPOS[2];

	RobotHandDXLPos[9] = Ring.ABS_GoalPOS[0];
	RobotHandDXLPos[10] = Ring.ABS_GoalPOS[1];
	RobotHandDXLPos[11] = Ring.ABS_GoalPOS[2];

	RobotHandDXLPos[12] = Pinky.ABS_GoalPOS[0];
	RobotHandDXLPos[13] = Pinky.ABS_GoalPOS[1];
	RobotHandDXLPos[14] = Pinky.ABS_GoalPOS[2];
}
