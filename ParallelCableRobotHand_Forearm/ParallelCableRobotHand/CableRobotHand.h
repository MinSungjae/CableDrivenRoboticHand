#pragma once
#include <iostream>
#include <omp.h>

#include "Finger.h"
#include "Joint.h"
#include "Robotics.h"
#include "Constants.h"

class CableRobotHand
{
private:
	double ThumbHomogeneous[4][4] = { 0 };
	double IndexHomogeneous[4][4] = { 0 };
	double MiddleHomogeneous[4][4] = { 0 };
	double RingHomogeneous[4][4] = { 0 };
	double PinkyHomogeneous[4][4] = { 0 };

	DXL_POS ABS_InitialPOS[DXLS] = { 0 };
	DXL_POS ABS_PresentPOS[DXLS] = { 0 };
	DXL_POS REL_MaxClenchPOS[DXLS] = { 0 };

	double viaAngles[NUM_FINGERS][4] = { 0 };

	void makeHandHomogeneous();

public:
	Finger Thumb{ 14.0, 42.0, 38.0, 35.0 };
	Finger Index{ 14.0, 42.5, 28.0, 25.0 };
	Finger Middle{ 14.0, 46.5, 28.0, 25.0 };
	Finger Ring{ 14.0, 42.5, 27.5, 25.0 };
	Finger Pinky{ 14.0, 36.6, 20.0, 22.5 };

	double GoalAngles[NUM_FINGERS][4] = { 0 };
	double presentAngles[NUM_FINGERS][4] = { 0 };
	bool endTrajectory[NUM_FINGERS][4] = { false };

	double HANDORIGIN[4][4] = { 0 };

	CableRobotHand();

	// CableRobotHand.cpp
	void setHandPosture();
	void updateKinematics();
	void updateCableLength();
	void updateRELGoalPos();
	void setHandInitialPos(DXL_POS[]);

	void updateHand();
	void getUpdatedDXLPos(DXL_POS[DXLS]);

	void checkGoalPositionFeasible();

	// CableRobotHandFunction.cpp
	void compareDXLPresentLoad(char, DXL_LOAD[]);
	void viaTrajectoryMaker(char);
};