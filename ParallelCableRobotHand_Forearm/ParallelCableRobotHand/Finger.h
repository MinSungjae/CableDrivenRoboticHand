#pragma once
#include <iostream>

#include "Joint.h"

#include "Constants.h"
#include "Robotics.h"

class Finger
{
private:
	Joint Origin = Joint(0);
	Joint MCP = Joint(0);
	Joint PIP = Joint(0);
	Joint DIP = Joint(0);

	double phi1 = 0, th1 = 0;
	double phi2 = 0, phi3 = 0;
	double linkLength[4] = { 0 };

	double InitCableLength[6] = { 0 };
	
public:
	double cableLength[6] = { 0 };
	double originOfFing[4][4] = { 0 };

	DXL_POS ABS_PresentPOS[3] = { 0 };
	DXL_POS ABS_InitialPOS[3] = { 0 };
	DXL_POS ABS_GoalPOS[3] = { 0 };

	DXL_POS REL_GoalPOS[3] = { 0 };

	Finger(double, double, double, double);

	void initializeFinger();
	void setJointAngle(double, double, double, double);
	void calculateKinematics();
	void calculateCableLength();
	void calculateDXLPos();
};