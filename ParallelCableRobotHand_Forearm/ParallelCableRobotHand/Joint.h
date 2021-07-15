#pragma once
#include "Robotics.h"
#include "Constants.h"

#include <cmath>

class Joint
{
private:
	double linkLength = 0;
	double phaseXShift = 0;

	double hall1homogeneous[4][4] = { 0 };
	double hall2homogeneous[4][4] = { 0 };
	double hall3homogeneous[4][4] = { 0 };
	double hall4homogeneous[4][4] = { 0 };
	double hall5homogeneous[4][4] = { 0 };
	double hall6homogeneous[4][4] = { 0 };

	void makeHallHomogeneous(double);

public:
	double joint[4][4] = { 0 };
	
	double hall1[3] = { 0 };
	double hall2[3] = { 0 };
	double hall3[3] = { 0 };
	double hall4[3] = { 0 };
	double hall5[3] = { 0 };
	double hall6[3] = { 0 };

	double linkEnd[4][4] = { 0 };

	Joint(double phaseShift);

	void setLinkLength(double);
	void calculateKinematic();
};
