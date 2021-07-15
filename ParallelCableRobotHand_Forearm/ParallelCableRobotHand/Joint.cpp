#include "Joint.h"

Joint::Joint(double phaseShift)
{
	phaseXShift = deg2rad(phaseShift);
	makeHallHomogeneous(PLATE_RADIUS);
}

void Joint::makeHallHomogeneous(double radius)
{
	double TEMP0[4][4] = { 0 }, TEMP1[4][4] = { 0 }, TEMP2[4][4] = { 0 };
	MakeHgTransform('R', phaseXShift, TEMP0);

	MakeHgTransform('y', -radius, TEMP1);
	MatrixMul4_4(TEMP0, TEMP1, hall1homogeneous);

	MakeHgTransform('x', radius*cos(deg2rad(225)), TEMP1);
	MatrixMul4_4(TEMP0, TEMP1, TEMP2);
	MakeHgTransform('y', radius*sin(deg2rad(225)), TEMP1);
	MatrixMul4_4(TEMP2, TEMP1, hall2homogeneous);

	MakeHgTransform('x', radius*cos(deg2rad(315)), TEMP1);
	MatrixMul4_4(TEMP0, TEMP1, TEMP2);
	MakeHgTransform('y', radius*sin(deg2rad(315)), TEMP1);
	MatrixMul4_4(TEMP2, TEMP1, hall3homogeneous);

	MakeHgTransform('x', radius*cos(deg2rad(135)), TEMP1);
	MatrixMul4_4(TEMP0, TEMP1, TEMP2);
	MakeHgTransform('y', radius*sin(deg2rad(135)), TEMP1);
	MatrixMul4_4(TEMP2, TEMP1, hall4homogeneous);

	MakeHgTransform('x', radius*cos(deg2rad(45)), TEMP1);
	MatrixMul4_4(TEMP0, TEMP1, TEMP2);
	MakeHgTransform('y', radius*sin(deg2rad(45)), TEMP1);
	MatrixMul4_4(TEMP2, TEMP1, hall5homogeneous);

	MakeHgTransform('y', radius, TEMP1);
	MatrixCopy4_4(TEMP1, hall6homogeneous);
}

void Joint::setLinkLength(double Length)
{
	linkLength = Length;
}

void Joint::calculateKinematic()
{
	double TEMP[4][4] = { 0 };

	MatrixMul4_4(joint, hall1homogeneous, TEMP);
	ExtractPosition(TEMP, hall1);

	MatrixMul4_4(joint, hall2homogeneous, TEMP);
	ExtractPosition(TEMP, hall2);

	MatrixMul4_4(joint, hall3homogeneous, TEMP);
	ExtractPosition(TEMP, hall3);

	MatrixMul4_4(joint, hall4homogeneous, TEMP);
	ExtractPosition(TEMP, hall4);

	MatrixMul4_4(joint, hall5homogeneous, TEMP);
	ExtractPosition(TEMP, hall5);

	MatrixMul4_4(joint, hall6homogeneous, TEMP);
	ExtractPosition(TEMP, hall6);

	MakeHgTransform('z', linkLength, TEMP);
	MatrixMul4_4(joint, TEMP, linkEnd);
}