#include "Finger.h"
int counter = 0;

using namespace std;

Finger::Finger(double metacarpals, double proximal, double middle, double distal)
{
	Origin.setLinkLength(metacarpals);
	MCP.setLinkLength(proximal);
	PIP.setLinkLength(middle);
	DIP.setLinkLength(distal);

	initializeFinger();
}

void Finger::initializeFinger()
{
	setJointAngle(0, 0, 0, 0);
	calculateKinematics();
	calculateCableLength();
}

void Finger::setJointAngle(double MCPx, double MCPy, double PIPx, double DIPx)
{
	phi1 = MCPx;
	th1 = MCPy;
	phi2 = PIPx;
	phi3 = DIPx;
}

void Finger::calculateKinematics()
{
	double TEMP1[4][4] = { 0 }, TEMP2[4][4] = { 0 };

	// Updating finger origin
	MatrixCopy4_4(originOfFing, Origin.joint);
	Origin.calculateKinematic();

	// Updating finger MCP joint
	MakeHgTransform('R', phi1, TEMP1);
	MatrixMul4_4(Origin.linkEnd, TEMP1, TEMP2);
	MakeHgTransform('P', th1, TEMP1);
	MatrixMul4_4(TEMP2, TEMP1, MCP.joint);
	MCP.calculateKinematic();

	// Updating finger PIP joint
	MakeHgTransform('R', phi2, TEMP1);
	MatrixMul4_4(MCP.linkEnd, TEMP1, PIP.joint);
	PIP.calculateKinematic();

	// Updating finger DIP joint
	MakeHgTransform('R', phi3, TEMP1);
	MatrixMul4_4(PIP.linkEnd, TEMP1, DIP.joint);
	DIP.calculateKinematic();
}

void Finger::calculateCableLength()
{
	for (int i = 0; i < 6; i++)
		cableLength[i] = 0.0;
	// Updating cable length from origin to MCP joint
	cableLength[0] += getEuclidDist(Origin.hall1, MCP.hall1);
	cableLength[1] += getEuclidDist(Origin.hall2, MCP.hall2);
	cableLength[2] += getEuclidDist(Origin.hall3, MCP.hall3);
	cableLength[3] += getEuclidDist(Origin.hall4, MCP.hall4);
	cableLength[4] += getEuclidDist(Origin.hall5, MCP.hall5);
	cableLength[5] += getEuclidDist(Origin.hall6, MCP.hall6);

	// Updating cable length from MCP joint to PIP joint
	cableLength[0] += getEuclidDist(MCP.hall1, PIP.hall1);
	cableLength[5] += getEuclidDist(MCP.hall6, PIP.hall6);

	// Updating cable length from PIP joint to DIP joint
	cableLength[0] += getEuclidDist(PIP.hall1, DIP.hall1);
	cableLength[5] += getEuclidDist(PIP.hall6, DIP.hall6);
}

void Finger::calculateDXLPos()
{
	double CableLengthChange[6] = { 0 };
	double CableLengthMean[3] = { 0 };
	double DXLChangeTick[3] = { 0 };
	vectorArithmetic('-', cableLength, InitCableLength, 0, CableLengthChange, 6);

	CableLengthMean[0] = (CableLengthChange[0] - CableLengthChange[5]) / 2;
	CableLengthMean[1] = (CableLengthChange[1] - CableLengthChange[4]) / 2;
	CableLengthMean[2] = (CableLengthChange[2] - CableLengthChange[3]) / 2;

	vectorArithmetic('s', CableLengthMean, CableLengthMean, DXL_ENCODER_RESOLUTION / (2 * PI*BOBIN_RADIUS), DXLChangeTick, 3);
	REL_GoalPOS[0] = (DXL_POS)DXLChangeTick[0];
	REL_GoalPOS[1] = (DXL_POS)DXLChangeTick[1];
	REL_GoalPOS[2] = (DXL_POS)DXLChangeTick[2];

	dxlPosArithmetic('+', ABS_InitialPOS, REL_GoalPOS, ABS_GoalPOS, 3);
}