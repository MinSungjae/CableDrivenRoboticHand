#include "Robotics.h"
// Soldier 214 still alive in your code

using namespace std;

double deg2rad(double deg)
{
	return deg * PI / 180;
}

double rad2deg(double rad)
{
	return rad * 180 / PI;
}

double getEuclidDist(double from[3], double to[3])
{
	double sum = 0;
	for (int i = 0; i < 3; i++)
		sum += pow(to[i] - from[i], 2);
	sum = sqrt(sum);
	return sum;
}

void dxlPosArithmetic(char oper, DXL_POS A[], DXL_POS B[], DXL_POS C[], int length)
{
	switch (oper)
	{
	case '+':
		for (int dxl = 0; dxl < length; dxl++)
			C[dxl] = A[dxl] + B[dxl];
		break;

	case '-':
		for (int dxl = 0; dxl < length; dxl++)
			C[dxl] = A[dxl] - B[dxl];
		break;
	}
}

void vectorArithmetic(char oper, double A_from[], double B[] , double mul_div, double C[], int length)
{
	switch (oper)
	{
	case '+':
		for (int element = 0; element < length; element++)
			C[element] = A_from[element] + B[element];
		break;

	case '-':
		for (int element = 0; element < length; element++)
			C[element] = A_from[element] - B[element];
		break;

	case 's':
		for (int element = 0; element < length; element++)
			C[element] = A_from[element] * mul_div;
		break;

	case '/':
		for (int element = 0; element < length; element++)
			C[element] = A_from[element] / mul_div;
		break;

	case 'C':
		for (int element = 0; element < length; element++)
			C[element] = A_from[element];
		break;
	}
}

void Print4_4Matrix(double matrix[4][4])
{
	for (int row = 0; row < 4; row++)
	{
		for (int col = 0; col < 4; col++)
		{
			cout << matrix[row][col] << '\t';
		}
		cout << endl;
	}
}

void MatrixMul4_4(double A[4][4], double B[4][4], double C[4][4])
{
	for (int row = 0; row < 4; row++)
		for (int col = 0; col < 4; col++)
			C[row][col] = 0;

	for (int row = 0; row < 4; row++)
	{
		for (int k = 0; k < 4; k++)
		{
			for (int col = 0; col < 4; col++)
			{
				C[row][col] += A[row][k] * B[k][col];
			}
		}
	}
}

void MatrixCopy4_4(double in[4][4], double dst[4][4]) 
{
	for (int row = 0; row < 4; row++)
		for (int col = 0; col < 4; col++)
			dst[row][col] = in[row][col];
}

void ExtractPosition(double homoMat[4][4], double posVec[3])
{
	posVec[0] = homoMat[0][3];
	posVec[1] = homoMat[1][3];
	posVec[2] = homoMat[2][3];
}

void ExtractOrientation(double homoMat[4][4], double rotVec[3])
{
	rotVec[0] = atan2(homoMat[2][1], homoMat[2][2]);
	rotVec[1] = atan2(-homoMat[2][0], sqrt(homoMat[2][1] * homoMat[2][1] + homoMat[2][2] * homoMat[2][2]));
	rotVec[2] = atan2(homoMat[1][0], homoMat[0][0]);
}

void MakeHgTransform(char transform, double val, double dst[4][4])
{
	for (int row = 0; row < 4; row++)
		for (int col = 0; col < 4; col++)
			if (row != col)
				dst[row][col] = 0;
			else
				dst[row][col] = 1;

	switch (transform)
	{
	case 'I':
		break;
	case 'x':
		dst[0][3] = val;
		break;
	case 'y':
		dst[1][3] = val;
		break;
	case 'z':
		dst[2][3] = val;
		break;
	case 'R':
		dst[1][1] = cos(val);
		dst[1][2] = -sin(val);
		dst[2][1] = sin(val);
		dst[2][2] = cos(val);
		break;
	case 'P':
		dst[0][0] = cos(val);
		dst[0][2] = sin(val);
		dst[2][0] = -sin(val);
		dst[2][2] = cos(val);
		break;
	case 'Y':
		dst[0][0] = cos(val);
		dst[0][1] = -sin(val);
		dst[1][0] = sin(val);
		dst[1][1] = cos(val);
		break;
	}
}