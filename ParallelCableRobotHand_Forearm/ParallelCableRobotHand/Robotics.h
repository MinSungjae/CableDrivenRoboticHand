#pragma once
#include <cmath>
#include <iostream>
#include <omp.h>

#include "Constants.h"

double deg2rad(double deg);
double rad2deg(double rad);

double getEuclidDist(double from[3], double to[3]);

void vectorArithmetic(char oper, double A_from[], double B[], double mul_div, double C[], int length);
void dxlPosArithmetic(char oper, DXL_POS A[], DXL_POS B[], DXL_POS C[], int length);

void Print4_4Matrix(double matrix[4][4]);
void MatrixMul4_4(double A[4][4], double B[4][4], double C[4][4]);
void MatrixCopy4_4(double in[4][4], double dst[4][4]);
	
void ExtractPosition(double homoMat[4][4], double posVec[3]);
void ExtractOrientation(double homoMat[4][4], double rotVec[3]);

void MakeHgTransform(char transform, double val, double dst[4][4]);