#pragma once
#include "Constants.h"
#include "Robotics.h"

double scaleAngles[MUSIC_SCALES+1][NUM_FINGERS][4] =
{
	{
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(5), deg2rad(-7), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // Do thumb
		deg2rad(-10), deg2rad(20), deg2rad(15), deg2rad(15),
		deg2rad(5), deg2rad(-7), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // Do sharp index
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(25), deg2rad(7.5), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // Re thumb
		deg2rad(-5), deg2rad(20), deg2rad(25), deg2rad(25),
		deg2rad(5), deg2rad(-7), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // Re sharp index
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(25), deg2rad(-8), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // Mi index
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(30), deg2rad(-15), deg2rad(25), deg2rad(25),
		deg2rad(7.5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // Fa middle
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(5), deg2rad(-7), deg2rad(10), deg2rad(10),
		deg2rad(30), deg2rad(0), deg2rad(25), deg2rad(25),
		deg2rad(5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // Fa sharp middle
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(5), deg2rad(-7), deg2rad(10), deg2rad(10),
		deg2rad(20), deg2rad(-7.5), deg2rad(20), deg2rad(20),
		deg2rad(5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // Sol middle
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(5), deg2rad(-7), deg2rad(10), deg2rad(10),
		deg2rad(30), deg2rad(-15), deg2rad(25), deg2rad(25),
		deg2rad(5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // Sol sharp Ring
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(5), deg2rad(-7), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(25), deg2rad(10), deg2rad(25), deg2rad(25),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // La ring
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(5), deg2rad(-7), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(35), deg2rad(-1.25), deg2rad(30), deg2rad(30),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // La sharp Ring
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(5), deg2rad(-7), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(25), deg2rad(-8), deg2rad(25), deg2rad(25),
		deg2rad(7.5), deg2rad(5), deg2rad(10), deg2rad(10)
	},
	{ // Ti Little
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(5), deg2rad(-7), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(30), deg2rad(5), deg2rad(25), deg2rad(25)
	},
	{ // HighDo little
		deg2rad(5), deg2rad(-10), deg2rad(15), deg2rad(15),
		deg2rad(5), deg2rad(-7), deg2rad(10), deg2rad(10),
		deg2rad(7.5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(5), deg2rad(0), deg2rad(10), deg2rad(10),
		deg2rad(30), deg2rad(-17.5), deg2rad(25), deg2rad(25)
	}
};

#define LengthAirplane 25
int MelodyAirplane[2][LengthAirplane] =
{
	{
		5, 3, 1, 3, 5, 5, 5, 3, 3, 3, 5, 5, 5, 5, 3, 1, 3, 5, 5, 5, 3, 3, 5, 3, 1
	},
	{
		3, 1, 2, 2, 2, 2, 4, 2, 2, 4, 2, 2, 4, 3, 1, 2, 2, 2, 2, 4, 2, 2, 3, 1, 8
	}
};

#define LengthButterfly 27
int MelodyButterfly[2][LengthButterfly] =
{
	{
		8, 5, 5, 6, 3, 3, 1, 3, 5, 6, 8, 8, 8, 8, 5, 5, 5, 6, 3, 3, 1, 5, 8, 8, 5, 5, 5
	},
	{
		2, 2, 4, 2, 2, 4, 2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 6
	}
};

#define LengthLittleStar 42
int MelodyLittleStar[2][LengthLittleStar] =
{
	{
		1,1,8,8,10,10,8,6,6,5,5,3,3,1,8,8,6,6,5,5,3,8,8,6,6,5,5,3,1,1,8,8,10,10,8,6,6,5,5,3,3,1
	},
	{
		2,2,2,2,2,2,4,2,2,2,2,2,2,4,2,2,2,2,2,2,4,2,2,2,2,2,2,4,2,2,2,2,2,2,4,2,2,2,2,2,2,4
	}
};

#define LengthCarol 29
int MelodyCarol[2][LengthCarol] =
{
	{
		5,6,8,8,10,12,13,13,5,6,8,8,8,8,8,10,8,6,6,6,6,5,8,1,5,3,6,1,1,
	},
	{
		1,1,2,8,1,1,2,8,1,1,1,1,1,1,6,1,1,1,1,2,2,2,2,2,2,2,8,1,2,
	}
};