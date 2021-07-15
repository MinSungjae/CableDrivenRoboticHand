#pragma once

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <Windows.h>
#include <iostream>
#include <math.h>

#include "Robotics.h"
#include "Constants.h"
#include "CableRobotHand.h"
#include "CableRobotHandFunction.h"
#include "GlobalVariables.h"
#include "dynamixel_sdk.h"

#include "PianoHand.h"

void printRobotHandStatus(CableRobotHand *);