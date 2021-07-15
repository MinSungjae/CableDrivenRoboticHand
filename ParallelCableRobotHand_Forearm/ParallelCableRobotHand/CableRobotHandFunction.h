#pragma once
#include <Windows.h>
#include <iostream>
#include <conio.h>
#include <omp.h>

#include "dynamixel_sdk.h"

#include "CableRobotHand.h"
#include "ActuatorFunciton.h"

bool initializeRobotHand(CableRobotHand *, dynamixel::PacketHandler *, dynamixel::PortHandler *, DXL_IDs[]);
void moveRobotHand(CableRobotHand *RobotHand, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, DXL_IDs IDs[], DXL_POS GOAL_POS[], char viaMethod, char compareMethod, DXL_POS PRESENT_POS[], DXL_LOAD PRESENT_LOAD[]);
void printRobotHandStatus(CableRobotHand *RobotHand);