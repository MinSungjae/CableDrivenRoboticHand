#pragma once
#include "dynamixel_sdk.h"
#include "Constants.h"
#include <iostream>
#include <conio.h>

bool setAllDXLGains(dynamixel::PacketHandler *, dynamixel::PortHandler *, DXL_IDs[]);
bool toggleDXLTorque(dynamixel::PacketHandler *, dynamixel::PortHandler *, DXL_ID, int );
bool toggleAllDXLTorque(dynamixel::PacketHandler *, dynamixel::PortHandler *, DXL_IDs[], int);
bool changeDXLOperatingMode(dynamixel::PacketHandler *, dynamixel::PortHandler *, DXL_ID, int );
bool changeAllDXLOperatingMode(dynamixel::PacketHandler *, dynamixel::PortHandler *, DXL_IDs[], int);

bool syncReadDXLPresentPos(dynamixel::PacketHandler *, dynamixel::PortHandler *, DXL_IDs[], DXL_POS[]);
bool syncReadDXLPresentLoad(dynamixel::PacketHandler *, dynamixel::PortHandler *, DXL_IDs[], DXL_LOAD[]);
bool syncWriteDXLGoalPos(dynamixel::PacketHandler *, dynamixel::PortHandler *, DXL_IDs[], DXL_POS[]);