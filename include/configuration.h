#pragma once

#include "utiltools.h"

const char OBSTACLE = '#';
const char FREESPACE = '.';
const float UNDEFINED_POSITION = 0xFFF5;
const int GETGOAL = 0xFFF6;
const int NOPATH = 0xFFF7;

const Position UNDEFINED{UNDEFINED_POSITION, UNDEFINED_POSITION, UNDEFINED_POSITION};


//无人机起始点，一定要按照前、左、后、右的顺序排好
// const std::vector<Position> init_uavs_positons{
//     {1, -1, 0}, {1, 0, 0}, {0, 0, 0}, {0, -1, 0}}; //初始无人机位置

const std::vector<Direction> init_uavs_direction{FRONT, LEFT, BACK, RIGHT};

// const std::vector<Position> tracking_uavs_positions{
//     {2, -1, 0}, {2, 0, 0}, {-1, 0, 0}, {-1, -1, 0}}; //追击无人机位置

// const int NUM_OF_UAV = 4;

// const std::vector<Direction> init_uavs_direction{FRONT, LEFT, BACK, RIGHT};

const int BATTERY_THRESHOLD = 30;
const int FULL_BATTERY = 100;
