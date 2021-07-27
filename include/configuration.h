#pragma once

#include "utiltools.h"

const char OBSTACLE = '#';
const char FREESPACE = '.';
const float UNDEFINED_POSITION = 0xFFF5;
const int GETGOAL = 0xFFF6;

const Position UNDEFINED{UNDEFINED_POSITION, UNDEFINED_POSITION, UNDEFINED_POSITION};

//网格地图路径
const std::string map_file_path = "/home/ln/ros_ws/src/uavs_explore_indoor_environment/maps/map7.txt";

//无人机起始点，一定要按照前、左、后、右的顺序排好
const std::vector<Position> init_positons{{2, -1, 0}, {2, 0, 0}, {0, 0, 0}, {0, -1, 0}};
const std::vector<Direction> init_direction{FRONT, LEFT, BACK, RIGHT};

const int NUM_OF_UAV = init_direction.size();

// 停止点
const Position end_position{19, -1, 0};