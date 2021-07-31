#pragma once

#include <ros/ros.h>
#include "uav.h"
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace simulation
{
    class Simulation
    {
    public:
        Simulation();
        ~Simulation();

        //启动
        void start();

        //每架无人机开始飞行的回调函数
        void startUAVCallback(int id);
        // void trajectoryCallback();

    private:
        ros::NodeHandle nh;

        // 无人机数组
        std::vector<UAVPtr> uavs;

        // WallAround算法规划器
        std::vector<WallAroundPlannerPtr> wall_around_planners;


        std::vector<bool> is_uav_used;

        // 追击无人机的数量
        int tracking_uavs_num;
        // 当前可派出追击无人机的索引
        int tracking_uavs_index;

        int uavs_num;

        // 地图宽度和高度
        int width;
        int height;

        // 中心原点和左上角原点转换偏移值
        int x_bias;
        int y_bias;

        // 从中心原点转化为左上角原点
        int getColFromX(int x);

        // 从中心原点转化为左上角原点
        int getRowFromY(int y);

        // 从左上角转化为中心原点
        int getXFromCol(int col);

        // 从左上角转化为中心原点
        int getYFromRow(int row);

        // 获取下一个可得到的tracking uav id
        int getNextTrackingUAVId(int pre_uav_id);

        void trackingUAVCallback();

        TheardSafe m_thread_safe;
    };
};