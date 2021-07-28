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

        // 追击无人机数组
        // std::vector<UAVPtr> tracking_uavs;

        // WallAround算法规划器
        std::vector<WallAroundPlannerPtr> wall_around_planners;

        // 追击无人机的数量
        int tracking_uavs_num;
        // 当前可派出追击无人机的索引
        int tracking_uavs_index;

        int uavs_num;

        // 用于多线程之间获取tracking uav的索引避免发生竞争
        std::mutex tracking_uav_index_mutex;

        std::queue<int> need_track_usvs_id_queue;
        std::mutex need_track_usvs_id_mutex;
        std::condition_variable need_track_usvs_id_cv;

        ros::ServiceServer trackingIdService;

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

        // 获取下一个可得到的tracking uav id
        int getNextTrackingUAVId();

        // 获取被追踪无人机uav id
        int getPreUAVId();

        bool getNeedTrackingUAVIdCallback(uavs_explore_indoor_environment::TrackingUAVId::Request &req,
                                          uavs_explore_indoor_environment::TrackingUAVId::Response &res);
        void trackingUAVCallback();
    };
};