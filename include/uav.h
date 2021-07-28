#pragma once

#include "uavs_explore_indoor_environment/EnableMotors.h"
#include "uavs_explore_indoor_environment/EntrancePosition.h"
#include "uavs_explore_indoor_environment/TrackingUAVId.h"
#include "uav_exception.h"
#include "wall_around_planner.h"
#include "tracking_planner.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <memory>
#include <tf/transform_datatypes.h>


typedef int UAVState;
typedef std::shared_ptr<wallaround::WallAround> WallAroundPlannerPtr;
typedef std::shared_ptr<trackingplanner::TrackingPlanner> TrackingPlannerPtr;


enum : UAVState
{
    TRACKING = 0xFFF5,
    WALL_AROUND = 0xFFF6,
    IDLE = 0xFFF7,
    FLYING = 0xFFF8
};

//话题及服务名称
const std::string controller_topic_name = "/cmd_vel";
const std::string position_topic_name = "/ground_truth_to_tf/pose";
const std::string entrance_position_topic_name = "/entrance_position";
const std::string motor_service_name = "/enable_motors";
const std::string tracking_uavs_service_name = "/tracking_uav";

namespace UAV
{
    class UAV
    {
    private:
        ros::NodeHandle m_nh_;
        ros::NodeHandle m_nh_private_;

        ros::Publisher m_cmd_vel_publisher_;             //用于发布cmd_vel话题
        ros::ServiceClient m_motor_client_;              //用于驱动无人机马达的服务客户端
        ros::Subscriber m_uav_position_subscriber_;      //用于订阅获取无人机位置的话题
        ros::Publisher m_entrance_position_publisher_;   //用于发布进入边界点
        ros::Subscriber m_entrance_position_subscriber_; //用于订阅上一架UAV进入边界点
        ros::ServiceClient m_tracking_uav_client_;       //将当前无人机id作为被追击无人机发送至总控制

        //无人机编号
        int m_id_;

        //前一架无人机
        int m_before_uav_id_;

        //Twist 变量
        float m_x_;
        float m_y_;
        float m_z_;
        float m_th_;
        float m_speed_;
        float m_turn_;
        double m_yaw_;

        //电量
        int m_battery_;

        //循环频率
        int m_rate_;

        //话题消息
        geometry_msgs::Twist m_msg_;

        //无人机机头朝向
        int m_head_toward_;

        //无人机实际位置
        Position m_uav_real_position_;
        //无人机坐标位置
        Position m_uav_coordinate_position_;
        //路径规划的目标点
        Position m_target_position_;
        //停止点
        Position m_stop_postion_;
        //前一架UAV进入口停止点
        Position m_entrance_stop_postion_;
        //当前无人机进入边界的位置
        Position m_entrance_position_;

        // 是否有需要追逐的目标
        bool m_is_catch_;

    public:
        //无人机当前运行状态
        int m_uav_state;

        // 无人机当前位姿
        geometry_msgs::PoseStamped m_uav_pose_msgs;

        UAV(const Position &init_position, int id);
        ~UAV();

        void initForDrive();
        void setTarget(const Position &position);

        Position getUAVRealPosition() const;
        Position getUAVCoordinatePosition() const;
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void entrancePositionCallback(const uavs_explore_indoor_environment::EntrancePosition::ConstPtr &entrance_position_msg);
        void publishEntrancePosition();
        void setBattery(int b);
        int getBattery();
        int getID();

        void sendCurrentUAVId();

        void calcuNextPosition(Direction d);
        Position getNextPsotion(Direction d);

        Position getEntranceStopPosition();
        void setEntranceStopPosition(const Position &position);

        void waitForSubscribles();

        // 操作无人机
        void rotateUAV(double angular, ros::Rate &loop_rate);
        void moveUAV(float distance, ros::Rate &loop_rate, int power);
        void hoverUAV(ros::Rate &loop_rate);

        // 修正无人机路线和旋转角度
        void fixUAVRoute(ros::Rate &loop_rate);
        void fixUAVAngle(ros::Rate &loop_rate);

        void driveByDirection(Direction direction, ros::Rate &loop_rate);
        void goToBoundary(const WallAroundPlannerPtr &planner, const Direction &direction, ros::Rate &loop_rate);

        // 无人际根据不同的算法进行导航
        void wallAround(const WallAroundPlannerPtr &planner);
        bool track(const TrackingPlannerPtr &planner, const std::shared_ptr<UAV> &pre_uav);

        // 开启或关闭无人机马达
        void enableMotors();
        void disableMotors();

        // 停止无人机
        void stop();

        // 搜索完毕时的停止点
        void setStopPosition(const Position &stop_position);

        // 电源消耗
        bool powerConsumption(int percent);

        //是否在飞行
        bool isFlying();
    };
};
typedef std::shared_ptr<UAV::UAV> UAVPtr;