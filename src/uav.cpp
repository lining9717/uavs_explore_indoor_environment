#include "uav.h"

UAV::UAV::UAV(const Position &init_position, int id)
{
    m_nh_ = ros::NodeHandle();
    m_nh_private_ = ros::NodeHandle("~");
    m_id_ = id;
    m_before_uav_id_ = (m_id_ + 1) % NUM_OF_UAV;

    std::stringstream ss_cmd_vel_name;
    ss_cmd_vel_name << "/uav" << m_id_ << controller_topic_name;

    std::stringstream ss_position_topic_name;
    ss_position_topic_name << "/uav" << m_id_ << position_topic_name;

    std::stringstream ss_motor_service_name;
    ss_motor_service_name << "/uav" << m_id_ << motor_service_name;

    std::stringstream ss_publish_entrance_position_topic_name;
    ss_publish_entrance_position_topic_name << "/uav" << m_id_ << entrance_position_topic_name;

    std::stringstream ss_subscrible_entrance_position_topic_name;
    ss_subscrible_entrance_position_topic_name << "/uav" << m_before_uav_id_ << entrance_position_topic_name;

    m_cmd_vel_publisher_ = m_nh_.advertise<geometry_msgs::Twist>(ss_cmd_vel_name.str(), 1);
    m_uav_position_subscriber_ = m_nh_.subscribe(ss_position_topic_name.str(), 10, &UAV::positionCallback, this);
    m_motor_client_ = m_nh_.serviceClient<uavs_explore_indoor_environment::EnableMotors>(ss_motor_service_name.str());
    m_entrance_position_publisher_ = m_nh_.advertise<uavs_explore_indoor_environment::EntrancePosition>(ss_publish_entrance_position_topic_name.str(), 10);
    m_entrance_position_subscriber_ = m_nh_.subscribe(ss_subscrible_entrance_position_topic_name.str(), 10, &UAV::entrancePositionCallback, this);
    m_tracking_uav_client_ = m_nh_.serviceClient<uavs_explore_indoor_environment::TrackingUAVId>(tracking_uavs_service_name);

    m_x_ = 0.0;
    m_y_ = 0.0;
    m_z_ = 0.0;
    m_th_ = 0.0;
    m_speed_ = 1.0;
    m_turn_ = 2.0;

    m_uav_real_position_ = init_position;
    m_uav_coordinate_position_ = init_position;
    m_entrance_position_ = UNDEFINED;
    m_entrance_stop_postion_ = UNDEFINED;

    m_head_toward_ = FRONT;
    m_rate_ = 1;
    m_battery_ = FULL_BATTERY;
    m_uav_state = IDLE;
}

UAV::UAV::~UAV() {}

void UAV::UAV::initForDrive()
{
    try
    {
        waitForSubscribles();
        enableMotors();
        ros::Rate loop_rate(m_rate_);
        m_msg_.angular.x = 0;
        m_msg_.angular.y = 0;
        m_msg_.angular.z = 0;
        m_uav_coordinate_position_.z = 1.0;

        //起飞, 距离地面1m
        m_msg_.linear.x = 0;
        m_msg_.linear.y = 0;
        m_msg_.linear.z = m_speed_;
        moveUAV(2, loop_rate, 0);
        hoverUAV(loop_rate);

        fixUAVAngle(loop_rate);
        hoverUAV(loop_rate);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void UAV::UAV::setBattery(int b)
{
    m_battery_ = b;
}

int UAV::UAV::getBattery()
{
    return m_battery_;
}

int UAV::UAV::getID()
{
    return m_id_;
}

bool UAV::UAV::isFlying()
{
    return m_uav_state != IDLE;
}

void UAV::UAV::setTarget(const Position &position)
{
    m_target_position_ = position;
    m_is_catch_ = true;
}

/**
 * @brief 停止无人机
 * 
 */
void UAV::UAV::stop()
{
    m_msg_.linear.x = 0;
    m_msg_.linear.y = 0;
    m_msg_.linear.z = 0;
    m_msg_.angular.x = 0;
    m_msg_.angular.y = 0;
    m_msg_.angular.z = 0;
    m_cmd_vel_publisher_.publish(m_msg_);
    disableMotors();
}
/**
 * @brief 获取无人机实时位置的回调函数
 * 
 * @param msg 
 */
void UAV::UAV::positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    m_uav_real_position_.x = msg->pose.position.x;
    m_uav_real_position_.y = msg->pose.position.y;
    m_uav_real_position_.z = msg->pose.position.z;
    m_yaw_ = tf::getYaw(msg->pose.orientation) * 180 / M_PI;
    m_uav_pose_msgs = *msg;
}

/**
 * @brief 获取前一架无人机进入边界的位置
 * 
 * @param entrance_position_msg 
 */
void UAV::UAV::entrancePositionCallback(const uavs_explore_indoor_environment::EntrancePosition::ConstPtr &entrance_position_msg)
{
    if (m_entrance_stop_postion_.z != UNDEFINED_POSITION)
        return;
    if (entrance_position_msg->z == UNDEFINED_POSITION)
        return;
    m_entrance_stop_postion_ = {entrance_position_msg->x,
                                entrance_position_msg->y,
                                entrance_position_msg->z};
    printf("[UAV%d] get the UAV%d stop entrance position(%f,%f,%f)\n", m_id_, m_before_uav_id_, m_entrance_stop_postion_.x, m_entrance_stop_postion_.y, m_entrance_stop_postion_.z);
    m_entrance_position_subscriber_.shutdown();
}

/**
 * @brief 用于发布当前无人机进入边界的位置
 * 
 */
void UAV::UAV::publishEntrancePosition()
{
    uavs_explore_indoor_environment::EntrancePosition msg;
    msg.x = m_entrance_position_.x;
    msg.y = m_entrance_position_.y;
    msg.z = m_entrance_position_.z;
    m_entrance_position_publisher_.publish(msg);
    ros::spinOnce();
}

/**
 * @brief 用于/cmd_vel话题的等待订阅 
 * 
 */
void UAV::UAV::waitForSubscribles()
{
    int count = 0;
    while (ros::ok() and m_cmd_vel_publisher_.getNumSubscribers() == 0)
    {
        if (count == 4)
            printf("[UAV%d] waiting for subscriber to connect /uav%d/%s.\n", m_id_, m_id_, controller_topic_name.c_str());
        sleep(1);
        ++count;
        count %= 5;
    }
    if (!ros::ok())
        throw UAVException("Got shutdown request before subscribers connected");
}

/**
 * @brief 开启无人机的马达
 * 
 */
void UAV::UAV::enableMotors()
{
    uavs_explore_indoor_environment::EnableMotors srv;
    srv.request.enable = true;
    if (m_motor_client_.call(srv))
    {
        if (srv.response.success)
            printf("[UAV%d] Success to enable motors\n", m_id_);
        else
            printf("[UAV%d] Fail to enable motors\n", m_id_);
    }
    else
    {
        std::stringstream ss;
        ss << "Failed to call service /uav" << m_id_ << "/" << motor_service_name;
        throw UAVException(ss.str().c_str());
    }
}

/**
 * @brief 关闭无人机的马达
 * 
 */
void UAV::UAV::disableMotors()
{
    uavs_explore_indoor_environment::EnableMotors srv;
    srv.request.enable = false;
    if (m_motor_client_.call(srv))
    {
        if (srv.response.success)
            printf("[UAV%d] Success to disable motors\n", m_id_);
        else
            printf("[UAV%d] Fail to disable motors\n", m_id_);
    }
    else
    {
        std::stringstream ss;
        ss << "Failed to call service /uav" << m_id_ << "/enable_motors";
        throw UAVException(ss.str().c_str());
    }
}

/**
 * @brief 发送当前无人机的ID至总控程序
 * 
 */
void UAV::UAV::sendCurrentUAVId()
{
    uavs_explore_indoor_environment::TrackingUAVId srv;
    srv.request.id = m_id_;
    if (m_tracking_uav_client_.call(srv))
    {
        if (srv.response.result)
            printf("[UAV%d] Success send tracking id to service\n", m_id_);
        else
            printf("[UAV%d] Fail send tracking id to service\n", m_id_);
    }
    else
    {
        std::stringstream ss;
        ss << "Failed to call service /uav" << m_id_ << tracking_uavs_service_name;
        throw UAVException(ss.str().c_str());
    }
}

void UAV::UAV::setStopPosition(const Position &stop_position)
{
    this->m_stop_postion_ = stop_position;
}

Position UAV::UAV::getEntranceStopPosition()
{
    return m_entrance_stop_postion_;
}

void UAV::UAV::setEntranceStopPosition(const Position &position)
{
    m_entrance_stop_postion_ = position;
}

/**
 * @brief 移动无人机
 * 
 * @param distance  距离
 * @param loop_rate  频率
 */
void UAV::UAV::moveUAV(float distance, ros::Rate &loop_rate, int power)
{
    int ticks = int(distance / m_speed_ * m_rate_);
    for (int i = 0; i < ticks; ++i)
    {
        powerConsumption(power);
        ros::spinOnce();
        m_cmd_vel_publisher_.publish(m_msg_);
        loop_rate.sleep();
    }
}

/**
 * @brief 悬浮无人机
 * 
 * @param loop_rate 
 */
void UAV::UAV::hoverUAV(ros::Rate &loop_rate)
{
    if (m_uav_real_position_.z < m_uav_coordinate_position_.z)
    {
        m_msg_.linear.x = 0;
        m_msg_.linear.y = 0;
        m_msg_.linear.z = m_speed_;
        m_msg_.angular.x = 0;
        m_msg_.angular.y = 0;
        m_msg_.angular.z = 0;
        int ticks = int((m_uav_coordinate_position_.z - m_uav_real_position_.z) / m_speed_ * m_rate_);
        ticks = ticks == 0 ? 1 : ticks;
        for (int i = 0; i < ticks; ++i)
        {
            // powerConsumption(1);
            ros::spinOnce();
            m_cmd_vel_publisher_.publish(m_msg_);
            loop_rate.sleep();
        }
    }
    else
    {
        m_msg_.linear.x = 0;
        m_msg_.linear.y = 0;
        m_msg_.linear.z = 0;
        m_msg_.angular.x = 0;
        m_msg_.angular.y = 0;
        m_msg_.angular.z = 0;
        m_cmd_vel_publisher_.publish(m_msg_);
        loop_rate.sleep();
    }
}

/**
 * @brief 旋转无人机
 * 
 * @param angular  角度
 * @param loop_rate  频率
 */
void UAV::UAV::rotateUAV(double angular, ros::Rate &loop_rate)
{
    m_msg_.linear.x = 0;
    m_msg_.linear.y = 0;
    m_msg_.linear.z = 0;
    m_msg_.angular.x = 0;
    m_msg_.angular.y = 0;
    m_msg_.angular.z = angular > 0 ? m_turn_ : -m_turn_;
    int ticks = int(abs(angular) / m_turn_ * m_rate_);
    for (int i = 0; i < ticks; ++i)
    {
        powerConsumption(1);
        m_cmd_vel_publisher_.publish(m_msg_);
        loop_rate.sleep();
    }
}

/**
 * @brief 电源消耗
 * 
 * @param percent 减少幅度
 * @return true  0 < battery <= BATTERY_THRESHOLD
 * @return false battery=0
 */
bool UAV::UAV::powerConsumption(int percent)
{
    m_battery_ -= percent;

    if (m_battery_ > 0 and m_battery_ <= BATTERY_THRESHOLD)
    {
        printf("[UAV%d] BATTERY STATE: Low %d%%\n", m_id_, m_battery_);
        return true;
    }
    else if (m_battery_ > BATTERY_THRESHOLD)
    {
        printf("[UAV%d] BATTERY STATE: %d%%\n", m_id_, m_battery_);
        return true;
    }
    else
    {
        printf("[UAV%d] BATTERY STATE: Empty 0%%\n", m_id_);
        return false;
    }
}

Position UAV::UAV::getUAVRealPosition() const
{
    return m_uav_real_position_;
}

Position UAV::UAV::getUAVCoordinatePosition() const
{
    return m_uav_coordinate_position_;
}

void UAV::UAV::fixUAVAngle(ros::Rate &loop_rate)
{
    if (fabs(m_yaw_) > 3)
    {
        m_msg_.linear.y = 0;
        m_msg_.linear.z = 0;
        m_msg_.linear.x = 0;
        ros::Rate loopRate(10);
        // printf("[UAV%d] fix yaw\n", id);

        int ticks = int(3 / 0.1 * 10);
        for (int i = 0; i < ticks and fabs(m_yaw_) > 0.5; ++i)
        {
            ros::spinOnce();
            // printf("\033[47;31m [UAV%d] yaw: %f\033[0m\n", id, yaw);
            m_msg_.angular.z = m_yaw_ > 0 ? -0.1 : 0.1;
            m_cmd_vel_publisher_.publish(m_msg_);
            loopRate.sleep();
        }
    }
}

void UAV::UAV::fixUAVRoute(ros::Rate &loop_rate)
{
    // if ((FRONT == next_direction or BACK == next_direction) and
    //     fabs(m_uav_real_position_.x - m_uav_coordinate_position_.x) >= 0.3)
    // {
    //     printf("----------[UAV%d] fix x axis ------------\n", m_id_);
    //     printf("[UAV%d] real position(%f, %f)\n", m_id_, m_uav_real_position_.x, m_uav_real_position_.y);
    //     printf("[UAV%d] coordinate position(%f, %f)\n", m_id_, m_uav_coordinate_position_.x, m_uav_coordinate_position_.y);
    //     m_msg_.linear.y = 0;
    //     m_msg_.linear.z = 0;
    //     m_msg_.linear.x = m_uav_real_position_.x - m_uav_coordinate_position_.x >= 0.3 ? -0.5 : 0.5;
    //     while (ros::ok() and fabs(m_uav_real_position_.x - m_uav_coordinate_position_.x) > 0.1)
    //     {
    //         ros::spinOnce();
    //         m_cmd_vel_publisher_.publish(m_msg_);
    //     }
    // }
    // else if ((LEFT == next_direction or RIGHT == next_direction) and
    //          fabs(m_uav_real_position_.y - m_uav_coordinate_position_.y) >= 0.3)
    // {
    //     printf("----------[UAV%d] fix y axis ------------\n", m_id_);
    //     printf("[UAV%d] real position(%f, %f)\n", m_id_, m_uav_real_position_.x, m_uav_real_position_.y);
    //     printf("[UAV%d] coordinate position(%f, %f)\n", m_id_, m_uav_coordinate_position_.x, m_uav_coordinate_position_.y);
    //     m_msg_.linear.x = 0;
    //     m_msg_.linear.z = 0;
    //     m_msg_.linear.y = m_uav_real_position_.y - m_uav_coordinate_position_.y >= 0.3 ? -0.5 : 0.5;
    //     while (ros::ok() and fabs(m_uav_real_position_.y - m_uav_coordinate_position_.y) > 0.1)
    //     {
    //         ros::spinOnce();
    //         m_cmd_vel_publisher_.publish(m_msg_);
    //     }
    // }

    if (m_uav_real_position_.isClose(m_uav_coordinate_position_, 0.5))
        return;
    // x方向的修正
    while (ros::ok() and fabs(m_uav_real_position_.x - m_uav_coordinate_position_.x) > 0.1)
    {
        ros::spinOnce();
        // printf("[UAV%d] -----> fix x real_position(%f, %f)\n", m_id_, m_uav_real_position_.x, m_uav_real_position_.y);
        // printf("[UAV%d] -----> fix x coordinate_position(%f, %f)\n\n", m_id_, m_uav_coordinate_position_.x, m_uav_coordinate_position_.y);
        m_msg_.linear.x = m_uav_real_position_.x - m_uav_coordinate_position_.x > 0 ? -0.1 : 0.1;
        m_msg_.linear.z = 0;
        m_msg_.linear.y = 0;
        m_cmd_vel_publisher_.publish(m_msg_);
        loop_rate.sleep();
    }

    // y方向的修正
    while (ros::ok() and fabs(m_uav_real_position_.y - m_uav_coordinate_position_.y) > 0.1)
    {
        ros::spinOnce();
        // printf("[UAV%d] -----> fix y real_position(%f, %f)\n", m_id_, m_uav_real_position_.x, m_uav_real_position_.y);
        // printf("[UAV%d] -----> fix y coordinate_position(%f, %f)\n\n", m_id_, m_uav_coordinate_position_.x, m_uav_coordinate_position_.y);
        m_msg_.linear.y = m_uav_real_position_.y - m_uav_coordinate_position_.y > 0 ? -0.1 : 0.1;
        m_msg_.linear.z = 0;
        m_msg_.linear.x = 0;
        m_cmd_vel_publisher_.publish(m_msg_);
        loop_rate.sleep();
    }
}

/**
 * @brief 通过方向计算下一次移动坐标
 * 
 * @param d 
 * @return Position 
 */
void UAV::UAV::calcuNextPosition(Direction d)
{
    if (d == FRONT)
        m_uav_coordinate_position_.x += 1;
    else if (d == BACK)
        m_uav_coordinate_position_.x -= 1;
    else if (d == LEFT)
        m_uav_coordinate_position_.y += 1;
    else
        m_uav_coordinate_position_.y -= 1;
    // printf("[UAV%d] Next position(%f, %f)\n", m_id_, m_uav_coordinate_position_.x, m_uav_coordinate_position_.y);
}

/**
 * @brief 通过方向计算下一次移动坐标
 * 
 * @param d 
 * @return Position 
 */
Position UAV::UAV::getNextPsotion(Direction d)
{
    Position temp = m_uav_coordinate_position_;
    if (d == FRONT)
        temp.x += 1;
    else if (d == BACK)
        temp.x -= 1;
    else if (d == LEFT)
        temp.y += 1;
    else
        temp.y -= 1;
    return temp;
}

void UAV::UAV::driveByDirection(Direction direction, ros::Rate &loop_rate)
{
    if (direction == FRONT)
    {
        m_msg_.linear.x = m_speed_;
        m_msg_.linear.y = 0;
        m_msg_.linear.z = 0;
    }
    else if (direction == BACK)
    {
        m_msg_.linear.x = -m_speed_;
        m_msg_.linear.y = 0;
        m_msg_.linear.z = 0;
    }
    else if (direction == RIGHT)
    {
        m_msg_.linear.x = 0;
        m_msg_.linear.y = -m_speed_;
        m_msg_.linear.z = 0;
    }
    else
    {
        m_msg_.linear.x = 0;
        m_msg_.linear.y = m_speed_;
        m_msg_.linear.z = 0;
    }
    moveUAV(1.0, loop_rate, 1);
    hoverUAV(loop_rate);
}

/**
 * @brief 靠近边界
 * 
 * @param direction 
 * @param loop_rate 
 */
void UAV::UAV::goToBoundary(const WallAroundPlannerPtr &planner, const Direction &direction, ros::Rate &loop_rate)
{
    int distance = 0;
    while (!planner->isObstacle(direction))
    {
        planner->move(direction);
        ++distance;
    }
    if (direction == FRONT)
        m_uav_coordinate_position_.x += distance;
    else if (direction == BACK)
        m_uav_coordinate_position_.x -= distance;
    else if (direction == RIGHT)
        m_uav_coordinate_position_.y -= distance;
    else
        m_uav_coordinate_position_.y += distance;
    while (ros::ok() and distance > 0)
    {
        if (direction == FRONT)
        {
            m_msg_.linear.x = m_speed_;
            m_msg_.linear.y = 0;
            m_msg_.linear.z = 0;
        }
        else if (direction == BACK)
        {
            m_msg_.linear.x = -m_speed_;
            m_msg_.linear.y = 0;
            m_msg_.linear.z = 0;
        }
        else if (direction == RIGHT)
        {
            m_msg_.linear.x = 0;
            m_msg_.linear.y = -m_speed_;
            m_msg_.linear.z = 0;
        }
        else
        {
            m_msg_.linear.x = 0;
            m_msg_.linear.y = m_speed_;
            m_msg_.linear.z = 0;
        }
        moveUAV(1.0, loop_rate, 1);
        hoverUAV(loop_rate);
        --distance;
    }
    fixUAVAngle(loop_rate);
    hoverUAV(loop_rate);
}

void prinDirection(Direction d)
{
    switch (d)
    {
    case FRONT:
        printf("  FRONT\n");
        break;
    case BACK:
        printf("  BACK\n");
        break;
    case RIGHT:
        printf("  RIGHT\n");
        break;
    case LEFT:
        printf("  LEFT\n");
        break;
    default:
        break;
    }
}

/**
 * @brief 用于绕边飞行
 * 
 * @param planner 
 */
void UAV::UAV::wallAround(const WallAroundPlannerPtr &planner)
{
    m_uav_state = WALL_AROUND;
    ros::Rate loop_rate(m_rate_);
    bool is_initial = true;
    bool is_send_id = false;
    try
    {
        while (ros::ok())
        {
            ros::spinOnce();
            if (m_entrance_stop_postion_ == m_uav_real_position_ or
                m_uav_real_position_.isClose(m_entrance_stop_postion_, 0.5))
            {
                printf("[UAV%d]  Finish by m_entrance_stop_postion_!\n", m_id_);
                break;
            }
            if (m_stop_postion_ == m_uav_real_position_)
            {
                printf("[UAV%d]  Finish by m_stop_postion_!\n", m_id_);
                break;
            }
            if (is_initial) // 起飞时进行初始化，导向最近的边界，并确定主方向
            {
                Direction towads_direction = planner->getMainDirection();
                goToBoundary(planner, towads_direction, loop_rate);
                m_entrance_position_ = m_uav_real_position_;
                is_initial = false;
            }
            if (m_entrance_position_publisher_.getNumSubscribers() != 0)
                publishEntrancePosition();
            Direction next_direction = planner->getNextTowardDirection(); //获取下一次前进的方向

            fixUAVRoute(loop_rate);
            calcuNextPosition(next_direction);

            driveByDirection(next_direction, loop_rate);
            fixUAVAngle(loop_rate);
            hoverUAV(loop_rate);

            if (m_battery_ <= BATTERY_THRESHOLD and !is_send_id)
            {
                sendCurrentUAVId();
                is_send_id = true;
            }

            if (m_battery_ <= 0)
            {
                printf("[UAV%d] Wallaround stop by battery empty\n", m_id_);
                break;
            }
        }
        stop();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    m_uav_state = IDLE;
}

/**
 * @brief 用于追击前一架无人机
 * 
 * @param planner 
 * @param pre_uav 
 */
bool UAV::UAV::track(const TrackingPlannerPtr &planner, const std::shared_ptr<UAV> &pre_uav)
{
    m_uav_state = TRACKING;
    Position moving_target;
    ros::Rate loop_rate(m_rate_);
    try
    {
        while (ros::ok())
        {
            ros::spinOnce(); //获取无人机位置
            moving_target = pre_uav->getUAVCoordinatePosition();
            printf("[UAV%d] get target UAV%d position(%d, %d)\n", m_id_, pre_uav->getID(), (int)moving_target.x, (int)moving_target.y);

            planner->updateNextTarget(int(moving_target.x), int(moving_target.y));
            Direction next_direction = planner->getNextTowardDirection(); //获取下一次前进的方向

            if (next_direction == NOPATH)
                throw UAVException("No path");
            if (next_direction == GETGOAL)
            {
                printf("[UAV%d] Get the goal!\n", m_id_);
                return true;
            }
            Position next_position = getNextPsotion(next_direction);
            if (pre_uav->isFlying() and
                fabs(next_position.x - moving_target.x) + fabs(next_position.y - moving_target.y) < 2)
            {
                printf("[UAV%d] stoped by UAV%d\n", pre_uav->getID(), m_id_);
                pre_uav->stop();
                return true;
            }
            printf("[UAV%d] next tracking position(%d, %d)\n", m_id_, int(next_position.x), int(next_position.y));
            fixUAVRoute(loop_rate);
            m_uav_coordinate_position_ = next_position;
            driveByDirection(next_direction, loop_rate);
            fixUAVAngle(loop_rate);
            hoverUAV(loop_rate);

            if (m_battery_ <= 0)
            {
                printf("[UAV%d] Tracking stop by battery empty\n", m_id_);
                break;
            }
        }
        stop();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    m_uav_state = IDLE;
    return false;
}