#include "start.h"

std::mutex tracking_uav_index_mutex;
int sequence_take_off = 1;

// 获取被追踪无人机uav id
int getPreUAVId(TheardSafe *thread_safe)
{
    // printf("Lock...\n");
    // printf("getPreUAVId thread_safe address: %p\n", thread_safe);
    std::unique_lock<std::mutex> lock(thread_safe->need_track_usvs_id_mutex);
    thread_safe->need_track_usvs_id_cv.wait(lock, [&]()
                                            { return !thread_safe->need_track_usvs_id_queue.empty(); });
    int ret = thread_safe->need_track_usvs_id_queue.front();
    thread_safe->need_track_usvs_id_queue.pop();
    // printf("Unlock...\n");
    return ret;
}

float getDistanceBetweenPositions(const Position &p1, const Position &p2)
{
    return sqrt(powf(fabs(p1.x - p2.x), 2) + powf(fabs(p1.y - p2.y), 2));
}

simulation::Simulation::Simulation()
{
    nh = ros::NodeHandle();
    tracking_uavs_num = tracking_uavs_positions.size();
    uavs_num = NUM_OF_UAV + tracking_uavs_num;
    is_uav_used = std::vector<bool>(tracking_uavs_num, false);
    uavs = std::vector<UAVPtr>(uavs_num);
    wall_around_planners = std::vector<WallAroundPlannerPtr>(NUM_OF_UAV);

    std::ifstream mapfile(map_file_path);
    std::string input;
    width = height = 0;
    while (mapfile >> input)
    {
        ++height;
        grid_map.push_back(input);
        width = std::max(width, (int)input.size());
    }
    x_bias = width / 2;
    y_bias = height / 2;

    for (int i = 0; i < NUM_OF_UAV; ++i)
    {
        uavs[i] = std::make_shared<UAV::UAV>(init_uavs_positons[i], i);
        uavs[i]->thread_safe = &m_thread_safe;
        wall_around_planners[i] = std::make_shared<wallaround::WallAround>();
        Node start_position{getColFromX(int(init_uavs_positons[i].x)), getRowFromY(int(init_uavs_positons[i].y))};
        wall_around_planners[i]->init(start_position, map_file_path);
        wall_around_planners[i]->setMainDirection(init_uavs_direction[i]);
        printf("[UAV%d] initial ", i);
        wall_around_planners[i]->printDirections();
    }

    for (int i = NUM_OF_UAV; i < uavs_num; ++i)
    {
        uavs[i] = std::make_shared<UAV::UAV>(tracking_uavs_positions[i - NUM_OF_UAV], i);
        uavs[i]->thread_safe = &m_thread_safe;
    }
    tracking_uavs_index = NUM_OF_UAV;
}

simulation::Simulation::~Simulation() {}

/**
 * @brief 从中心原点转化为左上角原点
 * 
 * @param x 
 * @return int 
 */
int simulation::Simulation::getColFromX(int x)
{
    if (x + x_bias < 0 or x + x_bias > width)
        printf("[ERROR] X out of bound\n");
    return x + x_bias;
}

/**
 * @brief 从中心原点转化为左上角原点
 * 
 * @param y 
 * @return int 
 */
int simulation::Simulation::getRowFromY(int y)
{
    if (y_bias - y < 0 or y_bias - y > height)
        printf("[ERROR] Y out of bound\n");
    return y_bias - y;
}

// 从左上角转化为中心原点
int simulation::Simulation::getXFromCol(int col)
{
    return col - x_bias;
}

// 从左上角转化为中心原点
int simulation::Simulation::getYFromRow(int row)
{
    return y_bias - row;
}

void simulation::Simulation::startUAVCallback(int id)
{
    printf("Start [UAV%d]\n", id);
    UAVPtr curr_uav = uavs[id];
    curr_uav->setStopPosition(end_position);
    curr_uav->initForDrive();
    curr_uav->wallAround(wall_around_planners[id]);
}

int simulation::Simulation::getNextTrackingUAVId(int pre_uav_id)
{
    // if (tracking_uavs_index > uavs_num)
    //     return -1;
    int last_sequence_take_off = sequence_take_off;
    std::lock_guard<std::mutex> lock(tracking_uav_index_mutex);
    int ret = -1;
    float min_distance = 1000;
    for (int i = NUM_OF_UAV; i < uavs_num; ++i)
    {
        if (is_uav_used[i - NUM_OF_UAV])
            continue;
        int temp_distance = getDistanceBetweenPositions(uavs[i]->getUAVCoordinatePosition(),
                                                        uavs[pre_uav_id]->getUAVCoordinatePosition());
        if (min_distance > temp_distance)
        {
            ret = i;
            min_distance = temp_distance;
        }
    }
    is_uav_used[ret - NUM_OF_UAV] = true;
    if (last_sequence_take_off != sequence_take_off)
        sleep(3);
    return ret;
    // int ret = tracking_uavs_index;
    // ++tracking_uavs_index;
    // ++sequence_take_off;
    // return ret;
}

void simulation::Simulation::obstacleMapCallback()
{
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacle_map", 10);
    visualization_msgs::MarkerArray obstacles;
    ros::Rate loop_rate(10);

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (grid_map[y][x] == '#')
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "/simulation_frame";
                marker.header.stamp = ros::Time::now();
                marker.ns = "obstacle_map";
                marker.id = y * width + x;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position.x = getXFromCol(x);
                marker.pose.position.y = getYFromRow(y);
                marker.pose.position.z = 1;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 1.0;
                marker.scale.y = 1.0;
                marker.scale.z = 2.0;

                marker.color.r = 0.5f;
                marker.color.g = 0.5f;
                marker.color.b = 0.5f;
                marker.color.a = 1.0;

                marker.lifetime = ros::Duration();
                obstacles.markers.push_back(marker);
            }
        }
    }
    while (ros::ok())
    {
        marker_pub.publish(obstacles);
        loop_rate.sleep();
    }
}

void simulation::Simulation::trajectoryCallback()
{
    std::vector<ros::Publisher> path_pubs = std::vector<ros::Publisher>(uavs_num);                                // 轨迹发布
    std::vector<nav_msgs::Path> wall_around_msgs = std::vector<nav_msgs::Path>(uavs_num);                         // 轨迹
    std::vector<ros::Publisher> marker_pubs = std::vector<ros::Publisher>(uavs_num);                              // 无人机位置发布
    std::vector<visualization_msgs::Marker> uav_markers_msgs = std::vector<visualization_msgs::Marker>(uavs_num); // 无人机位置
    std::vector<ros::Publisher> tracking_pubs = std::vector<ros::Publisher>(uavs_num);                            // 轨迹发布
    std::vector<nav_msgs::Path> tracking_msgs = std::vector<nav_msgs::Path>(uavs_num);                            // 轨迹

    for (int i = 0; i < uavs_num; ++i)
    {
        std::stringstream ss_path_topic;
        ss_path_topic << "uav" << i << "_path";

        std::stringstream ss_position_topic;
        ss_position_topic << "uav" << i << "_position";

        std::stringstream ss_tracking_topic;
        ss_tracking_topic << "uav" << i << "_tracking";

        path_pubs[i] = nh.advertise<nav_msgs::Path>(ss_path_topic.str(), 10, true);
        tracking_pubs[i] = nh.advertise<nav_msgs::Path>(ss_tracking_topic.str(), 10, true);
        marker_pubs[i] = nh.advertise<visualization_msgs::Marker>(ss_position_topic.str(), 10);

        wall_around_msgs[i].header.stamp = ros::Time::now();
        wall_around_msgs[i].header.frame_id = "/simulation_frame";

        tracking_msgs[i].header.stamp = ros::Time::now();
        tracking_msgs[i].header.frame_id = "/simulation_frame";

        uav_markers_msgs[i].header.stamp = ros::Time::now();
        uav_markers_msgs[i].header.frame_id = "/simulation_frame";
        uav_markers_msgs[i].ns = ss_position_topic.str();
        uav_markers_msgs[i].action = visualization_msgs::Marker::ADD;
        uav_markers_msgs[i].type = visualization_msgs::Marker::SPHERE;
        uav_markers_msgs[i].id = 0;
        uav_markers_msgs[i].pose.orientation.w = 1.0;
        uav_markers_msgs[i].scale.x = 0.2;
        uav_markers_msgs[i].scale.y = 0.2;
        uav_markers_msgs[i].scale.z = 0.2;
        uav_markers_msgs[i].color.r = 1.0f;
        uav_markers_msgs[i].color.a = 1.0;
    }

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        for (int i = 0; i < uavs_num; ++i)
        {
            if (uavs[i]->state == IDLE)
                continue;

            geometry_msgs::PoseStamped temp_path;
            temp_path = uavs[i]->pose;
            uav_markers_msgs[i].pose.position = temp_path.pose.position;
            if (uavs[i]->state == WALL_AROUND)
            {
                if (!tracking_msgs[i].poses.empty())
                {
                    tracking_msgs[i].poses.clear();
                    tracking_pubs[i].publish(tracking_msgs[i]);
                }
                wall_around_msgs[i].poses.push_back(temp_path);
                path_pubs[i].publish(wall_around_msgs[i]);
            }

            if (uavs[i]->state == TRACKING)
            {
                tracking_msgs[i].poses.push_back(temp_path);
                tracking_pubs[i].publish(tracking_msgs[i]);
            }
            marker_pubs[i].publish(uav_markers_msgs[i]);
            ros::spinOnce();
        }
        loop_rate.sleep();
    }
}

void simulation::Simulation::trackingUAVCallback()
{
    int pre_uav_id = getPreUAVId(&m_thread_safe);
    int curr_uav_id = getNextTrackingUAVId(pre_uav_id);

    printf("UAV%d catch UAV%d\n", curr_uav_id, pre_uav_id);
    UAVPtr curr_uav = uavs[curr_uav_id];
    curr_uav->setEntranceStopPosition(uavs[pre_uav_id]->getEntranceStopPosition());
    curr_uav->setStopPosition(end_position);

    Node start_position{getColFromX(int(tracking_uavs_positions[curr_uav_id - NUM_OF_UAV].x)),
                        getRowFromY(int(tracking_uavs_positions[curr_uav_id - NUM_OF_UAV].y))};
    Position pre_uav_position = uavs[pre_uav_id]->getUAVCoordinatePosition();
    Node goal_position{getColFromX(int(pre_uav_position.x)), getRowFromY(int(pre_uav_position.y))};

    TrackingPlannerPtr track_planner = std::make_shared<trackingplanner::TrackingPlanner>();
    track_planner->init(start_position, goal_position, map_file_path);
    curr_uav->initForDrive();

    if (!curr_uav->track(track_planner, uavs[pre_uav_id]))
        return;
    printf("Before [UAV%d] curr_uav_coordinate(%d, %d), curr_uav_real(%f,%f), WallAround(%d,%d)\n", curr_uav_id,
           (int)curr_uav->getUAVCoordinatePosition().x, (int)curr_uav->getUAVCoordinatePosition().y,
           curr_uav->getUAVRealPosition().x, curr_uav->getUAVRealPosition().y,
           getXFromCol(wall_around_planners[pre_uav_id]->getX()), getYFromRow(wall_around_planners[pre_uav_id]->getY()));
    curr_uav->setUAVCoordinatePosition(getXFromCol(wall_around_planners[pre_uav_id]->getX()),
                                       getYFromRow(wall_around_planners[pre_uav_id]->getY()));
    curr_uav->fixTransitionPosition();
    printf("After [UAV%d] curr_uav_coordinate(%d, %d), curr_uav_real(%f,%f), WallAround(%d,%d)\n", curr_uav_id,
           (int)curr_uav->getUAVCoordinatePosition().x, (int)curr_uav->getUAVCoordinatePosition().y,
           curr_uav->getUAVRealPosition().x, curr_uav->getUAVRealPosition().y,
           getXFromCol(wall_around_planners[pre_uav_id]->getX()), getYFromRow(wall_around_planners[pre_uav_id]->getY()));
    curr_uav->wallAround(wall_around_planners[pre_uav_id]);
}

void simulation::Simulation::start()
{
    std::vector<std::thread> jobs(uavs_num + 2);
    for (int i = 0; i < NUM_OF_UAV; ++i)
        jobs[i] = std::thread(&simulation::Simulation::startUAVCallback, this, i);

    for (int i = NUM_OF_UAV; i < uavs_num; ++i)
        jobs[i] = std::thread(&simulation::Simulation::trackingUAVCallback, this);
    jobs[uavs_num] = std::thread(&Simulation::obstacleMapCallback, this);
    jobs[uavs_num + 1] = std::thread(&Simulation::trajectoryCallback, this);
    for (auto &thread : jobs)
        thread.join();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uavs_simulation");
    std::shared_ptr<simulation::Simulation> p_simulation = std::make_shared<simulation::Simulation>();
    p_simulation->start();
    return 0;
}