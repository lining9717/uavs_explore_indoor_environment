#include "start.h"


std::mutex tracking_uav_index_mutex;

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

simulation::Simulation::Simulation()
{
    nh = ros::NodeHandle();
    tracking_uavs_num = tracking_uavs_positions.size();
    uavs_num = NUM_OF_UAV + tracking_uavs_num;

    uavs = std::vector<UAVPtr>(uavs_num);
    wall_around_planners = std::vector<WallAroundPlannerPtr>(NUM_OF_UAV);

    std::ifstream mapfile(map_file_path);
    std::string input;
    width = height = 0;
    while (mapfile >> input)
    {
        ++height;
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

void simulation::Simulation::startUAVCallback(int id)
{
    printf("Start [UAV%d]\n", id);
    UAVPtr curr_uav = uavs[id];
    curr_uav->setStopPosition(end_position);
    curr_uav->initForDrive();
    curr_uav->wallAround(wall_around_planners[id]);
}

int simulation::Simulation::getNextTrackingUAVId()
{
    if (tracking_uavs_index > uavs_num)
        return -1;
    std::lock_guard<std::mutex> lock(tracking_uav_index_mutex);
    int ret = tracking_uavs_index;
    ++tracking_uavs_index;
    return ret;
}

void simulation::Simulation::trackingUAVCallback()
{
    int pre_uav_id = getPreUAVId(&m_thread_safe);
    int curr_uav_id = getNextTrackingUAVId();

    UAVPtr curr_uav = uavs[curr_uav_id];
    curr_uav->setEntranceStopPosition(uavs[pre_uav_id]->getEntranceStopPosition());
    curr_uav->setStopPosition(end_position);
    Node start_position{getColFromX(int(tracking_uavs_positions[curr_uav_id - NUM_OF_UAV].x)),
                        getRowFromY(int(tracking_uavs_positions[curr_uav_id - NUM_OF_UAV].y))};
    TrackingPlannerPtr track_planner = std::make_shared<trackingplanner::TrackingPlanner>();
    track_planner->init(start_position, start_position, map_file_path);
    curr_uav->initForDrive();
    if (!curr_uav->track(track_planner, uavs[pre_uav_id]))
        return;
    curr_uav->wallAround(wall_around_planners[pre_uav_id]);
}

void simulation::Simulation::start()
{
    std::vector<std::thread> jobs(uavs_num);
    for (int i = 0; i < NUM_OF_UAV; ++i)
        jobs[i] = std::thread(&simulation::Simulation::startUAVCallback, this, i);

    for (int i = NUM_OF_UAV; i < uavs_num; ++i)
        jobs[i] = std::thread(&simulation::Simulation::trackingUAVCallback, this);

    // jobs[uavs_num] = std::thread(&Simulation::trackingThreadCallback, this);
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