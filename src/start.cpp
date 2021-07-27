#include "start.h"

simulation::Simulation::Simulation()
{
    nh = ros::NodeHandle();
    uavs = std::vector<UAVPtr>(NUM_OF_UAV);
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
    std::cout << "width:" << width << std::endl;
    std::cout << "height:" << height << std::endl;
    std::cout << "x_bias:" << x_bias << std::endl;
    std::cout << "y_bias:" << y_bias << std::endl;

    for (int i = 0; i < NUM_OF_UAV; ++i)
    {
        uavs[i] = std::make_shared<UAV::UAV>(init_positons[i], i);
        wall_around_planners[i] = std::make_shared<WallAround::WallAround>();
        Node start_position{getColFromX(int(init_positons[i].x)), getRowFromY(int(init_positons[i].y))};
        wall_around_planners[i]->init(start_position, map_file_path);
        wall_around_planners[i]->setMainDirection(init_direction[i]);
        printf("[UAV%d] initial ", i);
        wall_around_planners[i]->printDirections();
    }
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

void simulation::Simulation::startUAVCallback(int id, Position p)
{
    printf("Start [UAV%d]\n", id);
    UAVPtr curr_uav = uavs[id];
    curr_uav->setStopPosition(end_position);
    curr_uav->initForDrive();
    curr_uav->wallAround(wall_around_planners[id]);
}

void simulation::Simulation::start()
{
    std::vector<std::thread> jobs(NUM_OF_UAV);
    for (int i = 0; i < NUM_OF_UAV; ++i)
        jobs[i] = std::thread(&Simulation::startUAVCallback, this, i, init_positons[i]);
    // jobs[NUM_OF_UAV] = std::thread(&Simulation::trajectoryCallback, this);
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