#include "tracking_planner.h"

trackingplanner::TrackingPlanner::TrackingPlanner() : width(0), height(0) {}
trackingplanner::TrackingPlanner::~TrackingPlanner() {}

void trackingplanner::TrackingPlanner::init(const Node &nstart, const Node &ngoal, const std::string &map_file_path)
{
    std::ifstream map_file(map_file_path);
    std::string input;

    while (map_file >> input)
    {
        lab.push_back(input);
        width = std::max(width, (int)input.size());
    }
    height = lab.size();

    x_bias = width / 2;
    y_bias = height / 2;

    std::cout << "Tracking-D*lite Map: width=" << width << ", height=" << height << std::endl;
    graph.resize(width * height);
    map_size = width * height;
    start = width * nstart.second + nstart.first;
    goal = width * ngoal.second + ngoal.first;
    std::cout << "Tracking-D*lite start: (" << start % width << ", " << start / width << ")" << std::endl;
    std::cout << "Tracking-D*lite goal: (" << goal % width << ", " << goal / width << ")" << std::endl;

    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            if (lab[i][j] != '.')
                continue;
            int i_new, j_new;
            for (const auto &it : NEIGHTBORS)
            {
                i_new = i + it.first;
                j_new = j + it.second;
                if (i_new > -1 and i_new < height and j_new > -1 and j_new < width)
                {
                    if (lab[i_new][j_new] == '.')
                    {
                        //每个节点存放与自身相邻的无障碍节点
                        graph[getIndex(i, j)].push_back(getIndex(i_new, j_new));
                    }
                }
            }
        }
    }
    last_start = -1;
    // 用无穷大填充v值g值
    rhs.resize(map_size, INF);
    g.resize(map_size, INF);
    parents.resize(map_size, -1);
    // 运行算法
    g[start] = 0;
    OPEN.insert(SetElem(start, getFvalue(start)));
}

int trackingplanner::TrackingPlanner::getIndex(int x, int y)
{
    return (x * width + y);
}

int trackingplanner::TrackingPlanner::getHeruestic(int x, int y)
{
    int x_x = x / width;
    int x_y = x % width;
    int y_x = y / width;
    int y_y = y % width;
    return (abs(x_x - y_x) + abs(x_y - y_y));
}

double trackingplanner::TrackingPlanner::getFvalue(int i)
{
    return std::min((double)INF, g[i] + eps * getHeruestic(i, goal));
}

bool trackingplanner::TrackingPlanner::improvePath()
{
    //OPEN size不为0 并且fvalue（goal）> OPEN中的最小fvalue
    while (OPEN.size() and getFvalue(goal) > getFvalue(OPEN.begin()->id))
    {
        SetElem s = *OPEN.begin(); //取出OPEN中最小的fvale节点（迭代器）存到CLOSE中
        CLOSED.insert(s);
        OPEN.erase(OPEN.begin()); // 删除OPEN中最小值

        rhs[s.id] = g[s.id];
        for (auto new_s : graph[s.id]) //访问当前节点s的邻居
        {
            //s和new_s相邻，所以距离为1
            if (g[new_s] > rhs[s.id] + 1)
            {
                parents[new_s] = s.id; //new_s的父节点为s
                auto next = SetElem(new_s, getFvalue(new_s));
                if (CLOSED.find(next) == CLOSED.end()) //若new_s不在CLOSE中
                {
                    OPEN.erase(next);         //删除OPEN中的new_s,确保不在OPEN中，若存在则删除
                    g[new_s] = rhs[s.id] + 1; //更新g[new_s]
                    if (INCONS.find(next) == INCONS.end())
                    {
                        //第一次发现new_s，INCONS和CLSOE中都没有new_s才加入到OPEN中
                        OPEN.insert(SetElem(new_s, getFvalue(new_s)));
                    }
                    else
                    {
                        INCONS.erase(next);
                        INCONS.insert(SetElem(new_s, getFvalue(new_s)));
                    }
                }
                else
                {
                    CLOSED.erase(SetElem(new_s, getFvalue(new_s)));
                    g[new_s] = rhs[s.id] + 1;
                    INCONS.insert(SetElem(new_s, getFvalue(new_s)));
                }
            }
        }
    }
    if (g[goal] == INF)
        return false;
    return true;
}

void trackingplanner::TrackingPlanner::updateFvalues(std::set<SetElem> &s)
{
    std::set<SetElem> save;
    for (const auto &i : s)
        save.insert(SetElem(i.id, getFvalue(i.id)));
    s = save;
}

bool trackingplanner::TrackingPlanner::computePath()
{
    while (true)
    {
        auto result = improvePath();
        if (result == false)
            return false;
        if (eps == 1)
            return true;

        //将INCONS中所有元素加入到OPEN中
        OPEN.insert(INCONS.begin(), INCONS.end());
        INCONS.clear();
        CLOSED.clear();
        eps = fmax(1, eps - step);
        updateFvalues(OPEN);
    }
}

void trackingplanner::TrackingPlanner::step1()
{
    if (g[start] != rhs[start])
    {
        OPEN.erase(SetElem(start, getFvalue(start)));
        INCONS.erase(SetElem(start, getFvalue(start)));
        g[start] = rhs[start];
    }
}

void trackingplanner::TrackingPlanner::dfsDelete(int current)
{
    INCONS.erase(SetElem(current, getFvalue(current)));
    OPEN.erase(SetElem(current, getFvalue(current)));
    rhs[current] = INF;
    g[current] = INF;
    parents[current] = -1;
    DELETED.insert(current);
    for (auto i : graph[current])
    {
        if (parents[i] == current)
            dfsDelete(i);
    }
}

void trackingplanner::TrackingPlanner::step2()
{
    if (start != last_start)
    {
        parents[start] = -1;
        dfsDelete(last_start);
    }
}

void trackingplanner::TrackingPlanner::step3()
{
    for (auto i : DELETED)
    {
        for (auto j : graph[i])
        {
            if (g[i] > rhs[j] + 1)
            {
                g[i] = rhs[j] + 1;
                parents[i] = j;
            }
        }
        if (g[i] != INF)
        {
            OPEN.insert(SetElem(i, getFvalue(i)));
        }
    }
    updateFvalues(INCONS);
    OPEN.insert(INCONS.begin(), INCONS.end());
    CLOSED.clear();
    INCONS.clear();
    DELETED.clear();
}

void trackingplanner::TrackingPlanner::step4()
{
    if (getFvalue(goal) > OPEN.begin()->fvalue)
        eps = eps_max;
    else
        eps = fmax(eps - step, 1);

    updateFvalues(OPEN);
}

void trackingplanner::TrackingPlanner::getCurentPath()
{
    path.clear();
    int current = goal;
    path.push_back(goal);
    while (current != start)
    {
        current = parents[current];
        path.push_back(current);
    }
    path_len = path.size();
}

int trackingplanner::TrackingPlanner::sum(std::vector<int> &v)
{
    int summ = 0;
    for (auto i : v)
        summ += i;
    return summ;
}

Direction trackingplanner::TrackingPlanner::getDirection()
{
    int last_x = last_start % width;
    int last_y = last_start / width;
    int curr_x = start % width;
    int curr_y = start / width;

    if (last_x == curr_x)
    {
        if (curr_y < last_y)
            return RIGHT;
        else if (curr_y > last_y)
            return LEFT;
    }
    if (last_y == curr_y)
    {
        if (curr_x < last_x)
            return BACK;
        else if (curr_x > last_x)
            return FRONT;
    }
    return NONE;
}

Direction trackingplanner::TrackingPlanner::getNextTowardDirection()
{
    if (start == goal)
        return GETGOAL;
    if (old_goal == goal and !path.empty())
    {
        last_start = start;
        start = *(path.rbegin() + 1);
        path.pop_back();
        return getDirection();
    }
    if (computePath() == false)
    {
        std::cout << "No Path!" << std::endl;
        return NOPATH;
    }

    //获取找到的路径
    getCurentPath();
    last_start = start;
    start = *(path.rbegin() + 1);
    old_goal = goal;
    path.pop_back();
    if (start == goal)
    {
        std::cout << "Get target!" << std::endl;
        return GETGOAL;
    }
    updateFvalues(OPEN);
    step1();
    step2();
    step3();
    step4();
    return getDirection();
}

void trackingplanner::TrackingPlanner::updateNextTarget(int x, int y)
{
    goal = getRowFromCoordinate(y) * width + getColFromCoordinate(x);
}

std::string trackingplanner::TrackingPlanner::getPlannerName()
{
    return "Tracking-D*Lite";
}

int trackingplanner::TrackingPlanner::getColFromCoordinate(int x)
{
    if (x + x_bias < 0 or x + x_bias >= width)
        printf("[ERROR] X out of bound\n");
    return x + x_bias;
}
int trackingplanner::TrackingPlanner::getRowFromCoordinate(int y)
{
    if (y_bias - y < 0 or y_bias - y >= height)
        printf("[ERROR] Y out of bound\n");
    return y_bias - y;
}