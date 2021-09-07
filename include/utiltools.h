#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <mutex>
#include <condition_variable>

typedef std::pair<int, int> Node;
typedef int Direction;

/**
 * @brief 
 *       left
 *        ↑
 * back ←   → front
 *        ↓
 *      right
 */
enum : Direction
{
    FRONT = 0xFFF0,
    LEFT = 0xFFF1,
    BACK = 0xFFF2,
    RIGHT = 0xFFF3,
    NONE = 0xFFF4
};

struct Position
{
    float x;
    float y;
    float z;

    Position()
    {
        x = y = z = 0;
    }

    Position(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}

    Position(const Position &p)
    {
        x = p.x;
        y = p.y;
        z = p.z;
    }

    Position &operator=(const Position &p)
    {
        if (this == &p)
            return *this;
        x = p.x;
        y = p.y;
        z = p.z;
        return *this;
    }

    bool operator==(const Position &p) const
    {
        if (x == p.x and y == p.y)
            return true;
        return false;
    }

    bool operator!=(const Position &p) const
    {
        if (x == p.x and y == p.y)
            return false;
        return true;
    }

    bool isClose(const Position &p, float distance)
    {
        if (fabs(x - p.x) <= distance and fabs(y - p.y) <= distance)
            return true;
        return false;
    }
};

// 用于多线程之间获取tracking uav的索引避免发生竞争
struct TheardSafe
{
    std::queue<int> need_track_usvs_id_queue;
    std::mutex need_track_usvs_id_mutex;
    std::condition_variable need_track_usvs_id_cv;
};

