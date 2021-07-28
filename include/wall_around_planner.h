#pragma once

#include <vector>
#include <fstream>
#include "configuration.h"

namespace wallaround
{
    class WallAround
    {
    private:
        Direction m_main_direction_;
        Direction m_next_direction_;
        Direction m_previous_direction_;
        Direction m_opposite_direction_;
        Node m_position_;
        std::vector<std::string> m_map_;
        int m_width_;
        int m_height_;
        
        char getMapValue(int x, int y) const;
        

    public:
        WallAround();
        ~WallAround();
        Direction getNextTowardDirection();
        std::string getPlannerName();
        void init(const Node &position, const std::string &map);
        Direction getMainDirection();
        void printDirections();
        bool isObstacle(Direction direction);
        void setMainDirection(Direction md);
        void calcuDirections();
        void move(Direction direction);
        int getX() const;
        int getY() const;
        void setX(int x);
        void setY(int y);
    };

};
