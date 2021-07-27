#pragma once

#include <string>
#include "configuration.h"


class BasePlanner
{
public:
    // virtual bool init(Node nstart, Node ngoal, const char *map) = 0;
    virtual Direction getNextPosition() = 0;
    virtual std::string getPlannerName() = 0;
    virtual void updateNextTarget(int x, int y);
};
