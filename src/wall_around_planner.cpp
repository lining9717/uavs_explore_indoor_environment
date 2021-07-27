#include "wall_around_planner.h"

Direction getNextDirection(Direction d)
{
    switch (d)
    {
    case FRONT:
        return LEFT;
    case LEFT:
        return BACK;
    case BACK:
        return RIGHT;
    case RIGHT:
        return FRONT;
    default:
        return 0;
    }
}

Direction getPreDirection(Direction d)
{
    switch (d)
    {
    case FRONT:
        return RIGHT;
    case LEFT:
        return FRONT;
    case BACK:
        return LEFT;
    case RIGHT:
        return BACK;
    default:
        return 0;
    }
}

Direction getOppositeDirection(Direction d)
{
    switch (d)
    {
    case FRONT:
        return BACK;
    case LEFT:
        return RIGHT;
    case BACK:
        return FRONT;
    case RIGHT:
        return LEFT;
    default:
        return 0;
    }
}

WallAround::WallAround::WallAround()
{
    m_main_direction_ = NONE;
    m_next_direction_ = NONE;
    m_previous_direction_ = NONE;
    m_opposite_direction_ = NONE;
    m_position_ = {0, 0};
    m_width_ = 0;
    m_height_ = 0;
}

WallAround::WallAround::~WallAround() {}

std::string WallAround::WallAround::getPlannerName()
{
    return "WallAroundPlanner";
}

void WallAround::WallAround::init(const Node &position, const std::string &map)
{
    std::ifstream map_file(map);
    std::string input_line;
    while (map_file >> input_line)
    {
        m_map_.emplace_back(input_line);
        m_width_ = std::max(m_width_, (int)input_line.size());
    }
    m_height_ = m_map_.size();
    m_position_ = position;
    std::cout << "Wall Around Position(" << position.first << "," << position.second << ")" << std::endl;
}

Direction WallAround::WallAround::getMainDirection()
{
    return m_main_direction_;
}

void WallAround::WallAround::setMainDirection(Direction md)
{
    m_main_direction_ = md;
    m_next_direction_ = getNextDirection(m_main_direction_);
    m_previous_direction_ = getPreDirection(m_main_direction_);
    m_opposite_direction_ = getOppositeDirection(m_main_direction_);
}

int WallAround::WallAround::getX() const
{
    return m_position_.first;
}

int WallAround::WallAround::getY() const
{
    return m_position_.second;
}

void WallAround::WallAround::setX(int x)
{
    m_position_.first = x;
}

void WallAround::WallAround::setY(int y)
{
    m_position_.second = y;
}

char WallAround::WallAround::getMapValue(int x, int y) const
{
    return m_map_[y][x];
}

bool WallAround::WallAround::isObstacle(Direction direction)
{
    switch (direction)
    {
    case FRONT:
        return getX() + 1 >= m_width_ or getMapValue(getX() + 1, getY()) == OBSTACLE;
    case RIGHT:
        return getY() + 1 >= m_height_ or getMapValue(getX(), getY() + 1) == OBSTACLE;
    case BACK:
        return getX() - 1 < 0 or getMapValue(getX() - 1, getY()) == OBSTACLE;
    case LEFT:
        return getY() - 1 < 0 or getMapValue(getX(), getY() - 1) == OBSTACLE;
    default:
        return false;
    }
}

void WallAround::WallAround::move(Direction direction)
{
    if (direction == FRONT)
        setX(getX() + 1);
    else if (direction == BACK)
        setX(getX() - 1);
    else if (direction == LEFT)
        setY(getY() - 1);
    else
        setY(getY() + 1);
    // printf("WallAround position: (%d, %d)\n", getX(), getY());
}
Direction WallAround::WallAround::getNextTowardDirection()
{
    // if (isObstacle(m_main_direction_) and !isObstacle(m_next_direction_))
    // {
    // }
    // else
    if (!isObstacle(m_main_direction_))
    {
        m_next_direction_ = m_main_direction_;
        m_opposite_direction_ = getNextDirection(m_main_direction_);
        m_previous_direction_ = getOppositeDirection(m_main_direction_);
        m_main_direction_ = getPreDirection(m_main_direction_);
    }
    else if (isObstacle(m_main_direction_) and
             isObstacle(m_next_direction_) and
             !isObstacle(m_opposite_direction_))
    {
        m_next_direction_ = getOppositeDirection(m_main_direction_);
        m_opposite_direction_ = getPreDirection(m_main_direction_);
        m_previous_direction_ = m_main_direction_;
        m_main_direction_ = getNextDirection(m_main_direction_);
    }
    else if (isObstacle(m_main_direction_) and
             isObstacle(m_next_direction_) and
             isObstacle(m_opposite_direction_))
    {
        m_next_direction_ = getPreDirection(m_main_direction_);
        m_previous_direction_ = getNextDirection(m_main_direction_);
        m_opposite_direction_ = m_main_direction_;
        m_main_direction_ = getOppositeDirection(m_main_direction_);
    }
    move(m_next_direction_);
    return m_next_direction_;
}

void WallAround::WallAround::printDirections()
{
    switch (m_main_direction_)
    {
    case FRONT:
        printf("  Main Direction: FRONT\n");
        break;
    case BACK:
        printf("  Main Direction: BACK\n");
        break;
    case RIGHT:
        printf("  Main Direction: RIGHT\n");
        break;
    case LEFT:
        printf("  Main Direction: LEFT\n");
        break;
    default:
        break;
    }
}