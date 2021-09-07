#pragma once

#include <exception>

class UAVException : public std::exception
{
private:
    const char *msg;

public:
    explicit UAVException(const char *m) : msg(m) {}
    const char *what() const throw()
    {
        return msg;
    }
};