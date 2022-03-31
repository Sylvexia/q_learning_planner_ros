#ifndef _TEST_H_
#define _TEST_H_

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <cstdlib>
#include <vector>

class Planner
{
public:
    Planner();
    ~Planner();

    void print(std::string str);

private:
    int a;
    int b;
    int c;
};

#endif
