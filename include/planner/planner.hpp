#pragma once
#include "ros/ros.h"
class Planner
{
public:
    virtual void init() = 0;  //A plann always need to be initialized
    virtual void start() = 0; //A plan always need to be started
    virtual void plan() = 0;  //A plan is executred in a loop
    virtual ~Planner();
};