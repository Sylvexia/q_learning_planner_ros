#pragma once
#include "ros/ros.h"

enum class PlannerState
{
    INIT,
    SUSPEND,
    EXECUTING,
    TERMINATED
};