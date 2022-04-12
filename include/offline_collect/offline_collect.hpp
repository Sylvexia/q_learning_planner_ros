#pragma once

#include "ros/ros.h"
#include "planner/planner.hpp"

#include "reinforcement_learning_planner/action.h"
#include "reinforcement_learning_planner/state.h"
#include "reinforcement_learning_planner/reward.h"

class OfflineCollect : Planner
{
public:
    OfflineCollect();
    OfflineCollect(ros::NodeHandle &nh);
    ~OfflineCollect();

    void init() override;  //initialize the planner
    void start() override; //start the planner
    void plan() override;  //plan offline collect

    void state_callback(const reinforcement_learning_planner::state::ConstPtr &msg);
    void reward_callback(const reinforcement_learning_planner::reward::ConstPtr &msg);

    void get_state();
    void get_reward();
    void set_action();

    void save();

private:
    ros::NodeHandle m_nh;
};