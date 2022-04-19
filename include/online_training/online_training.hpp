#pragma once

#include "conio.h"

#include <string>
#include <vector>

#include "ros/ros.h"
#include "reinforcement_learning_planner/action.h"
#include "reinforcement_learning_planner/state.h"
#include "reinforcement_learning_planner/reward.h"

#include "planner/planner.hpp"
#include "rl_handler/rl_handler.hpp"

enum class PlannerState
{
    INIT,
    SUSPEND,
    EXECUTING,
    TERMINATED
};

class OnlineTraining
{
public:
    OnlineTraining();
    OnlineTraining(ros::NodeHandle &nh);
    ~OnlineTraining();

    void init();    //initialize the planner
    void start();   //start the planner
    void plan();    //plan online training
    void suspend(); //suspend the planner
    void execute();
    void stop_wheel();
    void save(); //save the model

    void get_state();
    void get_reward();
    void set_action();

    void update_state();

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub_state;
    ros::Subscriber m_sub_reward;
    ros::Publisher m_pub_action;

    reinforcement_learning_planner::state m_state_msg;
    reinforcement_learning_planner::reward m_reward_msg;
    reinforcement_learning_planner::action m_action_msg;

    RL_handler m_rl_handler;
    PlannerState m_planner_state;

    void state_callback(const reinforcement_learning_planner::state::ConstPtr &msg);
    void reward_callback(const reinforcement_learning_planner::reward::ConstPtr &msg);

    bool m_exit;
};