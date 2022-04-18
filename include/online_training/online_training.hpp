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

class OnlineTraining;

class PlannerState
{
protected:
    OnlineTraining *m_online_training;

public:
    virtual ~PlannerState();

    void set_context(OnlineTraining *online_training);

    virtual void init() = 0;
    virtual void execute() = 0;
    virtual void pause() = 0;
    virtual void terminate() = 0;
};

class Initialization : public PlannerState
{
public:
    void init() override;
    void execute() override;
    void pause() override;
    void terminate() override;
};

class Suspending : public PlannerState
{
public:
    void init() override;
    void execute() override;
    void pause() override;
    void terminate() override;
};

class Executing : public PlannerState
{
public:
    void init() override;
    void execute() override;
    void pause() override;
    void terminate() override;
};

class Terminating : public PlannerState
{
public:
    void init() override;
    void execute() override;
    void pause() override;
    void terminate() override;
};

class OnlineTraining
{
public:
    OnlineTraining();
    OnlineTraining(ros::NodeHandle &nh);
    OnlineTraining(ros::NodeHandle &nh, PlannerState *state);
    ~OnlineTraining();

    void transition_to(PlannerState *state);

    void init();    //initialize the planner
    void start();   //start the planner
    void plan();    //plan online training
    void suspend(); //suspend the planner
    void save();    //save the model

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

    void state_callback(const reinforcement_learning_planner::state::ConstPtr &msg);
    void reward_callback(const reinforcement_learning_planner::reward::ConstPtr &msg);

    bool m_resume = false;
    bool m_suspend = false;
    bool m_exit = false;

    PlannerState *m_planner_state;
    RL_handler m_rl_handler;

    friend class Initialization;
    friend class Suspending;
    friend class Executing;
    friend class Terminating;
};