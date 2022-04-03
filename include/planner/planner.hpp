#pragma once

#include "ros/ros.h"
#include "relearn.hpp"
#include "reinforcement_learning_planner/action.h"
#include "reinforcement_learning_planner/state.h"
#include "reinforcement_learning_planner/reward.h"

#include <unordered_set>
#include <random>
#include <ctime>
#include <chrono>
#include <string>
#include <vector>

struct semantic_line_state
{
    int8_t offset_discretization = 0;
    uint8_t special_case = 0;
};

struct driving_action
{
    int8_t angular_discretization = 0;
    int8_t linear_discretization = 0;
    bool revert = 0;
};

namespace std
{
    template <>
    struct hash<semantic_line_state>
    {
        std::size_t operator()(const semantic_line_state &arg) const
        {
            std::size_t seed = 0;
            relearn::hash_combine(seed, arg.offset_discretization);
            relearn::hash_combine(seed, arg.special_case);
            return seed;
        }
    };
    template <>
    struct hash<driving_action>
    {
        std::size_t operator()(const driving_action &arg) const
        {
            std::size_t seed = 0;
            relearn::hash_combine(seed, arg.angular_discretization);
            relearn::hash_combine(seed, arg.linear_discretization);
            relearn::hash_combine(seed, arg.revert);
            return seed;
        }
    };
}

struct model
{
    std::unordered_set<semantic_line_state> line_platform;
};

class Planner
{
public:
    Planner();
    Planner(ros::NodeHandle &nh);
    ~Planner();

    void create();
    void execute();

    void plan_q_learning();

    void state_callback(const reinforcement_learning_planner::state::ConstPtr &msg);
    void reward_callback(const reinforcement_learning_planner::reward::ConstPtr &msg);

    void get_state();
    void get_reward();
    void set_action();

private:
    std::mt19937 m_rand_gen;

    ros::NodeHandle m_nh;
    ros::Subscriber m_sub_state;
    ros::Subscriber m_sub_reward;
    ros::Publisher m_pub_action;

    reinforcement_learning_planner::state m_state_msg;
    reinforcement_learning_planner::reward m_reward_msg;
    reinforcement_learning_planner::action m_action_msg;

    using c_state = relearn::state<semantic_line_state>;
    using c_action = relearn::action<driving_action>;

    relearn::state<c_state> m_state = {{{0, 0}}};
    relearn::action<c_action> m_action = {{{0, 0, 0}}};
    relearn::policy<c_state, c_action> m_policy;
};