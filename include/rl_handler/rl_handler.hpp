#ifndef RL_HANDLER_HPP
#define RL_HANDLER_HPP

#include "relearn.hpp"
#include "ros/ros.h"

#include <iostream>
#include <unordered_set>
#include <chrono>
#include <ctime>
#include <random>

struct semantic_line_state
{
    int8_t offset_discretization = 0;
    uint8_t special_case = 0;
    int reward = 0;
};

struct driving_action
{
    int8_t angular_discretization = 0;
    int8_t linear_discretization = 0;
    bool revert = 0;
};

//You should hash_combine first, then you could interate the member of the struct

namespace std
{
    template <>
    struct hash<semantic_line_state>
    {
        std::size_t operator()(semantic_line_state const& arg) const
        {
            std::size_t seed = 0;
            relearn::hash_combine(seed, arg.offset_discretization);
            return seed;
        }
    };
    template <>
    struct hash<driving_action>
    {
        std::size_t operator()(driving_action const& arg) const
        {
            std::size_t seed = 0;
            relearn::hash_combine(seed, arg.angular_discretization);
            relearn::hash_combine(seed, arg.linear_discretization);
            return seed;
        }
    };
}

struct model
{
    std::unordered_set<semantic_line_state> line_platform;
};

class RL_handler
{
public:
    RL_handler();
    ~RL_handler();

    using rl_state = relearn::state<semantic_line_state>;
    using rl_action = relearn::action<driving_action>;

    relearn::state<rl_state> state = {{{0, 0}}};
    relearn::action<rl_action> action = {{{0, 0, 0}}};
    relearn::policy<rl_state, rl_action> policy;

    void generate_rand();
    void generate_model();

    void set_parameter();
    void get_action();

    void rand_action();
    void best_action();

    void learn();

private:
    std::mt19937 m_rand_gen;

    double learning_rate;
    double discount_factor;
    double epsilon;

    model m_env;
};

//#include "rl_handler_lib.cpp"

#endif
