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
    uint8_t state_size;

    int8_t offset_discretization;
    uint8_t special_case;
};

struct driving_action
{
    uint8_t angular_size;
    uint8_t linear_size;

    int8_t angular_discretization;
    int8_t linear_discretization;
    bool revert;
};

//You should hash_combine first, then you could interate the member of the struct

namespace std
{
    template <>
    struct hash<semantic_line_state>
    {
        std::size_t operator()(semantic_line_state const &arg) const
        {
            std::size_t seed = 0;
            relearn::hash_combine(seed, arg.offset_discretization);
            return seed;
        }
    };
    template <>
    struct hash<driving_action>
    {
        std::size_t operator()(driving_action const &arg) const
        {
            std::size_t seed = 0;
            relearn::hash_combine(seed, arg.angular_discretization);
            relearn::hash_combine(seed, arg.linear_discretization);
            return seed;
        }
    };
}

struct environment
{
    std::unordered_set<semantic_line_state> line_platform;
};

class RL_handler
{
private:
    std::mt19937 m_rand_gen;

    double m_learning_rate;
    double m_discount_factor;
    double m_epsilon;

    semantic_line_state m_state;
    driving_action m_action;
    environment m_env;

public:
    RL_handler();
    ~RL_handler();

    using rl_state = relearn::state<semantic_line_state>;
    using rl_action = relearn::action<driving_action>;

    rl_state state;
    rl_action action;
    relearn::policy<rl_state, rl_action> policy;

    std::deque<relearn::link<rl_state, rl_action>> episode;

    void set_parameter();

    void generate_rand();

    void load_model();
    void save_model();

    void get_action();

    void rand_action();
    void best_action();

    void learn();
};

//#include "rl_handler_lib.cpp"

#endif
