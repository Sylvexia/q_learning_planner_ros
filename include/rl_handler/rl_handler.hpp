#ifndef RL_HANDLER_HPP
#define RL_HANDLER_HPP

#include "relearn.hpp"
#include "ros/ros.h"

#include <iostream>
#include <unordered_set>
#include <chrono>
#include <ctime>
#include <random>
#include <fstream>
#include <string>
#include <filesystem>
#include <regex>

struct semantic_line_state
{
    int8_t offset_discretization;
    uint8_t special_case;

    uint8_t state_size;

    bool operator==(const semantic_line_state &arg) const
    {
        return (this->offset_discretization == arg.offset_discretization);
    }
};

struct driving_action
{
    int8_t angular_discretization;
    int8_t linear_discretization;
    bool revert;

    uint8_t angular_size;
    uint8_t linear_size;

    bool operator==(const driving_action &arg) const
    {
        return (this->angular_discretization == arg.angular_discretization) &&
               (this->linear_discretization == arg.linear_discretization);
    }
};

//You should hash_combine first, then you could interate the member of the struct
//also, operator== should be defined before hash_code, to know if two structs are equal

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

class RL_handler
{
private:
    std::mt19937 m_rand_gen;

    double m_learning_rate;
    double m_discount_factor;
    double m_epsilon;

    semantic_line_state m_state;
    driving_action m_action;

    const std::filesystem::path m_model_folder; //folder preset path: rl_model/online

public:
    RL_handler();
    ~RL_handler();

    using rl_state = relearn::state<semantic_line_state>;
    using rl_action = relearn::action<driving_action>;

    rl_state state;
    rl_state state_next;
    rl_action action;
    
    relearn::policy<rl_state, rl_action> policy;
    std::deque<relearn::link<rl_state, rl_action>> episode;
    relearn::q_learning<rl_state, rl_action> learner;

    void set_parameter();

    void init();
    void init_rand_generator();

    void load_model(const std::string &filename);
    void save_model(const std::string &filename);
    std::string get_filename_by_cur_time();
    std::string get_recent_filename();

    void get_action();

    void rand_action();
    void best_action();

    void update_state();

    void learn();
};

//#include "rl_handler_lib.cpp"

#endif
