#include "rl_handler/rl_handler.hpp"

RL_handler::RL_handler()
{
    ROS_INFO("RL_handler constructed");
}

RL_handler::~RL_handler()
{
    ROS_INFO("RL_handler destructed");
}

void RL_handler::generate_rand()
{
    m_rand_gen = std::mt19937(static_cast<std::size_t>(std::chrono::high_resolution_clock::now()
                                                           .time_since_epoch()
                                                           .count()));
}

void RL_handler::get_action() //epsilon greedy
{
    std::uniform_real_distribution<double> rand_num(0.0, 1.0);
    if (rand_num(m_rand_gen) < epsilon)
    {
        rand_action();
    }
    else
    {
        best_action();//selected from the policy
    }
}

void RL_handler::rand_action()
{
    std::uniform_int_distribution<unsigned int> dist(0, 3);
}

void RL_handler::best_action()
{
    std::cout<<"best_action"<<"\n";
}