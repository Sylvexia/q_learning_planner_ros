#include "rl_handler/rl_handler.hpp"

RL_handler::RL_handler()
    : m_rand_gen(),
      m_learning_rate(0.9),
      m_discount_factor(0.1),
      m_epsilon(0.1),
      m_state{7, 0, 0},
      m_action{5, 3, 0, 0, 0},
      m_env(),
      state(0, m_state),
      action{m_action},
      policy(),
      episode()
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

void RL_handler::load_model()
{
    ROS_INFO("Loading model");
}

void RL_handler::save_model()
{
    ROS_INFO("Saving model");
}

void RL_handler::get_action() //epsilon greedy
{
    std::uniform_real_distribution<double> rand_num(0.0, 1.0);
    if (rand_num(m_rand_gen) > m_epsilon)
        best_action(); //selected from the policy
    else
        rand_action();
}

void RL_handler::rand_action()
{
    std::uniform_int_distribution<unsigned int> angular(0, 5);
    std::uniform_int_distribution<unsigned int> linear(0, 3);
}

void RL_handler::best_action()
{
    ROS_INFO("Best action");
}

void RL_handler::learn()
{
    ROS_INFO("Learning");
}