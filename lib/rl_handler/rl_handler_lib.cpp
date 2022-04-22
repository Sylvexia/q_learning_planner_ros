#include "rl_handler/rl_handler.hpp"

RL_handler::RL_handler()
    : m_rand_gen(),
      m_learning_rate(0.9),
      m_discount_factor(0.1),
      m_epsilon(0.1),
      m_state{0, 0, 13},
      m_action{0, 0, 0, 5, 3},
      state(0, m_state),
      state_next(0, m_state),
      action{m_action},
      policy(),
      episode(),
      learner{m_learning_rate, m_discount_factor}
{
    ROS_INFO("RL_handler constructed");
}

RL_handler::~RL_handler()
{
    ROS_INFO("RL_handler destructed");
}

void RL_handler::init_rand_generator()
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
    std::uniform_int_distribution<int8_t> angular_gen(-2, 2);
    std::uniform_int_distribution<int8_t> linear_gen(0, 3);

    auto angular = angular_gen(m_rand_gen);
    auto linear = linear_gen(m_rand_gen);

    action = rl_action(driving_action{angular, linear});
}

void RL_handler::best_action()
{
    auto action_ptr = policy.best_action(state);

    action = *(action_ptr);
}

void RL_handler::update_state()
{
    ROS_INFO("Update state");
    state_next = state;
}

void RL_handler::learn()
{
    ROS_INFO("Learning");
    learner(state, action, state_next, policy, false);
}