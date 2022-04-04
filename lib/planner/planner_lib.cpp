#include "planner/planner.hpp"

Planner::Planner()
{
    ROS_INFO("Default class planner has been constructed");
}

Planner::Planner(ros::NodeHandle &nh)
    : m_nh(nh), m_state{{{0, 0}}}, m_action{{{0, 0, 0}}}
{
    ROS_INFO("Class planner has been constructed");

    m_sub_state = m_nh.subscribe("/state", 1, &Planner::state_callback, this);
    m_sub_reward = m_nh.subscribe("/reward", 1, &Planner::reward_callback, this);

    m_pub_action = m_nh.advertise<reinforcement_learning_planner::action>("/action", 1);
}

Planner::~Planner()
{
    ROS_INFO("Class planner has been destroyed");
}

void Planner::init()
{
    m_resume = false;
    m_suspend = false;
    m_exit = false;

    m_rand_gen = std::mt19937(static_cast<std::size_t>(std::chrono::high_resolution_clock::now()
                                                           .time_since_epoch()
                                                           .count()));
}

void Planner::start()
{
    ros::Rate loop_rate(1);

    while (!m_exit)
    {
        init();

        while (!m_resume && !m_exit)
        {
            ros::spinOnce();
            execute();

            loop_rate.sleep();

            if (kbhit())
            {
                switch (tolower(getch()))
                {
                case 's':
                {
                    save();
                    suspend();
                    break;
                }
                case 27: // ESC
                {
                    m_exit = true;
                    save();
                    break;
                }
                default:
                    break;
                }
            }
        }
    }
}

void Planner::execute()
{
    //sense();
    get_state();
    get_reward();
    //plan();
    plan_q_learning();
    //act();
    set_action();
}

void Planner::suspend()
{
    ros::Rate loop_rate(2);
    m_suspend = true;
    while (m_suspend)
    {
        ROS_INFO("Suspending, press S to resume");
        loop_rate.sleep();

        if (kbhit())
        {
            switch (tolower(getch()))
            {
            case 's':
            {
                m_suspend = false;
                m_resume = true;
                ROS_INFO("Resume");
                break;
            }
            case 27: // ESC
            {
                m_exit = true;
                m_suspend = false;
                break;
            }
            default:
                break;
            }
        }
    }
}

void Planner::save()
{
    ROS_INFO("Saving model");
}

void Planner::plan_q_learning()
{
    ROS_INFO("plan_q_learning");
}

void Planner::state_callback(const reinforcement_learning_planner::state::ConstPtr &msg)
{
    ROS_INFO("State callback");
}

void Planner::reward_callback(const reinforcement_learning_planner::reward::ConstPtr &msg)
{
    ROS_INFO("Reward callback");
}

void Planner::get_state()
{
    ROS_INFO("Getting state");
}

void Planner::get_reward()
{
    ROS_INFO("Getting reward");
}

void Planner::set_action()
{
    m_pub_action.publish(m_action_msg);
    ROS_INFO("Setting action:%d", m_action_msg.linear_action);
}
