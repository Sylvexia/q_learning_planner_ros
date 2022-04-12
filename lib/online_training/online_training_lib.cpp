#include "online_training/online_training.hpp"

OnlineTraining::OnlineTraining()
{
    ROS_INFO("Default class OnlineTraining has been constructed");
}

OnlineTraining::OnlineTraining(ros::NodeHandle &nh)
    : m_nh(nh)
{
    ROS_INFO("Class OnlineTraining has been constructed");

    m_sub_state = m_nh.subscribe("/state", 1, &OnlineTraining::state_callback, this);
    m_sub_reward = m_nh.subscribe("/reward", 1, &OnlineTraining::reward_callback, this);

    m_pub_action = m_nh.advertise<reinforcement_learning_planner::action>("/action", 1);
}

OnlineTraining::~OnlineTraining()
{
    ROS_INFO("Class OnlineTraining has been destroyed");
}

void OnlineTraining::init()
{
    m_resume = false;
    m_suspend = false;
    m_exit = false;
}

void OnlineTraining::start()
{
    ros::Rate loop_rate(1);

    while (!m_exit)
    {
        init();

        while (!m_resume && !m_exit)
        {
            ros::spinOnce();
            plan();

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

void OnlineTraining::suspend()
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

void OnlineTraining::save()
{
    ROS_INFO("Saving model");
}

void OnlineTraining::plan()
{
    ROS_INFO("plan_q_learning");
    //m_rl_handler.get_action();
    set_action();
    ros::spinOnce();
    get_reward();
    get_state();

    //m_rl_handler.learn();
    update_state();

}

void OnlineTraining::state_callback(const reinforcement_learning_planner::state::ConstPtr &msg)
{
    ROS_INFO("State callback");
}

void OnlineTraining::reward_callback(const reinforcement_learning_planner::reward::ConstPtr &msg)
{
    ROS_INFO("Reward callback");
}

void OnlineTraining::get_state()
{
    ROS_INFO("Getting state");
}

void OnlineTraining::get_reward()
{
    ROS_INFO("Getting reward");
}

void OnlineTraining::set_action()
{
    m_pub_action.publish(m_action_msg);
    ROS_INFO("Setting action:%d", m_action_msg.linear_action);
}

void OnlineTraining::update_state()
{
    ROS_INFO("Updating state");
}