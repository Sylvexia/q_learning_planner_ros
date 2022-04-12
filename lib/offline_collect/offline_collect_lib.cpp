#include "offline_collect/offline_collect.hpp"

OfflineCollect::OfflineCollect()
{
    ROS_INFO("Default OfflineCollect constructed");
}

OfflineCollect::OfflineCollect(ros::NodeHandle &nh)
    : m_nh(nh)
{
    ROS_INFO("OfflineCollect constructed");
}

OfflineCollect::~OfflineCollect()
{
    ROS_INFO("OfflineCollect destructed");
}

void OfflineCollect::init()
{
    ROS_INFO("OfflineCollect init");
}

void OfflineCollect::start()
{
    ROS_INFO("OfflineCollect start");
}

void OfflineCollect::plan()
{
    ROS_INFO("OfflineCollect plan");
}

void OfflineCollect::state_callback(const reinforcement_learning_planner::state::ConstPtr &msg)
{
    ROS_INFO("OfflineCollect state_callback");
}

void OfflineCollect::reward_callback(const reinforcement_learning_planner::reward::ConstPtr &msg)
{
    ROS_INFO("OfflineCollect reward_callback");
}

void OfflineCollect::get_state()
{
    ROS_INFO("OfflineCollect get_state");
}

void OfflineCollect::get_reward()
{
    ROS_INFO("OfflineCollect get_reward");
}

void OfflineCollect::set_action()
{
    ROS_INFO("OfflineCollect set_action");
}

void OfflineCollect::save()
{
    ROS_INFO("OfflineCollect save");
}