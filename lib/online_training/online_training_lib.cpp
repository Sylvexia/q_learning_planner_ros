#include "online_training/online_training.hpp"

OnlineTraining::OnlineTraining()
{
    ROS_INFO("Default class OnlineTraining has been constructed");
}

OnlineTraining::OnlineTraining(ros::NodeHandle &nh)
    : m_nh(nh),
      m_sub_state(nh.subscribe("/state", 1, &OnlineTraining::state_callback, this)),
      m_sub_reward(nh.subscribe("/reward", 1, &OnlineTraining::reward_callback, this)),
      m_pub_action(nh.advertise<reinforcement_learning_planner::action>("/action", 1)),
      m_planner_state(PlannerState::INIT),
      m_exit(false)
{
    ROS_INFO("Class OnlineTraining has been constructed");
}

OnlineTraining::~OnlineTraining()
{
    ROS_INFO("Class OnlineTraining has been destroyed");
}

void OnlineTraining::init()
{
    m_rl_handler.load_model();
    m_rl_handler.generate_rand();
    //m_rl_handler.episode.clear();
    m_planner_state = PlannerState::SUSPEND;
}

void OnlineTraining::start()
{
    while (!m_exit)
    {
        switch (m_planner_state)
        {

        case PlannerState::INIT:
        {
            init();
            break;
        }
        case PlannerState::SUSPEND:
        {
            suspend();
            break;
        }
        case PlannerState::EXECUTING:
        {
            execute();
            break;
        }
        case PlannerState::TERMINATED:
        {
            m_exit = true;
            break;
        }
        default:
        {
            ROS_INFO("Why are you here?");
            break;
        }
        }
    }
}

void OnlineTraining::suspend()
{
    ros::Rate suspend_rate(2);

    while (true)
    {
        ROS_INFO("Suspending, press s to save, press e to execute, press ESC to exit");
        suspend_rate.sleep();
        bool is_suspend = true;

        if (kbhit())
        {
            switch (tolower(getch()))
            {
            case 's':
            {
                save();
                ROS_INFO("Saved model");
                is_suspend = false;
                break;
            }
            case 'e':
            {
                ROS_INFO("Execute");
                m_planner_state = PlannerState::EXECUTING;
                is_suspend = false;
                break;
            }
            case 27: // ESC
            {
                m_planner_state = PlannerState::TERMINATED;
                is_suspend = false;
                break;
            }
            }
        }

        if (!is_suspend)
            break;
    }
}

void OnlineTraining::execute()
{
    ros::Rate execute_rate(2);
    while (true)
    {
        ros::spinOnce();
        plan();
        execute_rate.sleep();
        bool is_executing = true;

        if (kbhit())
        {
            switch (tolower(getch()))
            {
            case 's':
            {
                save();
                stop_wheel();
                m_planner_state = PlannerState::SUSPEND;
                is_executing = false;
                break;
            }
            case 27: // ESC
            {
                m_planner_state = PlannerState::TERMINATED;
                is_executing = false;
                break;
            }
            }
        }
        if (!is_executing)  break;
    }
}

void OnlineTraining::stop_wheel()
{
    m_action_msg.linear_action = 0;
    m_action_msg.angular_action = 0;
    m_pub_action.publish(m_action_msg);
}

void OnlineTraining::save()
{
    ROS_INFO("Saving model");
}

void OnlineTraining::plan()
{
    ROS_INFO("plan_q_learning");
    m_rl_handler.get_action();
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
    using rl_state = relearn::state<semantic_line_state>;
    ROS_INFO("Getting state");
    m_rl_handler.state = rl_state(m_reward_msg.offset, {7, m_state_msg.offset, m_state_msg.special_case});
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