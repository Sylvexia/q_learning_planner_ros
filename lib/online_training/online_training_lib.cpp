#include "online_training/online_training.hpp"

void PlannerState::set_context(OnlineTraining *online_training)
{
    m_online_training = online_training;
}

PlannerState::~PlannerState()
{
    ROS_INFO("PlannerState destructed");
};


void Initialization::init()
{
    ROS_INFO("Initialization");
    ROS_INFO("Loading the model");
    ROS_INFO("Suspending state");
    m_online_training->transition_to(new Suspending);
}

void Initialization::execute()
{
    ROS_INFO("Initialization haven't completed yet");
}

void Initialization::pause()
{
    ROS_INFO("Initialization haven't completed yet");
}

void Initialization::terminate()
{
    ROS_INFO("Force stop");
}

void Suspending::init()
{
    ROS_INFO("reload model");
}

void Suspending::execute()
{
    ROS_INFO("Starting the plan");
}

void Suspending::pause()
{
    ROS_INFO("What are you pausing for?");
}

void Suspending::terminate()
{
    ROS_INFO("Terminating");
}

void Executing::init()
{
    ROS_INFO("reload model without saving");
}

void Executing::execute()
{
    ROS_INFO("What are you executing when the robot is running?");
}

void Executing::pause()
{
    ROS_INFO("Epoch is completed, saving model, and waiting for the next epoch");
}

void Executing::terminate()
{
    ROS_INFO("Red alert, the whold world is fucked");
}

void Terminating::init()
{
    ROS_INFO("Are you try to restart the world, father Pucci?");
}

void Terminating::execute()
{
    ROS_INFO("No, You can't");
}

void Terminating::pause()
{
    ROS_INFO("The world is fucked, you can't pause");
}

void Terminating::terminate()
{
    ROS_INFO("KEK LUL");
}

OnlineTraining::OnlineTraining()
{
    ROS_INFO("Default class OnlineTraining has been constructed");
}

OnlineTraining::OnlineTraining(ros::NodeHandle &nh)
    : m_nh(nh), m_planner_state(nullptr)
{
    ROS_INFO("Class OnlineTraining has been constructed");

    m_sub_state = m_nh.subscribe("/state", 1, &OnlineTraining::state_callback, this);
    m_sub_reward = m_nh.subscribe("/reward", 1, &OnlineTraining::reward_callback, this);

    m_pub_action = m_nh.advertise<reinforcement_learning_planner::action>("/action", 1);

    transition_to(new Initialization);
}

OnlineTraining::OnlineTraining(ros::NodeHandle &nh, PlannerState *state)
    : m_nh(nh), m_planner_state(nullptr)
{
    ROS_INFO("Class OnlineTraining has been constructed with assigned state");

    m_sub_state = m_nh.subscribe("/state", 1, &OnlineTraining::state_callback, this);
    m_sub_reward = m_nh.subscribe("/reward", 1, &OnlineTraining::reward_callback, this);

    m_pub_action = m_nh.advertise<reinforcement_learning_planner::action>("/action", 1);

    transition_to(state);    
}

OnlineTraining::~OnlineTraining()
{
    ROS_INFO("Class OnlineTraining has been destroyed");
    delete m_planner_state;
}

void OnlineTraining::transition_to(PlannerState *state)
{
    if(m_planner_state != nullptr)
    {
        delete m_planner_state;
    }
    m_planner_state = state;
    m_planner_state->set_context(this);
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