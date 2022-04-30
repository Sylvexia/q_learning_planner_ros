#include <chrono>
#include <random>
#include <ros/ros.h>

#include "reinforcement_learning_planner/state.h"
#include "reinforcement_learning_planner/reward.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_rand_generator");
    ros::NodeHandle nh;
    ros::Rate loop_rate(3);

    ros::Publisher state_pub = nh.advertise<reinforcement_learning_planner::state>("/state", 1);
    ros::Publisher reward_pub = nh.advertise<reinforcement_learning_planner::reward>("/reward", 1);

    reinforcement_learning_planner::state state_msg;
    reinforcement_learning_planner::reward reward_msg;

    auto rand_gen = std::mt19937(static_cast<std::size_t>(std::chrono::high_resolution_clock::now()
                                                              .time_since_epoch()
                                                              .count()));

    while (ros::ok())
    {
        std::uniform_int_distribution<int> state_rand(-3, 3);
        std::uniform_real_distribution<double> reward_rand(-2.0, 2.0);

        state_msg.offset = state_rand(rand_gen);
        reward_msg.offset = reward_rand(rand_gen);

        state_pub.publish(state_msg);
        reward_pub.publish(reward_msg);

        ROS_INFO("State: %d, Reward: %f", state_msg.offset, reward_msg.offset);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
