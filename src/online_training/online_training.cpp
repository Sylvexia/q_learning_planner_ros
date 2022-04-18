#include "online_training/online_training.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rl_planner");
    ros::NodeHandle nh;

    OnlineTraining online_training(nh);

    online_training.start();

    return 0;
}