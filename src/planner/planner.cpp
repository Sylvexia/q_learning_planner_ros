#include "planner/planner.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rl_planner");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    Planner planner(nh);

    planner.start();

    return 0;
}