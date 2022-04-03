#include "planner/planner.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rl_planner");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    Planner planner;

    planner.create();

    while (ros::ok())
    {
        planner.execute();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
