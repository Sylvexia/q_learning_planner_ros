#include "planner/planner.hpp" 

int main(int argc,char **argv)
{
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;

    Planner planner;
    while(ros::ok())
    {
        planner.print("holy shit if this worked");
    }
    return 0;
}
