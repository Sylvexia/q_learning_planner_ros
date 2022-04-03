#include "planner/planner.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rl_planner");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    bool save = false;
    bool exit = false;
    bool suspend = false;

    Planner planner(nh);

    while (!exit)
    {
        save = false;
        planner.init();

        while (!save && !exit)
        {
            ros::spinOnce();
            planner.execute();
            loop_rate.sleep();

            if (kbhit())
            {
                switch (tolower(getch()))
                {
                case 's':
                {
                    planner.save();
                    save = true;
                    suspend = true;
                    while (suspend)
                    {
                        ROS_INFO("Suspending, press S to resume");
                        loop_rate.sleep();
                        if (kbhit() && getch() == 's')
                        {
                            suspend = false;
                            ROS_INFO("Resume!");
                        }
                    }
                    break;
                }
                case 27:
                {
                    planner.save();
                    exit = true;
                    break;
                }
                }
            }
        }
    }

    return 0;
}