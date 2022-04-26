#include "online_training/online_training.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rl_planner");
    ros::NodeHandle nh;

    RL_handler rl_handler;

    rl_handler.load_model("load_model.txt");    //this error should be catched since there is no such file

    //rl_handler.save_model("save_model.txt");

    //rl_handler.load_model("save_model.txt");

    ros::Rate loop_rate(1);
    for (int i = 0; i < 10; i++)
    {
        loop_rate.sleep();
        rl_handler.save_model(rl_handler.get_filename_by_cur_time());
    }

    rl_handler.load_model(rl_handler.get_recent_filename());

    //rl_handler.save_model("~model/save_another_model.txt");

    return 0;
}