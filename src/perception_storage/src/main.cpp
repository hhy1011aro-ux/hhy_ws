#include "perception_storage/hecbot_storage_itof_monitor.h"
// #include <ctime>
// std::string res_file_name;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hecbot_storage_itof_monitor");
    ros::NodeHandle nh;

    
    // time_t rawtime;
    // struct tm *ptminfo;
    // time(&rawtime);
    // ptminfo = localtime(&rawtime);
    // {
    //     char filename[512];
    //     sprintf(filename, "res_%02d%02d%02d%02d%02d%02d.csv",
    //         ptminfo->tm_year + 1900, ptminfo->tm_mon + 1, ptminfo->tm_mday,
    //         ptminfo->tm_hour, ptminfo->tm_min, ptminfo->tm_sec);
    //     res_file_name = filename;
    //     ROS_INFO("=====res_file_name%s",res_file_name.c_str());
    // }
    int config_prepared_ = 0;
    ros::param::get("/navigation/initialize/config_prepared",config_prepared_);
    
    while(0 == config_prepared_) //配置未完成，一直等待
    {
        nh.param("/navigation/initialize/config_prepared",config_prepared_,0);
        ros::Duration(1).sleep();
    }

    int storage_collision_detection = 0;
    nh.getParam("/navigation/perception/robot_storage_collision_detection/enable", storage_collision_detection);

    if(storage_collision_detection == 1){
        StorageItof storage_itof_detect;
        ros::spin(); 
    }
    else{
        while(ros::ok()){
            ros::Duration(1).sleep();
        }
    }
    return 0;
}
