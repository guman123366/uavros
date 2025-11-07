#include <ros/console.h>
#include <ros/ros.h>
#include "uat_log_cloud.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "uat_log_cloud");
    ros::NodeHandle nh("~");

    uat_log_cloud log_cloud;
    
    ros::spin();
    
    return 0;
}