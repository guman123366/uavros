#include <ros/console.h>
#include <ros/ros.h>
#include "uat_flycontrol_analyse.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "uat_flycontrol_analyse");
    ros::NodeHandle nh("~");
    
    uat_flycontrol_analyse fcDataProtocol;
    ros::spin();
    
    return 0;
}