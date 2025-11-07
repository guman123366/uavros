#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <errno.h>
#include <iostream>
#include <fstream>
#include <string>
#include <regex>
#include <cstdlib>
#include <iostream>
#include <thread>
#include "CloudExchangeNx.hpp"

using namespace std;

void MySigintHandler(int sig) {
    //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("shutting down!");
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "identify_node");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);//while以50Hz进行循环
    ros::AsyncSpinner spinner(4);
    spinner.start();
    signal(SIGINT, MySigintHandler);  //ros::waitForShutdown();
    //覆盖原来的Ctrl+C中断函数，原来的只会调用ros::shutdown()，按下了Ctrl+C，则会调用MySigintHandler
    //为你关闭节点相关的subscriptions, publications, service calls, and service servers，退出进程

    CLOUDEXCHANGENX::GetNode(nh);
    std::thread ThreadCloudExchangeNx(&CLOUDEXCHANGENX::funcThread);
    
    while(ros::ok()) {                                 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }
    ThreadCloudExchangeNx.join();
    return 0;
}


