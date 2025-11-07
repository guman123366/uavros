


#pragma once
#include <unistd.h> // Linux系统中
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <errno.h>
#include <fstream>
#include <string>
#include <regex>
#include <dirent.h> // 所在头文件
#include <vector>
#include "../data/DataExchange.hpp"
#include "driver_data_code.hpp" 

namespace ROS_FILE_OPERATE {
    enum UNZIP_STATE {
        UNZIP_OK = 0,
        UNZIP_NULL_FILE = 1,
        UNZIP_ERROR_FILE = 2
    };
    enum NAME_NAME {
        NAME_DEFAULT = 0,
        NAME_SN_Q20 = 5,
        NAME_AUTHEN_URL = 6,
        NAME_USERNAME = 7,
        NAME_PASSWORD = 8,
    };
    struct version_info {
        char name[FRAME_MAX_SIZE] = {0};
        char version[FRAME_MAX_SIZE] = {0};
        char type = 0;
        char name_type = 0;
    };
    int ReadVersionFile(const char * SourcePath, std::string &sn_code);
    int ReadVersionFile(const char * SourcePath, struct DRIVER_DATA_CODE::mg_authen_cmd &mg_authen_cmd);
}