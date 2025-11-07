#pragma once
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <iostream>
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 

namespace DRIVER_DATA_CODE {     
	#define SN_LEN 24 
	struct mg_authen_cmd {
		bool cloud_en;
		bool need_identify;
		std::string authen_url;        //url
		std::string username;          //username
		std::string password;          //password
		std::string type;              //type 
		std::string SnCode;            //sncode 
	};
	struct mg_authen_state {
		std::string msg;
		std::string url;               //url
		std::string clientid;          //clientid
		std::string username;          //username 
		std::string password;          //password
		std::string key;               //key
		int code;
		int connect_state;
		int device_type;
	};
}


