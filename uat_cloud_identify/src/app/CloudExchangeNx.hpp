// Copyright (c) 2020 Cesanta Software Limited
// All rights reserved
//
// Example MQTT client. It performs the following steps:
//    1. Connects to the MQTT server specified by `s_url` variable
//    2. When connected, subscribes to the topic `s_sub_topic`
//    3. Publishes message `hello` to the `s_pub_topic`
//    4. Receives that message back from the subscribed topic and closes
//    5. Timer-based reconnection logic revives the connection when it is down
//
// To enable SSL/TLS for this client, build it like this:
//    make MBEDTLS_DIR=/path/to/your/mbedtls/installation
#pragma once
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
#include "../data/DataExchange.hpp"
#include "../drivers/mongoose.h"
#include "../drivers/ros_file_operate.hpp"
#include "../drivers/driver_data_code.hpp"
#include "../drivers/cJSON.h"
#include "../drivers/mqtt_warehouse_code.hpp"
#include <mavros_msgs/UAT_MqttAuthenStatus.h>
#include <mavros_msgs/UAT_AuthenCtl.h>

namespace CLOUDEXCHANGENX {
    void funcThread();
    void GetNode(ros::NodeHandle &nh);
    void RosserviceSend();
}

