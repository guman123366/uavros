#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <fstream>
#include <iostream>
#include "uat_msg/TD550CommandSwitch.h"
#include "uat_msg/TD550OsdReply.h"
#include "uat_msg/TD550WRLoad.h"
#include <pluginlib/class_loader.hpp>
#include "uavros_plugin.h"

//class TD550DataAnalysis;
class DataAnalysisInterface;
class DataDefineInterface;
class uat_command_package;
class DataPublisherInterface;
//class OsdReply;

class uat_flycontrol_analyse
{
public:
    uat_flycontrol_analyse();
    ~uat_flycontrol_analyse();

private:
    ros::NodeHandle protocol_nh;

    ros::Publisher remote_control_pub;
	ros::Subscriber telemeter_data_sub;

    ros::ServiceServer command_switch_srv;
    ros::ServiceServer osd_reply_srv;
    ros::ServiceServer waypoints_load_srv;

	//void remote_control_pub_cb(const uint8_t *buf,const size_t bufsize);
	void telemeter_data_sub_cb(std_msgs::UInt8MultiArray);

    std::ofstream _telemeterFile;

    long total_nums;

    std::shared_ptr<DataAnalysisInterface> data_analysis;
    std::shared_ptr<DataPublisherInterface> data_publisher;

    //void publishTD550TelemetryData(std::shared_ptr<DataDefineInterface> telemetryData);

    bool command_switch_cb(uat_msg::TD550CommandSwitch::Request &req, uat_msg::TD550CommandSwitch::Response &res);
    bool osd_reply_cb(uat_msg::TD550OsdReply::Request &req, uat_msg::TD550OsdReply::Response &res);
    bool waypoints_load_cb(uat_msg::TD550WRLoad::Request &req, uat_msg::TD550WRLoad::Response &res);

    ros::Timer _timer_call_80ms;
    ros::Timer _timer_waypoint_load;

    void timer_80ms_cb(const ros::TimerEvent &event);
    void timer_waypoints_load_cb(const ros::TimerEvent &event);
    std::shared_ptr<uat_command_package> _command_package;

    bool mqtt_link_state;
    ros::Time update_time;

    int m_nWPIndex;	//未收到航点回复重复上传某个航点次数
	int m_nWPTime;			//未收到航点回复时间
	int m_nUpWPIndex;		//上传的航点编号
	bool m_bUpRouteState = false;	//上传航线状态
	std::vector<double> m_vecSendWPData;//上传航点数据
	std::vector<std::vector<double>> m_pointsVec;

    std::vector<uavros::plugin::PluginBase::Ptr> loaded_plugins;
};