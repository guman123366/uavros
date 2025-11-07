#include "uat_flycontrol_analyse.h"
#include <iostream>
#include "DataAnalysis/TD550DataAnalysis.h"
#include "DataDefine/TD550TelemetryData.h"
#include "DataPublisher/TD550DataPublisher.h"
#include "DataProcessFactory/DataProcessorFactory.h"
#include "uat_msg/TD550FirstsubOneframe.h"
#include "uat_msg/TD550FirstsubTwoframe.h"
#include "uat_msg/TD550FirstsubThreeframe.h"
#include "uat_msg/TD550FirstsubFourframe.h"
#include "uat_msg/TD550SecondsubOneframe.h"
#include "uat_msg/TD550SecondsubTwoframe.h"
#include "uat_msg/TD550SecondsubThreeframe.h"
#include "uat_msg/TD550SecondsubFourframe.h"
#include "uat_msg/TD550ThirdsubOneframe.h"
#include "uat_msg/TD550ThirdsubTwoframe.h"
#include "uat_msg/TD550ThirdsubThreeframe.h"
#include "uat_msg/TD550ThirdsubFourframe.h"
#include "uat_command_package.h"

uat_flycontrol_analyse::uat_flycontrol_analyse()
:protocol_nh("protocol"),total_nums(0),mqtt_link_state(false)
{
    telemeter_data_sub = protocol_nh.subscribe<std_msgs::UInt8MultiArray>("/telemeter_data", 100, &uat_flycontrol_analyse::telemeter_data_sub_cb,this);

	remote_control_pub = protocol_nh.advertise<std_msgs::UInt8MultiArray>("/remote_control_data", 100);

	command_switch_srv=protocol_nh.advertiseService("/uavros/cmd/command_switch", &uat_flycontrol_analyse::command_switch_cb, this);
	osd_reply_srv=protocol_nh.advertiseService("/uavros/cmd/osd_reply", &uat_flycontrol_analyse::osd_reply_cb, this);
	waypoints_load_srv=protocol_nh.advertiseService("/uavros/cmd/waypoints_load", &uat_flycontrol_analyse::waypoints_load_cb, this);

    _telemeterFile.open("/home/uatair/Vehicle_Firmware/telemeter_sub_Data.dat", std::ios::binary);
	if(_telemeterFile.is_open())
	{
		ROS_INFO("telemeterFile open successfully!");
	}
	else{
		ROS_INFO("telemeterFile open Failed!");
	}

	std::string uav_type="";
	protocol_nh.getParam("/uav_type",uav_type);
	ROS_ERROR("uav_type is %s",uav_type.c_str());
	if("TD550"==uav_type)
	{
		data_analysis=DataProcessorFactory::createAnalyzer(DataType::TD550);
		data_publisher=DataProcessorFactory::createPublisher(DataType::TD550, protocol_nh);
		ROS_ERROR("TD550 DataAnlyser And Publisher is Created sucessfully!");
	}
	else if("T1400"==uav_type)
	{
		ROS_ERROR("T1400 DataAnlyser And Publisher is Created sucessfully!");
	}
	else{
		ROS_ERROR("Could not find uav type!");
	}

	_command_package=std::make_shared<uat_command_package>();

	_timer_call_80ms = protocol_nh.createTimer(ros::Duration(0.08), &uat_flycontrol_analyse::timer_80ms_cb, this, false);

	update_time=ros::Time::now();

	_timer_waypoint_load = protocol_nh.createTimer(ros::Duration(0.1), &uat_flycontrol_analyse::timer_waypoints_load_cb, this, false,false);

	// pluginlib::ClassLoader<uavros::plugin::PluginBase> plugin_loader("uat_flycontrol_analyse", "uavros::plugin::PluginBase");
	// std::cout << "uat_flycontrol_analyse  Loading plugins..." << std::endl;
	// std::vector<std::string> vecNames=plugin_loader.getDeclaredClasses();
	// std::cout<<"---------------loading plugins size is"<<vecNames.size()<<std::endl;
	// for (const auto &plugin_name : plugin_loader.getDeclaredClasses())
	// {
	// 	try
	// 	{
	// 	auto plugin = plugin_loader.createInstance(plugin_name);
	// 	ROS_INFO_STREAM("Plugin " << plugin_name << " loaded");
		     
	// 	plugin->initializePluginBase();
	// 	plugin->initialize();
	// 	loaded_plugins.push_back(plugin);
	// 	}
	// 	catch (const std::exception &e)
	// 	{      
	// 	std::cerr << e.what() << '\n';
	// 	}
	// }
}

uat_flycontrol_analyse::~uat_flycontrol_analyse()
{
    if(_telemeterFile.is_open())
	{
		_telemeterFile.close();
	}
}

void uat_flycontrol_analyse::telemeter_data_sub_cb(std_msgs::UInt8MultiArray msg)
{
    std::vector<uint8_t> telemeter_data=msg.data;

	if (data_analysis && data_publisher) {
		std::shared_ptr<DataDefineInterface> analysisData=data_analysis->AnalyseData((unsigned char*)telemeter_data.data(),telemeter_data.size());
        if (analysisData != nullptr) {
            data_publisher->publish(analysisData);
        }
    }

	total_nums+=telemeter_data.size();
	//ROS_INFO("rcv telemeter subscribe total_size is%d",total_nums);

    std::cout<<"rcv telemeter subscribe total_size is "<<total_nums<<endl;

    if(_telemeterFile.is_open())
	{
		_telemeterFile.write((char*)telemeter_data.data(),telemeter_data.size());
	}
}

bool uat_flycontrol_analyse::command_switch_cb(uat_msg::TD550CommandSwitch::Request &req, uat_msg::TD550CommandSwitch::Response &res)
{
	_command_package->package_switch_order(req.value);

	ROS_INFO("uat_flycontrol_analyse receive is %d",req.value);

	return true;
}

bool uat_flycontrol_analyse::waypoints_load_cb(uat_msg::TD550WRLoad::Request &req, uat_msg::TD550WRLoad::Response &res)
{
	if(!_timer_waypoint_load.hasPending())
	{
		_timer_waypoint_load.start();
	}
	
	m_nWPIndex=0;
	
	std::vector<double> pointData;
	m_pointsVec.clear();
	for(int i=0;i<req.wp_length;i++)
	{
		pointData.clear();
		pointData.push_back(req.wr_num);
		pointData.push_back(req.points[i].wp_num);
		pointData.push_back(req.points[i].wp_type);
		pointData.push_back(req.points[i].wp_lon);
		pointData.push_back(req.points[i].wp_lat);
		pointData.push_back(req.points[i].wp_alt);
		pointData.push_back(req.points[i].wp_speed);
		pointData.push_back(req.points[i].wp_time);
		m_pointsVec.push_back(pointData);
	}

	_command_package->package_combine_order(0x16,m_pointsVec.at(m_nWPIndex));
	m_nWPIndex++;

	return true;
}

bool uat_flycontrol_analyse::osd_reply_cb(uat_msg::TD550OsdReply::Request &req, uat_msg::TD550OsdReply::Response &res)
{
	mqtt_link_state=req.linkState;
	update_time=ros::Time::now();

	ROS_INFO("uat_flycontrol_analyse is osd_reply_cb is %d",req.linkState);
	
    return true;
}

void uat_flycontrol_analyse::timer_80ms_cb(const ros::TimerEvent &event)
{
	if(ros::Time::now()-update_time>ros::Duration(3.0f))
	{
		mqtt_link_state=false;
	}

	if(/*mqtt_link_state*/true)
	{
		unsigned char sendbuf[128]={0};
		int sendbufsize=0;
		_command_package->package_send_buf(sendbuf,sendbufsize);

		std_msgs::UInt8MultiArray send_msg;
		std::vector<uint8_t> vecSendData(sendbuf,sendbuf+128);
		
		send_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
		send_msg.layout.dim[0].label = "length";
		send_msg.layout.dim[0].size = sendbufsize;   // 数组长度为 5
		send_msg.layout.dim[0].stride = 1;  // 步长为 1

		send_msg.data=vecSendData;

		remote_control_pub.publish(send_msg);
	}
	
}

void uat_flycontrol_analyse::timer_waypoints_load_cb(const ros::TimerEvent &event)
{
	if (m_nWPIndex == 5)
	{
		//QString strText = "";
		//strText = u8"上传航线失败";
		m_bUpRouteState = false;
		m_nWPIndex = 0;
		m_nWPTime = 0;
		m_nUpWPIndex = 0;

		if (_timer_waypoint_load.hasPending())
		{
			_timer_waypoint_load.stop();
		}

		return;
	}

	m_nWPTime += 100;
	if (m_nWPTime > 2000)
	{
		_command_package->package_combine_order(0x16,m_pointsVec.at(m_nWPIndex));
		m_nWPIndex++;
		m_nWPTime = 0;
	}
}