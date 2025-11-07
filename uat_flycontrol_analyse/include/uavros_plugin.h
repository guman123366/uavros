/**
 * @brief UAVROS Plugin base
 * @file uavros_plugin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 *  @brief UAVROS Plugin system
 */
/*
 * Copyright 2013,2014,2015 Vladimir Ermakov.
 *
 * This file is part of the uavros package and subject to the license terms
 * in the top-level LICENSE file of the uavros repository.
 */

#pragma once

#include <tuple>
#include <vector>
#include <functional>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mavconn/interface.h>
#include <std_msgs/UInt8MultiArray.h>

namespace uavros {
namespace plugin {
typedef std::lock_guard<std::recursive_mutex> lock_guard;
typedef std::unique_lock<std::recursive_mutex> unique_lock;

/**
 * @brief MAVROS Plugin base class
 */
class PluginBase
{
private:
	PluginBase(const PluginBase&) = delete;

public:
	// pluginlib return boost::shared_ptr
	using Ptr = boost::shared_ptr<PluginBase>;
	using ConstPtr = boost::shared_ptr<PluginBase const>;

	virtual ~PluginBase() {};

	/**
	 * @brief Plugin initializer
	 */
	virtual void initialize()
	{
		cmd_pub=cmd_nh.advertise<std_msgs::UInt8MultiArray>("/remote_control_data", 100);
	}

protected:
	/**
	 * @brief Plugin constructor
	 * Should not do anything before initialize()
	 */
	PluginBase(){};

	ros::NodeHandle cmd_nh;
	ros::Publisher cmd_pub;
	ros::ServiceServer cmd_srv;

};
}	// namespace plugin
}	// namespace mavros
