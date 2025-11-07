/**
 * @brief Waypoint plugin
 * @file waypoint.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016,2017,2018 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include "uavros_plugin.h"


namespace mavros {
namespace std_plugins {
/**
 * @brief Mission manupulation plugin
 */
class WaypointPlugin : public uavros::plugin::PluginBase {
public:
	WaypointPlugin()
	{ }

	void initialize() override
	{
		PluginBase::initialize();

	}

private:
	ros::NodeHandle wp_nh;

	ros::Publisher wp_list_pub;
	ros::ServiceServer pull_srv;
	ros::ServiceServer push_srv;
	ros::ServiceServer clear_srv;

	ros::Publisher wp_reached_pub;
	ros::ServiceServer set_cur_srv;

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WaypointPlugin, mavros::plugin::PluginBase)