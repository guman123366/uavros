
#include <pluginlib/class_list_macros.hpp>
#include <uat_cloud_vechile/cloud_vehicle_base_plugin.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <std_msgs/Float64.h>
#include "cJSON.h"
#include <cmath>
#include "uat_msg/TD550WRLoad.h"

namespace uat
{
    namespace plugin
    {
        class WPLoadPlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            WPLoadPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~WPLoadPlugin() {};
            virtual void initialize()
            {
                ROS_INFO("WPLoadPlugin  initialize");
                _waypoint_load_client = _nh.serviceClient<uat_msg::TD550WRLoad>("/uavros/cmd/waypoints_load");

                _mqtt2ros_command_sub = _nh.subscribe<std_msgs::String>("/uat_cloud_ros/command", 10, &WPLoadPlugin::mqtt_waypoints_cb, this);
            }

          private:
            void mqtt_waypoints_cb(const std_msgs::String::ConstPtr &msg)
            {
                ROS_INFO("receive waypoints data!!!!!!!!!!!!");
                mqtt_services_cb(msg);
            }

          private:
            void handle_services_command(const std::string& strBid, const std::string& strMethod, const cJSON *jsonData)
            {
                _command_method=strMethod;
                _bid=strBid;

                ROS_INFO("receive command method is %s",strMethod.c_str());
                ROS_INFO("receive command bid is %s",strBid.c_str());

                if("wr_load"==strMethod)
                {
                    handle_WayPoints_message(strMethod,strBid,jsonData);
                }
            }

            void handle_WayPoints_message(const std::string& strMethod,const std::string& strBid, const cJSON *jsonData)
            {
                int wr_index=0;
                uat_msg::TD550WRLoad points_msg;
                if(!getIntValue(jsonData,"wr_num",wr_index))
                {
                    do_need_reply(strMethod, strBid, 0, 1, "waypoints param error");
                    return;
                }

                if(wr_index<0||wr_index>20)
                {
                    do_need_reply(strMethod, strBid, 0, 1, "waypoints param error");
                    return;
                }

                points_msg.request.wr_num=wr_index;

                cJSON *way_points=cJSON_GetObjectItemCaseSensitive(jsonData, "wr_points");
                if (cJSON_IsArray(way_points))
                {
                    int wp_num=cJSON_GetArraySize(way_points);
                    points_msg.request.wp_length=wp_num;
                    points_msg.request.points.resize(wp_num);
                    ROS_INFO("points_msg.request.wp_length is %d",wp_num);
                    int wp_index=0,wp_type=0;
                    double wp_lon=0,wp_lat=0,wp_alt=0,wp_speed=0,wp_time=0;
                    for(int i=0;i<wp_num;i++)
                    {
                        cJSON *item = cJSON_GetArrayItem(way_points, i);
                        if(nullptr==item)
                        {
                            ROS_INFO("waypoint_item is empty!!!!");
                            return;
                        }
                        if(!getIntValue(item,"wp_num",wp_index)||!getIntValue(item,"wp_type",wp_type)||!getDoubleValue(item,"wp_lon",wp_lon)||
                           !getDoubleValue(item,"wp_lat",wp_lat)||!getDoubleValue(item,"wp_alt",wp_alt)||!getDoubleValue(item,"wp_speed",wp_speed)||
                           !getDoubleValue(item,"wp_time",wp_time))
                        {
                            do_need_reply(strMethod, strBid, 0, 1, "waypoints param error");
                            ROS_INFO("waypoints param error");
                            return;
                        }

                        if((wp_index<1||wp_index>150)||(wp_type<0||wp_type>10)||(wp_lon<-180||wp_lon>180)||(wp_lat<-90||wp_lat>90)||
                            (wp_alt<0||wp_alt>8000)||(wp_speed<0||wp_speed>300)||(wp_time<0||wp_time>65535))
                        {
                            do_need_reply(strMethod, strBid, 0, 1, "waypoints param error");
                            ROS_INFO("waypoints param error");
                            return;
                        }

                        points_msg.request.points[i].wp_num=wp_index;
                        points_msg.request.points[i].wp_type=wp_type;
                        points_msg.request.points[i].wp_lon=wp_lon;
                        points_msg.request.points[i].wp_lat=wp_lat;
                        points_msg.request.points[i].wp_alt=wp_alt;
                        points_msg.request.points[i].wp_speed=wp_speed;
                        points_msg.request.points[i].wp_time=wp_time;

                        ROS_INFO("wp_num is %d,wp_type is %d,wp_lon is %.8f,wp_lat is %.8f,wp_alt is %.2f,wp_speed is %.2f,wp_time is %.2f",wp_index,wp_type,wp_lon,wp_lat,wp_alt,wp_speed,wp_time);
                    }
                }

                if(_waypoint_load_client.call(points_msg))
                {
                    do_need_reply(strMethod, strBid, 0, 0, "waypoints success");
                    ROS_INFO("waypoints success");
                }
                else 
                {
                    do_need_reply(strMethod, strBid, 0, 1, "waypoints failed");
                    ROS_INFO("waypoints failed");
                }
            }

          private:
            ros::ServiceClient _waypoint_load_client;

            ros::Subscriber _mqtt2ros_command_sub;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::WPLoadPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
