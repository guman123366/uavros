
#include <pluginlib/class_list_macros.hpp>
#include <uat_cloud_vechile/cloud_vehicle_base_plugin.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <std_msgs/Float64.h>
#include "cJSON.h"
#include <cmath>
#include "uat_msg/TD550CommandSwitch.h"
#include "uat_msg/TD550OsdReply.h"

namespace uat
{
    namespace plugin
    {
        class OsdReplyPlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            OsdReplyPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~OsdReplyPlugin() {};
            virtual void initialize()
            {
                ROS_INFO("OsdReplyPlugin  initialize");
                _osd_reply_client = _nh.serviceClient<uat_msg::TD550OsdReply>("/uatros/cmd/osd_reply");
                _mqtt2ros_osd_reply_sub = _nh.subscribe<std_msgs::String>("/uat_cloud_ros/telemetry_reply", 10, &OsdReplyPlugin::mqtt_osd_reply_cb, this);
            }

          private:
            void mqtt_osd_reply_cb(const std_msgs::String::ConstPtr &msg)
            {
                mqtt_services_cb(msg);
            }

            void handle_services_command(const std::string& strBid, const std::string& strMethod, const cJSON *jsonData)
            {
                ROS_INFO("Receive mqtt command!!!");
                if("osd"==strMethod)
                {
                    handle_switch_message(strMethod,strBid,jsonData);
                    ROS_INFO("Receive telemetry_reply is succeed!!!");
                }
            }

            void handle_switch_message(const std::string& strMethod,const std::string& strBid, const cJSON *jsonData) 
            {
                int linkState=1;
                if(!getIntValue(jsonData,"result",linkState))
                {
                    do_need_reply(strMethod, strBid, -1, 0, "param error");
                    return;
                }

                uat_msg::TD550OsdReply cmd;
                if(0==linkState)
                {
                    cmd.request.linkState=true;
                }
                else
                {
                    cmd.request.linkState=false;
                }
                

                if(_osd_reply_client.call(cmd))
                {
                    ROS_INFO("receive telemetry_reply: %d is succeed",linkState);
                }
                else 
                {
                    ROS_INFO("receive telemetry_reply: %d is failed",linkState);
                }
            }

          private:
            ros::ServiceClient _osd_reply_client;
            ros::Timer _timer_call_1s;

            std::mutex _data_mutex;

            ros::Subscriber _mqtt2ros_osd_reply_sub;


        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::OsdReplyPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
