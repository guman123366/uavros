
#include <pluginlib/class_list_macros.hpp>
#include <uat_cloud_vechile/cloud_vehicle_base_plugin.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include "cJSON.h"

namespace uat
{
    namespace plugin
    {

        class StatusPlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            StatusPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~StatusPlugin() {};
            virtual void initialize()
            {
                _ros2mqtt_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/status", 10);
                mqtt_status_sub = _nh.subscribe<std_msgs::String>("/uat_cloud_ros/status_reply", 10, &StatusPlugin::mqtt_status_reply, this);
                timer_call_1s = _nh.createTimer(ros::Duration(2), &StatusPlugin::timer_1s_cb, this, false);
                cloud_hb_pub = _nh.advertise<std_msgs::Int8>("/uat_cloud_ros/heartbeat", 10);
                ROS_INFO("StatusPlugin initialize");
            }

          private:
            ros::Subscriber mqtt_status_sub;

            ros::Publisher cloud_hb_pub;

            ros::Timer timer_call_1s;

            bool reply_online = false;
            bool heartbeat_lost = false;
            ros::Time hb_timestamps = ros::Time(0);
            std::mutex _data_mutex;


            void mqtt_status_reply(const std_msgs::String::ConstPtr &msg)
            {
                ROS_INFO("receive mqtt_status_reply!!!!!!");

                std_msgs::String msg_data = *msg;
                cJSON *json = cJSON_Parse(msg_data.data.c_str()); /* 开始解析 */
                if (NULL == json)
                {
                    const char *error_ptr = cJSON_GetErrorPtr();
                    if (error_ptr != NULL)
                    {
                        fprintf(stderr, "Error before: %s\n", error_ptr);
                    }
                    cJSON_Delete(json);
                    return;
                }

                cJSON *method = cJSON_GetObjectItemCaseSensitive(json, "method");
                if (cJSON_IsString(method) && (method->valuestring != NULL))
                {
                    lock_guard lock(_data_mutex);
                    std::string strMethod = method->valuestring;
                    if (strMethod == "online")
                    {
                        ROS_INFO("reply online");
                        reply_online = true;
                    }
                    else if (strMethod == "heartbeat")
                    {
                        heartbeat_lost = false;
                        hb_timestamps = ros::Time::now();
                    }
                }

                cJSON_Delete(json); // 释放内存
            }

            void heart_beat_pub()
            {
                cJSON *root = generate_json_root("heartbeat");
                cJSON *data = cJSON_CreateObject();
                if (root == NULL || data == NULL)
                {
                    cJSON_Delete(root);
                    cJSON_Delete(data);
                    return;
                }

                cJSON_AddStringToObject(data, "model", "T1400");
                cJSON_AddStringToObject(data, "version", "v1.0.12");
                cJSON_AddItemToObject(root, "data", data);
                cJSON_AddNumberToObject(root, "need_reply", 1);

                char *jsonString = cJSON_Print(root);
                std_msgs::String rosString;
                rosString.data = jsonString;

                publishRos2mqtt(rosString);
                cJSON_Delete(root);
                free(jsonString);
            }

            // 发送上线
            void online_pub()
            {
                {
                    lock_guard lock(_data_mutex);
                    if (reply_online)
                        return;
                }
                cJSON *root = generate_json_root("online");
                cJSON *data = cJSON_CreateObject();
                if (root == NULL || data == NULL)
                {
                    cJSON_Delete(root);
                    cJSON_Delete(data);
                    return;
                }

                cJSON_AddStringToObject(data, "model", "T1400");
                cJSON_AddStringToObject(data, "version", "v1.0.12");
                cJSON_AddItemToObject(root, "data", data);
                cJSON_AddNumberToObject(root, "need_reply", 1);

                char *jsonString = cJSON_Print(root);
                std_msgs::String rosString;
                rosString.data = jsonString;
                publishRos2mqtt(rosString);
                cJSON_Delete(root);
                free(jsonString);
            }

            void timer_1s_cb(const ros::TimerEvent &event)
            {
                heart_beat_pub();
                {
                    ros::Duration(0.01).sleep();
                    lock_guard lock(_data_mutex);
                    if (((ros::Time::now() - hb_timestamps) > ros::Duration(5.0f)) && (hb_timestamps != ros::Time(0)))
                    {
                        heartbeat_lost = true;
                        reply_online = false;
                    }
                    std_msgs::Int8 cloud_state;
                    cloud_state.data = heartbeat_lost;
                    cloud_hb_pub.publish(cloud_state);
                }
                online_pub();
            }
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::StatusPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
