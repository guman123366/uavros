
#include <pluginlib/class_list_macros.hpp>
#include <uat_cloud_vechile/cloud_vehicle_base_plugin.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <std_msgs/Float64.h>
#include "cJSON.h"
#include <cmath>
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

namespace uat
{
    namespace plugin
    {
        struct Remote_Control_Info
        {
            float vertical_remote_control=0;            //纵向遥控指令回报
            float lateral_remote_control=0;             //横向遥控指令回报
            float total_distance_remote_control=0;      //总距遥控指令回报
            float heading_remote_control=0;             //航向遥控指令回报
            int FUTABA_state=0;                         //FUTABA状态字
            ros::Time update_time = ros::Time::now();
        };

        class RemoteControlPlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            RemoteControlPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~RemoteControlPlugin() {};
            virtual void initialize()
            {
                ROS_INFO("RemoteControlPlugin  initialize");
                _ros2mqtt_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/telemetry", 10);
                third_four_sub = _nh.subscribe<uat_msg::TD550ThirdsubFourframe>("/telemeter_data_thirdsubfourframe", 100, &RemoteControlPlugin::thirdsubfourframe_cb, this);
                _timer_call_1s = _nh.createTimer(ros::Duration(1), &RemoteControlPlugin::timer_1s_cb, this, false);
            }

          private:
            void thirdsubfourframe_cb(const uat_msg::TD550ThirdsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _remote_control_info.vertical_remote_control=msg->B2YControl;
                _remote_control_info.lateral_remote_control=msg->B2XControl;
                _remote_control_info.total_distance_remote_control=msg->B1YControl;
                _remote_control_info.heading_remote_control=msg->B1XControl;
                _remote_control_info.FUTABA_state=msg->FUTABAState;

                _remote_control_info.update_time = ros::Time::now();
            }

            void timer_1s_cb(const ros::TimerEvent &event)
            {
                main_ins_publish();
            }

          private:
            void main_ins_publish()
            {
                std::string strRemoteControl;
                {
                    lock_guard lock(_data_mutex);
                    if (ros::Time::now() - _remote_control_info.update_time > ros::Duration(3.0f))
                        return;

                    cJSON *root = generate_json_root("remote_control_td550_t1400");
                    cJSON *data = cJSON_CreateObject();
                    if (root == NULL || data == NULL)
                    {
                        cJSON_Delete(root);
                        cJSON_Delete(data);
                        return;
                    }

                    cJSON_AddNumberToObject(data, "vertical_remote_control", _remote_control_info.vertical_remote_control);
                    cJSON_AddNumberToObject(data, "lateral_remote_control", _remote_control_info.lateral_remote_control);
                    cJSON_AddNumberToObject(data, "total_distance_remote_control", _remote_control_info.total_distance_remote_control);
                    cJSON_AddNumberToObject(data, "heading_remote_control", _remote_control_info.heading_remote_control);
                    cJSON_AddNumberToObject(data, "FUTABA_state", _remote_control_info.FUTABA_state);

                    cJSON_AddItemToObject(root, "data", data);
                    char *jsonString = cJSON_Print(root);
                    strRemoteControl = jsonString;
                    cJSON_Delete(root);
                    free(jsonString);
                }
                std_msgs::String rosString;
                rosString.data = strRemoteControl.c_str();
                //std::cout<<strRemoteControl<<std::endl;
                publishRos2mqtt(rosString);
            }

          private:
            Remote_Control_Info _remote_control_info;
            ros::Subscriber third_four_sub;
            ros::Timer _timer_call_1s;

            std::mutex _data_mutex;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::RemoteControlPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
