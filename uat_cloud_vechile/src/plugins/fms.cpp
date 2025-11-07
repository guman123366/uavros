
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
        struct FMS_Info
        {
            int current_route_number=0;         //当前航线编号
            int current_waypoint_number=0;      //当前航点编号
            int current_waypoint_characters=0;  //当前航点特征字
            int next_waypoint_number=0;         //下一航点编号
            int next_waypoint_character=0;      //下一航点特征字
            int flight_phase=0;                 //飞行阶段
            ros::Time update_time = ros::Time::now();
        };

        class FMSPlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            FMSPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~FMSPlugin() {};
            virtual void initialize()
            {
                ROS_INFO("FMSPlugin  initialize");
                _ros2mqtt_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/telemetry", 10);
                second_one_sub=_nh.subscribe<uat_msg::TD550SecondsubOneframe>("/telemeter_data_secondsuboneframe", 100, &FMSPlugin::secondsuboneframe_cb, this);
                _timer_call_1s = _nh.createTimer(ros::Duration(1), &FMSPlugin::timer_1s_cb, this, false);
            }

          private:
            void secondsuboneframe_cb(const uat_msg::TD550SecondsubOneframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _fms_info.current_route_number=msg->RouteNum;
                _fms_info.current_waypoint_number=msg->WayPointNum;
                _fms_info.current_waypoint_characters=msg->WayPointState;
                _fms_info.next_waypoint_number=msg->NextPointNum;
                _fms_info.next_waypoint_character=msg->NextPointState;
                _fms_info.flight_phase=msg->FlightState;

                _fms_info.update_time = ros::Time::now();
            }

            void timer_1s_cb(const ros::TimerEvent &event)
            {
                fms_publish();
            }

          private:
            void fms_publish()
            {
                std::string strFMS;
                {
                    lock_guard lock(_data_mutex);
                    if (ros::Time::now() - _fms_info.update_time > ros::Duration(3.0f))
                        return;

                    cJSON *root = generate_json_root("fms_td550_t1400");
                    cJSON *data = cJSON_CreateObject();
                    if (root == NULL || data == NULL)
                    {
                        cJSON_Delete(root);
                        cJSON_Delete(data);
                        return;
                    }

                    cJSON_AddNumberToObject(data, "current_route_number", _fms_info.current_route_number);
                    cJSON_AddNumberToObject(data, "current_waypoint_number", _fms_info.current_waypoint_number);
                    cJSON_AddNumberToObject(data, "current_waypoint_characters", _fms_info.current_waypoint_characters);
                    cJSON_AddNumberToObject(data, "next_waypoint_number", _fms_info.next_waypoint_number);
                    cJSON_AddNumberToObject(data, "next_waypoint_character", _fms_info.next_waypoint_character);
                    cJSON_AddNumberToObject(data, "flight_phase", _fms_info.flight_phase);

                    cJSON_AddItemToObject(root, "data", data);
                    char *jsonString = cJSON_Print(root);
                    strFMS = jsonString;
                    cJSON_Delete(root);
                    free(jsonString);
                }
                std_msgs::String rosString;
                rosString.data = strFMS.c_str();
                //std::cout<<strFMS<<std::endl;
                publishRos2mqtt(rosString);
            }

          private:
            FMS_Info _fms_info;
            ros::Subscriber second_one_sub;
            ros::Timer _timer_call_1s;

            std::mutex _data_mutex;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::FMSPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
