
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
        struct Guidance_law_Info
        {
            int lateral_offset_signal=0;            //侧偏距信号
            int pending_flight_distance_signal=0;   //待飞距信号
            int waiting_time=0;                     //待飞时间
            float track_error_angle=0;              //航迹误差角
            float GSB=0;                            //预置倾斜角GSB
            float d_xy=0;                           //飞机当前位置与起飞点之间距离
            float radio_altitude=0;                 //无线电高度
            int flight_switch_response=0;           //飞行开关指令应答
            int flight_combination_response=0;      //飞行组合指令代码回报
            ros::Time update_time = ros::Time::now();
        };

        class GuidanceLawPlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            GuidanceLawPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~GuidanceLawPlugin() {};
            virtual void initialize()
            {
                ROS_INFO("GuidanceLawPlugin  initialize");
                _ros2mqtt_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/telemetry", 10);
                second_one_sub=_nh.subscribe<uat_msg::TD550SecondsubOneframe>("/telemeter_data_secondsuboneframe", 100, &GuidanceLawPlugin::secondsuboneframe_cb, this);
                second_two_sub=_nh.subscribe<uat_msg::TD550SecondsubTwoframe>("/telemeter_data_secondsubtwoframe", 100, &GuidanceLawPlugin::secondsubtwoframe_cb, this);
                second_three_sub=_nh.subscribe<uat_msg::TD550SecondsubThreeframe>("/telemeter_data_secondsubthreeframe", 100, &GuidanceLawPlugin::secondsubthreeframe_cb, this);
                second_four_sub=_nh.subscribe<uat_msg::TD550SecondsubFourframe>("/telemeter_data_secondsubfourframe", 100, &GuidanceLawPlugin::secondsubfourframe_cb, this);
                third_four_sub = _nh.subscribe<uat_msg::TD550ThirdsubFourframe>("/telemeter_data_thirdsubfourframe", 100, &GuidanceLawPlugin::thirdsubfourframe_cb, this);
                _timer_call_1s = _nh.createTimer(ros::Duration(1), &GuidanceLawPlugin::timer_1s_cb, this, false);

            }

          private:
            void secondsuboneframe_cb(const uat_msg::TD550SecondsubOneframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _guidance_law_info.lateral_offset_signal=msg->SetoverDis;
                _guidance_law_info.pending_flight_distance_signal=msg->FlushingDis;
                _guidance_law_info.waiting_time=msg->FlushingTime;
                _guidance_law_info.track_error_angle=msg->TrackError;

                _guidance_law_info.update_time = ros::Time::now();
            }

            void secondsubtwoframe_cb(const uat_msg::TD550SecondsubTwoframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _guidance_law_info.GSB=msg->GSB;
            }

            void secondsubthreeframe_cb(const uat_msg::TD550SecondsubThreeframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _guidance_law_info.d_xy=msg->DIS_XY;
            }

            void secondsubfourframe_cb(const uat_msg::TD550SecondsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _guidance_law_info.radio_altitude=msg->RadioHeight;
            }

            void thirdsubfourframe_cb(const uat_msg::TD550ThirdsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _guidance_law_info.flight_switch_response=msg->KgReply;
                _guidance_law_info.flight_combination_response=msg->YtReply;
            }

            void timer_1s_cb(const ros::TimerEvent &event)
            {
                guidance_law_publish();
            }

          private:
            void guidance_law_publish()
            {
                std::string strGuidanceLaw;
                {
                    lock_guard lock(_data_mutex);
                    if (ros::Time::now() - _guidance_law_info.update_time > ros::Duration(3.0f))
                        return;

                    cJSON *root = generate_json_root("guidance_law_td550_t1400");
                    cJSON *data = cJSON_CreateObject();
                    if (root == NULL || data == NULL)
                    {
                        cJSON_Delete(root);
                        cJSON_Delete(data);
                        return;
                    }

                    cJSON_AddNumberToObject(data, "lateral_offset_signal", _guidance_law_info.lateral_offset_signal);
                    cJSON_AddNumberToObject(data, "pending_flight_distance_signal", _guidance_law_info.pending_flight_distance_signal);
                    cJSON_AddNumberToObject(data, "waiting_time", _guidance_law_info.waiting_time);
                    cJSON_AddNumberToObject(data, "track_error_angle", _guidance_law_info.track_error_angle);
                    cJSON_AddNumberToObject(data, "GSB", _guidance_law_info.GSB);
                    cJSON_AddNumberToObject(data, "d_xy", _guidance_law_info.d_xy);
                    cJSON_AddNumberToObject(data, "radio_altitude", _guidance_law_info.radio_altitude);
                    cJSON_AddNumberToObject(data, "flight_switch_response", _guidance_law_info.flight_switch_response);
                    cJSON_AddNumberToObject(data, "flight_combination_response", _guidance_law_info.flight_combination_response);

                    cJSON_AddItemToObject(root, "data", data);
                    char *jsonString = cJSON_Print(root);
                    strGuidanceLaw = jsonString;
                    cJSON_Delete(root);
                    free(jsonString);
                }
                std_msgs::String rosString;
                rosString.data = strGuidanceLaw.c_str();
                //std::cout<<strGuidanceLaw<<std::endl;
                publishRos2mqtt(rosString);
            }

          private:
            Guidance_law_Info _guidance_law_info;
            ros::Subscriber second_one_sub;
            ros::Subscriber second_two_sub;
            ros::Subscriber second_three_sub;
            ros::Subscriber second_four_sub;
            ros::Subscriber third_four_sub;
            ros::Timer _timer_call_1s;

            std::mutex _data_mutex;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::GuidanceLawPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
