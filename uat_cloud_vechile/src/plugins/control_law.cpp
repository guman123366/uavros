
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
        struct Control_Law_Info
        {
            float b1c=0;                    //俯仰桨距角
            float a1c=0;                    //滚转桨距角
            float dtc=0;                    //偏航桨距角
            float ctc=0;                    //总距角
            float chk=0;                    //风门舵机开度
            float give_value_speed_z=0;     //垂直速度给定基准值
            float give_value_speed_y=0;     //纵向速度给定值
            float give_value_position_y=0;  //纵向位置给定值
            float give_value_speed_x=0;     //横向速度给定值
            float give_value_position_x=0;  //横向位置给定值
            float give_value_yaw=0;         //航向给定值
            float give_value_alt=0;         //高度给定值
            int landing_point_number=0;     //迫降点序号
            float servo_pos=0;              //伺服电机位置
            float servo_goal_pos=0;         //伺服电机目标位置
            ros::Time update_time = ros::Time::now();
        };

        class ControlLawPlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            ControlLawPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~ControlLawPlugin() {};
            virtual void initialize()
            {
                ROS_INFO("ControlLawPlugin  initialize");
                _ros2mqtt_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/telemetry", 10);
                second_two_sub=_nh.subscribe<uat_msg::TD550SecondsubTwoframe>("/telemeter_data_secondsubtwoframe", 100, &ControlLawPlugin::secondsubtwoframe_cb, this);
                second_three_sub=_nh.subscribe<uat_msg::TD550SecondsubThreeframe>("/telemeter_data_secondsubthreeframe", 100, &ControlLawPlugin::secondsubthreeframe_cb, this);
                third_four_sub = _nh.subscribe<uat_msg::TD550ThirdsubFourframe>("/telemeter_data_thirdsubfourframe", 100, &ControlLawPlugin::thirdsubfourframe_cb, this);
                _timer_call_1s = _nh.createTimer(ros::Duration(1), &ControlLawPlugin::timer_1s_cb, this, false);
            }

          private:
            void secondsubtwoframe_cb(const uat_msg::TD550SecondsubTwoframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _control_law_info.b1c=msg->b1c;
                _control_law_info.a1c=msg->a1c;
                _control_law_info.dtc=msg->dtc;
                _control_law_info.ctc=msg->ctc;
                _control_law_info.chk=msg->chk;
                _control_law_info.give_value_speed_z=msg->RouteVSpeed;
                _control_law_info.give_value_speed_y=msg->RouteSpeed;
                _control_law_info.give_value_position_y=msg->RouteVPostion;
                _control_law_info.give_value_speed_x=msg->RouteSideSpeed;
                _control_law_info.give_value_position_x=msg->RouteHPostion;
                _control_law_info.give_value_yaw=msg->RouteYPostion;
                _control_law_info.give_value_alt=msg->RouteHeight;

                _control_law_info.update_time = ros::Time::now();
            }

            void secondsubthreeframe_cb(const uat_msg::TD550SecondsubThreeframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _control_law_info.landing_point_number=msg->ForceLandPointIndex;
            }

            void thirdsubfourframe_cb(const uat_msg::TD550ThirdsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _control_law_info.servo_pos=msg->ServoPos;
                _control_law_info.servo_goal_pos=msg->ServoGoalPos;
            }

            void timer_1s_cb(const ros::TimerEvent &event)
            {
                control_law_publish();
            }

          private:
            void control_law_publish()
            {
                std::string strControlLaw;
                {
                    lock_guard lock(_data_mutex);
                    if (ros::Time::now() - _control_law_info.update_time > ros::Duration(3.0f))
                        return;

                    cJSON *root = generate_json_root("control_law_td550_t1400");
                    cJSON *data = cJSON_CreateObject();
                    if (root == NULL || data == NULL)
                    {
                        cJSON_Delete(root);
                        cJSON_Delete(data);
                        return;
                    }

                    cJSON_AddNumberToObject(data, "b1c", _control_law_info.b1c);
                    cJSON_AddNumberToObject(data, "a1c", _control_law_info.a1c);
                    cJSON_AddNumberToObject(data, "dtc", _control_law_info.dtc);
                    cJSON_AddNumberToObject(data, "ctc", _control_law_info.ctc);
                    cJSON_AddNumberToObject(data, "chk", _control_law_info.chk);
                    cJSON_AddNumberToObject(data, "give_value_speed_z", _control_law_info.give_value_speed_z);
                    cJSON_AddNumberToObject(data, "give_value_speed_y", _control_law_info.give_value_speed_y);
                    cJSON_AddNumberToObject(data, "give_value_position_y", _control_law_info.give_value_position_y);
                    cJSON_AddNumberToObject(data, "give_value_speed_x", _control_law_info.give_value_speed_x);
                    cJSON_AddNumberToObject(data, "give_value_position_x", _control_law_info.give_value_position_x);
                    cJSON_AddNumberToObject(data, "give_value_yaw", _control_law_info.give_value_yaw);
                    cJSON_AddNumberToObject(data, "give_value_alt", _control_law_info.give_value_alt);
                    cJSON_AddNumberToObject(data, "landing_point_number", _control_law_info.landing_point_number);
                    cJSON_AddNumberToObject(data, "servo_pos", _control_law_info.servo_pos);
                    cJSON_AddNumberToObject(data, "servo_goal_pos", _control_law_info.servo_goal_pos);

                    cJSON_AddItemToObject(root, "data", data);
                    char *jsonString = cJSON_Print(root);
                    strControlLaw = jsonString;
                    cJSON_Delete(root);
                    free(jsonString);
                }
                std_msgs::String rosString;
                rosString.data = strControlLaw.c_str();
                //std::cout<<strControlLaw<<std::endl;
                publishRos2mqtt(rosString);
            }

          private:
            Control_Law_Info _control_law_info;
            ros::Subscriber second_two_sub;
            ros::Subscriber second_three_sub;
            ros::Subscriber third_four_sub;
            ros::Timer _timer_call_1s;

            std::mutex _data_mutex;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::ControlLawPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
