
#include <pluginlib/class_list_macros.hpp>
#include <uat_cloud_vechile/cloud_vehicle_base_plugin.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <sensor_msgs/NavSatFix.h>
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
        struct Power_Info
        {
            float fc_computer_28V=0;            //飞控计算机28V
            float power_management_Box28V=0;    //电源管理盒28V
            float power_management_box12V=0;    //电源管理盒12V
            float generator_current=0;          //发电机供电电流
            float power_management_box74V=0;   //电源管理盒7.4V
            ros::Time update_time = ros::Time::now();
        };

        class PowerPlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            PowerPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~PowerPlugin() {};
            virtual void initialize()
            {
                ROS_INFO("PowerPlugin  initialize");
                _ros2mqtt_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/telemetry", 10);
                second_three_sub=_nh.subscribe<uat_msg::TD550SecondsubThreeframe>("/telemeter_data_secondsubthreeframe", 100, &PowerPlugin::secondsubthreeframe_cb, this);
                second_four_sub=_nh.subscribe<uat_msg::TD550SecondsubFourframe>("/telemeter_data_secondsubfourframe", 100, &PowerPlugin::secondsubfourframe_cb, this);
                _timer_call_1s = _nh.createTimer(ros::Duration(1), &PowerPlugin::timer_1s_cb, this, false);
            }

          private:
            void secondsubthreeframe_cb(const uat_msg::TD550SecondsubThreeframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _power_info.fc_computer_28V=msg->FCCPower;

                _power_info.update_time = ros::Time::now();
            }

            void secondsubfourframe_cb(const uat_msg::TD550SecondsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _power_info.power_management_Box28V=msg->Power24V;
                _power_info.power_management_box12V=msg->Power12V;
                _power_info.power_management_box74V=msg->PowerBox7V;
                _power_info.generator_current=msg->GeneratorCurrent;
            }

            void timer_1s_cb(const ros::TimerEvent &event)
            {
                main_ins_publish();
            }

          private:
            void main_ins_publish()
            {
                std::string strPower;
                {
                    lock_guard lock(_data_mutex);
                    if (ros::Time::now() - _power_info.update_time > ros::Duration(3.0f))
                        return;

                    cJSON *root = generate_json_root("power_td550_t1400");
                    cJSON *data = cJSON_CreateObject();
                    if (root == NULL || data == NULL)
                    {
                        cJSON_Delete(root);
                        cJSON_Delete(data);
                        return;
                    }

                    cJSON_AddNumberToObject(data, "fc_computer_28V", _power_info.fc_computer_28V);
                    cJSON_AddNumberToObject(data, "power_management_Box28V", _power_info.power_management_Box28V);
                    cJSON_AddNumberToObject(data, "power_management_box12V", _power_info.power_management_box12V);
                    cJSON_AddNumberToObject(data, "generator_current", _power_info.generator_current);
                    cJSON_AddNumberToObject(data, "power_management_box7.4V", _power_info.power_management_box74V);

                    cJSON_AddItemToObject(root, "data", data);
                    char *jsonString = cJSON_Print(root);
                    strPower = jsonString;
                    cJSON_Delete(root);
                    free(jsonString);
                }
                std_msgs::String rosString;
                rosString.data = strPower.c_str();
                //std::cout<<strPower<<std::endl;
                publishRos2mqtt(rosString);
            }

          private:
            Power_Info _power_info;
            ros::Subscriber second_three_sub;
            ros::Subscriber second_four_sub;
            ros::Timer _timer_call_1s;

            std::mutex _data_mutex;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::PowerPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
