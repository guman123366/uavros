
#include <pluginlib/class_list_macros.hpp>
#include <uat_cloud_vechile/cloud_vehicle_base_plugin.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/Uat_NetlinkState.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/Sensors_Health.h>
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
        struct Vehicle_Info
        {
            std::string uas_model = ""; //*
            int coordinate = 1;         //*
            float longitude = 0;        //*
            float latitude = 0;         //*
            float height = 0;           //*
            float altitude = 0;         //*
            bool update_coordinate = false;
            float vs = 0;      //*
            float gs = 0;      //*
            float heading = 0; //*
            float pitch = 0;   //*
            float roll = 0;    //*
            float voltage=0;

            int fuel = -1;    //*
            std::string mode; //*
            int flying = 0;   //*
            int fly_time=0;    //*
            ros::SteadyTime fly_start_time = ros::SteadyTime::now(); //*
            ros::SteadyTime fly_end_time = fly_start_time;           //*

            int fly_distance = 0; //*

            int home_set = 1;        //*
            float home_distance = 0; //*
            bool update_home_coordinate = false;
            float home_location_longitude = 0; //*
            float home_location_latitude = 0;  //*
            float home_location_altitude = 0;  //*

            int safe_enabled = 1;
            int rtk_enabled = 0;
            int rtk_connected = 0;
            int rc_connected = 0;
            int obs_enabled = 0;
            int accel_cal = 0;
            int gyro_cal = 0;
            int hor_cal = 0;
            int mag_cal = 0;
            int fpv_live = 0; //*
            int down_link_state=0;
            ros::Time update_time = ros::Time::now();
        };

        class OsdPlugin : public cloud_vehicle_bridge::PluginBase
        {
            public:
            OsdPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~OsdPlugin() {};
            virtual void initialize()
            {
                ROS_INFO("OsdPlugin  initialize");
                _ros2mqtt_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/telemetry", 10);
                first_two_sub = _nh.subscribe<uat_msg::TD550FirstsubTwoframe>("/telemeter_data_firstsubtwoframe", 100, &OsdPlugin::firstsubtwoframe_cb, this);
                first_three_sub = _nh.subscribe<uat_msg::TD550FirstsubThreeframe>("/telemeter_data_firstsubthreeframe", 100, &OsdPlugin::firstsubthreeframe_cb, this);
                second_four_sub=_nh.subscribe<uat_msg::TD550SecondsubFourframe>("/telemeter_data_secondsubfourframe", 100, &OsdPlugin::secondsubfourframe_cb, this);
                third_two_sub = _nh.subscribe<uat_msg::TD550ThirdsubTwoframe>("/telemeter_data_thirdsubtwoframe", 100, &OsdPlugin::thirdsubtwoframe_cb, this);
                _timer_call_1s = _nh.createTimer(ros::Duration(1), &OsdPlugin::timer_1s_cb, this, false);
                //_timer_call_100ms = _nh.createTimer(ros::Duration(0.1), &OsdPlugin::timer_100ms_cb, this, false);
                _front_height=0;
            }

            private:
            void firstsubtwoframe_cb(const uat_msg::TD550FirstsubTwoframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                _vehicle_info.longitude=msg->lon;
                _vehicle_info.latitude=msg->lat;

                _vehicle_info.update_time = ros::Time::now();

                ROS_INFO("plugins receive data!!!!!");
                std::cout<<"plugins receive data!!!!!"<<std::endl;
            }

            void firstsubthreeframe_cb(const uat_msg::TD550FirstsubThreeframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                
                if(0<_takeoffHeight<7000)
                {
                    _vehicle_info.height=msg->RelHeight-_takeoffHeight;
                    _front_height=_vehicle_info.height;
                }
                else{
                    _vehicle_info.height=_front_height;
                }
                
                _vehicle_info.altitude=msg->AbsolutelyHeight;
                _vehicle_info.vs=msg->ZSpeed;
                _vehicle_info.gs=msg->XSpeed;
                _vehicle_info.heading=msg->Yaw;
                _vehicle_info.pitch=msg->Pitch;
                _vehicle_info.roll=msg->Roll;
            }

            void secondsubfourframe_cb(const uat_msg::TD550SecondsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _vehicle_info.fly_time=msg->FlightTime;
                _vehicle_info.voltage=msg->Power24V;
                _takeoffHeight=msg->takeoffHeight;
            }

            void thirdsubtwoframe_cb(const uat_msg::TD550ThirdsubTwoframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _vehicle_info.fuel=msg->Oil+msg->OilRight;
            }

            void timer_1s_cb(const ros::TimerEvent &event)
            {
                osd_publish();
            }

            void timer_100ms_cb(const ros::TimerEvent &event)
            {
                if(ros::Time::now() - _vehicle_info.update_time > ros::Duration(1.0f))
                {
                    _vehicle_info.down_link_state=1;
                }
                else if(ros::Time::now() - _vehicle_info.update_time > ros::Duration(3.0f))
                {
                    _vehicle_info.down_link_state=0;
                }
                else
                {
                    _vehicle_info.down_link_state=2;
                }
            }

            private:
            std::string generateString(const std::string &sn)
            {
                std::time_t t = ros::Time::now().toSec();
                std::tm *tm = std::localtime(&t);
                char buffer[9]; // YYYYMMDD, 8 characters + null terminator
                std::strftime(buffer, sizeof(buffer), "%Y%m%d", tm);
                std::string date(buffer);
                std::string prefix = _vehicle_info.uas_model;
                std::string suffix = "-1";
                return prefix + sn + "-" + date + suffix;
            }

            private:
            void osd_publish()
            {
                std::string strOsd;
                {
                    lock_guard lock(_data_mutex);
                    if (ros::Time::now() - _vehicle_info.update_time > ros::Duration(3.0f))
                        return;

                    cJSON *root = generate_json_root("osd");
                    cJSON *data = cJSON_CreateObject();
                    if (root == NULL || data == NULL)
                    {
                        cJSON_Delete(root);
                        cJSON_Delete(data);
                        return;
                    }

                    cJSON_AddStringToObject(data, "order_id", generateString(get_device_sn().c_str()).c_str());
                    cJSON_AddStringToObject(data, "manufacturer_id", "UAT");
                    cJSON_AddStringToObject(data, "uas_id", "UAT");
                    cJSON_AddNumberToObject(data, "timestamp", ros::Time::now().toNSec() / 1000000);
                    cJSON_AddStringToObject(data, "source", "device");
                    cJSON_AddStringToObject(data, "uas_model", "TD550");
                    cJSON_AddNumberToObject(data, "coordinate", 1);
                    cJSON_AddNumberToObject(data, "longitude", _vehicle_info.longitude);
                    cJSON_AddNumberToObject(data, "latitude", _vehicle_info.latitude);
                    cJSON_AddNumberToObject(data, "height", _vehicle_info.height);
                    cJSON_AddNumberToObject(data, "altitude", _vehicle_info.altitude);
                    cJSON_AddNumberToObject(data, "vs", _vehicle_info.vs);
                    cJSON_AddNumberToObject(data, "gs", _vehicle_info.gs);
                    cJSON_AddNumberToObject(data, "heading", _vehicle_info.heading);
                    cJSON_AddNumberToObject(data, "pitch", _vehicle_info.pitch);
                    cJSON_AddNumberToObject(data, "roll", _vehicle_info.roll);
                    cJSON_AddNumberToObject(data, "battery", 0);
                    cJSON_AddNumberToObject(data, "voltage", _vehicle_info.voltage);
                    cJSON_AddNumberToObject(data, "fuel", _vehicle_info.fuel);
                    cJSON_AddStringToObject(data, "mode", "MANUAL");
                    cJSON_AddNumberToObject(data, "flying", 2);
                    cJSON_AddNumberToObject(data, "fly_time", _vehicle_info.fly_time);
                    cJSON_AddNumberToObject(data, "fly_distance", 0);
                    cJSON_AddNumberToObject(data, "home_set", 1);
                    cJSON_AddNumberToObject(data, "home_distance", 0);
                    cJSON *home_location = cJSON_CreateObject();
                    cJSON_AddNumberToObject(home_location, "altitude", 0);
                    cJSON_AddNumberToObject(home_location, "latitude", 0);
                    cJSON_AddNumberToObject(home_location, "longitude", 0);
                    cJSON_AddItemToObject(data, "home_location", home_location);

                    // add state object
                    cJSON *state = cJSON_CreateObject();
                    cJSON_AddNumberToObject(state, "safe_enabled", 0);
                    cJSON_AddNumberToObject(state, "rtk_enabled", 1);
                    cJSON_AddNumberToObject(state, "rtk_connected", 0);
                    cJSON_AddNumberToObject(state, "rc_connected", 1);
                    cJSON_AddNumberToObject(state, "obs_enabled", 1);
                    cJSON_AddNumberToObject(state, "accel_cal", 1);
                    cJSON_AddNumberToObject(state, "gyro_cal", 1);
                    cJSON_AddNumberToObject(state, "hor_cal", 1);
                    cJSON_AddNumberToObject(state, "mag_cal", 1);
                    cJSON_AddNumberToObject(state, "fpv_live", 1);
                    cJSON_AddItemToObject(data, "state", state); // add state to data

                    cJSON_AddNumberToObject(data, "down_link_state", _vehicle_info.down_link_state);

                    cJSON_AddItemToObject(root, "data", data);

                    // add reply
                    cJSON_AddNumberToObject(root, "need_reply", 1);

                    char *jsonString = cJSON_Print(root);
                    strOsd = jsonString;
                    cJSON_Delete(root);
                    free(jsonString);
                }
                std_msgs::String rosString;
                rosString.data = strOsd.c_str();
                //std::cout<<strOsd<<std::endl;
                publishRos2mqtt(rosString);
            }

            private:
            Vehicle_Info _vehicle_info;
            ros::Subscriber first_two_sub;
            ros::Subscriber first_three_sub;
            ros::Subscriber second_four_sub;
            ros::Subscriber third_one_sub;
            ros::Subscriber third_two_sub;
            ros::Timer _timer_call_1s;
            ros::Timer _timer_call_100ms;

            std::mutex _data_mutex;

            float _takeoffHeight;
            float _front_height;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::OsdPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
