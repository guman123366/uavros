
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
        struct Main_Ins_Info
        {
            int main_mems_state=0;            //主MEMS惯导状态字
            int main_mems_error=0;            //MEMS主惯导故障字        
            float mian_pdop=0;                //卫星PSTD值
            int sat_num=0;                    //卫星颗数
            int BD_positon_station=0;         //卫导接收机定位状态字 
            int backup_mems_state=0;          //备MEMS惯导状态
            int disperse_outRe2=0;            //离散输出回绕2
            float mems_temp=0;                //主MEMS温度
            int backup_mems_error=0;          //备MEMS惯导故障字
            float takeoff_airport_elevation=0;//起飞机场场高
            float backup_pdop=0;              //备惯导PSTD值
            float eastspeed=0;                //东向速度
            float northspeed=0;               //北向速度
            float downspeed=0;                //地向速度
            float airspeed=0;                 //空速
            float yawrate=0;                //偏航角速率
            float longitudinal_speed=0;       //纵向速度
            float lateral_speed=0;            //侧向速度
            ros::Time update_time = ros::Time::now();
        };

        class MainInsPlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            MainInsPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~MainInsPlugin() {};
            virtual void initialize()
            {
                ROS_INFO("MainInsPlugin  initialize");
                _ros2mqtt_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/telemetry", 10);
                first_one_sub = _nh.subscribe<uat_msg::TD550FirstsubOneframe>("/telemeter_data_firstsuboneframe", 100, &MainInsPlugin::firstsuboneframe_cb, this);
                first_three_sub = _nh.subscribe<uat_msg::TD550FirstsubThreeframe>("/telemeter_data_firstsubthreeframe", 100, &MainInsPlugin::firstsubthreeframe_cb, this);
                second_one_sub=_nh.subscribe<uat_msg::TD550SecondsubOneframe>("/telemeter_data_secondsuboneframe", 100, &MainInsPlugin::secondsuboneframe_cb, this);
                second_two_sub=_nh.subscribe<uat_msg::TD550SecondsubTwoframe>("/telemeter_data_secondsubtwoframe", 100, &MainInsPlugin::secondsubtwoframe_cb, this);
                second_three_sub=_nh.subscribe<uat_msg::TD550SecondsubThreeframe>("/telemeter_data_secondsubthreeframe", 100, &MainInsPlugin::secondsubthreeframe_cb, this);
                second_four_sub=_nh.subscribe<uat_msg::TD550SecondsubFourframe>("/telemeter_data_secondsubfourframe", 100, &MainInsPlugin::secondsubfourframe_cb, this);
                third_one_sub = _nh.subscribe<uat_msg::TD550ThirdsubOneframe>("/telemeter_data_thirdsuboneframe", 100, &MainInsPlugin::thirdsuboneframe_cb, this);
                third_four_sub = _nh.subscribe<uat_msg::TD550ThirdsubFourframe>("/telemeter_data_thirdsubfourframe", 100, &MainInsPlugin::thirdsubfourframe_cb, this);
                _timer_call_1s = _nh.createTimer(ros::Duration(1), &MainInsPlugin::timer_1s_cb, this, false);
            }

          private:
            void firstsuboneframe_cb(const uat_msg::TD550FirstsubOneframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                _main_ins_info.backup_mems_state=msg->NaviState;
                _main_ins_info.airspeed=msg->TureAirSpeed;

                _main_ins_info.update_time = ros::Time::now();
            }
          
            void firstsubthreeframe_cb(const uat_msg::TD550FirstsubThreeframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                
                _main_ins_info.eastspeed=msg->EastSpeed;
                _main_ins_info.northspeed=msg->NothSpeed;
                _main_ins_info.downspeed=msg->ZSpeed;
                _main_ins_info.yawrate=msg->YawAngVelocity;
                _main_ins_info.longitudinal_speed=msg->XSpeed;
                _main_ins_info.lateral_speed=msg->YSpeed;
            }

            void secondsuboneframe_cb(const uat_msg::TD550SecondsubOneframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _main_ins_info.main_mems_state=static_cast<int>(msg->InertialState);
                _main_ins_info.main_mems_error=msg->MemsWarningState;
                _main_ins_info.mian_pdop=msg->BDPdop;
                _main_ins_info.sat_num=msg->MainNavNum;
            }

            void secondsubtwoframe_cb(const uat_msg::TD550SecondsubTwoframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _main_ins_info.BD_positon_station=msg->NaviState;
            }

            void secondsubthreeframe_cb(const uat_msg::TD550SecondsubThreeframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                memcpy(&secondsubthreeframe,msg.get(),sizeof(uat_msg::TD550SecondsubThreeframe));
            }


            void secondsubfourframe_cb(const uat_msg::TD550SecondsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _main_ins_info.takeoff_airport_elevation=msg->takeoffHeight;
                _main_ins_info.backup_mems_error=msg->BackupNaviError3*0x10000+msg->BackupNaviError2*0x100+secondsubthreeframe.BackupNaviError1;
            }

            void thirdsuboneframe_cb(const uat_msg::TD550ThirdsubOneframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                _main_ins_info.disperse_outRe2=msg->DisperseOutRe2;
                _main_ins_info.mems_temp=msg->MemsTemp;
            }

            void thirdsubfourframe_cb(const uat_msg::TD550ThirdsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _main_ins_info.backup_pdop=msg->NaviPDOP;
            }

            void timer_1s_cb(const ros::TimerEvent &event)
            {
                main_ins_publish();
            }

          private:
            void main_ins_publish()
            {
                std::string strMainIns;
                {
                    lock_guard lock(_data_mutex);
                    if (ros::Time::now() - _main_ins_info.update_time > ros::Duration(3.0f))
                        return;

                    cJSON *root = generate_json_root("main_ins");
                    cJSON *data = cJSON_CreateObject();
                    if (root == NULL || data == NULL)
                    {
                        cJSON_Delete(root);
                        cJSON_Delete(data);
                        return;
                    }

                    cJSON_AddNumberToObject(data, "main_mems_state", _main_ins_info.main_mems_state);
                    cJSON_AddNumberToObject(data, "main_mems_error", _main_ins_info.main_mems_error);
                    cJSON_AddNumberToObject(data, "mian_pdop", _main_ins_info.mian_pdop);
                    cJSON_AddNumberToObject(data, "sat_num", _main_ins_info.sat_num);
                    cJSON_AddNumberToObject(data, "BD_positon_station", _main_ins_info.BD_positon_station);
                    cJSON_AddNumberToObject(data, "backup_mems_state", _main_ins_info.backup_mems_state);

                    cJSON_AddNumberToObject(data, "disperse_outRe2", _main_ins_info.disperse_outRe2);

                    cJSON_AddNumberToObject(data, "mems_temp", _main_ins_info.mems_temp);
                    cJSON_AddNumberToObject(data, "backup_mems_error", _main_ins_info.backup_mems_error);
                    cJSON_AddNumberToObject(data, "takeoff_airport_elevation", _main_ins_info.takeoff_airport_elevation);
                    cJSON_AddNumberToObject(data, "backup_pdop", _main_ins_info.backup_pdop);


                    cJSON_AddNumberToObject(data, "east_speed", _main_ins_info.eastspeed);
                    cJSON_AddNumberToObject(data, "north_speed", _main_ins_info.northspeed);
                    cJSON_AddNumberToObject(data, "down_speed", _main_ins_info.downspeed);
                    cJSON_AddNumberToObject(data, "airspeed", _main_ins_info.airspeed);
                    cJSON_AddNumberToObject(data, "yaw_rate", _main_ins_info.yawrate);

                    cJSON_AddNumberToObject(data, "longitudinal_speed", _main_ins_info.longitudinal_speed);
                    cJSON_AddNumberToObject(data, "lateral_speed", _main_ins_info.lateral_speed);

                    cJSON_AddItemToObject(root, "data", data);
                    char *jsonString = cJSON_Print(root);
                    strMainIns = jsonString;
                    cJSON_Delete(root);
                    free(jsonString);
                }
                std_msgs::String rosString;
                rosString.data = strMainIns.c_str();
                //std::cout<<strMainIns<<std::endl;
                publishRos2mqtt(rosString);
            }

          private:
            Main_Ins_Info _main_ins_info;
            ros::Subscriber first_one_sub;
            ros::Subscriber first_three_sub;
            ros::Subscriber second_one_sub;
            ros::Subscriber second_two_sub;
            ros::Subscriber second_three_sub;
            ros::Subscriber second_four_sub;
            ros::Subscriber third_one_sub;
            ros::Subscriber third_four_sub;
            ros::Timer _timer_call_1s;

            std::mutex _data_mutex;

            uat_msg::TD550SecondsubThreeframe secondsubthreeframe;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::MainInsPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
