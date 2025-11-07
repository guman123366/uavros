
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
        struct Drivetrain_Info
        {
            float switch_K4=0;                          //右到位开关K4
            float switch_K1=0;                          //下限位开关K1
            float tension_state=0;                      //张紧状态指示
            float switch_K3=0;                          //左到位开关K3
            float switch_K2=0;                          //上限位开关K2
            float rotor_speed=0;                        //旋翼转速
            float Reducer_lubricating_oil_temperature=0;//减速器滑油温度
            float reducer_lubricating_oil_pressure=0;   //减速器滑油压力
            int main_steer_state_D1=0;                  //D1舵机主余度状态
            int backup_steer_state_D1=0;                //D1舵机备余度状态
            int main_steer_state_D2=0;                  //D2舵机主余度状态
            int backup_steer_state_D2=0;                //D2舵机备余度状态
            int main_steer_state_D3=0;                  //D3舵机主余度状态
            int backup_steer_state_D3=0;                //D3舵机备余度状态
            int main_steer_state_U1=0;                  //U1舵机主余度状态
            int backup_steer_state_U1=0;                //U1舵机备余度状态
            int main_steer_state_U2=0;                  //U2舵机主余度状态
            int backup_steer_state_U2=0;                //U2舵机备余度状态
            int main_steer_state_U3=0;                  //U3舵机主余度状态
            int backup_steer_state_U3=0;                //U3舵机主余度状态
            int main_steer_state_door=0;                //风门舵机主余度状态
            int backup_steer_state_door=0;              //风门舵机备余度状态
            int main_steer_state_direction=0;           //方向舵舵机主余度状态
            int backup_steer_state_direction=0;         //方向舵舵机备余度状态
            ros::Time update_time = ros::Time::now();
        };

        class DrivetrainPlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            DrivetrainPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~DrivetrainPlugin() {};
            virtual void initialize()
            {
                ROS_INFO("DrivetrainPlugin  initialize");
                _ros2mqtt_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/telemetry", 10);
                first_four_sub = _nh.subscribe<uat_msg::TD550FirstsubFourframe>("/telemeter_data_firstsubfourframe", 100, &DrivetrainPlugin::firstsubfourframe_cb, this);
                first_three_sub = _nh.subscribe<uat_msg::TD550FirstsubThreeframe>("/telemeter_data_firstsubthreeframe", 100, &DrivetrainPlugin::firstsubthreeframe_cb, this);
                third_one_sub = _nh.subscribe<uat_msg::TD550ThirdsubOneframe>("/telemeter_data_thirdsuboneframe", 100, &DrivetrainPlugin::thirdsuboneframe_cb, this);
                third_two_sub = _nh.subscribe<uat_msg::TD550ThirdsubTwoframe>("/telemeter_data_thirdsubtwoframe", 100, &DrivetrainPlugin::thirdsubtwoframe_cb, this);
                _timer_call_1s = _nh.createTimer(ros::Duration(1), &DrivetrainPlugin::timer_1s_cb, this, false);
            }

          private:
            void firstsubthreeframe_cb(const uat_msg::TD550FirstsubThreeframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                
                _drivetrain_info.rotor_speed=msg->MainRotor;

                _drivetrain_info.update_time = ros::Time::now();
            }

            void firstsubfourframe_cb(const uat_msg::TD550FirstsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                
                _drivetrain_info.main_steer_state_D1=msg->D1ServoMFault;
                _drivetrain_info.backup_steer_state_D1=msg->D1ServoBFault;
                _drivetrain_info.main_steer_state_D2=msg->D2ServoMFault;
                _drivetrain_info.backup_steer_state_D2=msg->D2ServoBFault;
                _drivetrain_info.main_steer_state_D3=msg->D3ServoMFault;
                _drivetrain_info.backup_steer_state_D3=msg->D3ServoBFault;
                _drivetrain_info.main_steer_state_U1=msg->U1ServoMFault;
                _drivetrain_info.backup_steer_state_U1=msg->U1ServoBFault;
                _drivetrain_info.main_steer_state_U2=msg->U2ServoMFault;
                _drivetrain_info.backup_steer_state_U2=msg->U2ServoBFault;
                _drivetrain_info.main_steer_state_U3=msg->U3ServoMFault;
                _drivetrain_info.backup_steer_state_U3=msg->U3ServoBFault;
                _drivetrain_info.main_steer_state_door=msg->DamperServoMFault;
                _drivetrain_info.backup_steer_state_door=msg->DamperServoBFault;
                _drivetrain_info.main_steer_state_direction=msg->DirServoMFault;
                _drivetrain_info.backup_steer_state_direction=msg->DirServoBFault;
            }

            void thirdsuboneframe_cb(const uat_msg::TD550ThirdsubOneframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                
                _drivetrain_info.switch_K4=msg->OpenK4;
                _drivetrain_info.switch_K1=msg->OpenK1;
                _drivetrain_info.tension_state=msg->Tension;
                _drivetrain_info.switch_K2=msg->OpenK2;
            }

            void thirdsubtwoframe_cb(const uat_msg::TD550ThirdsubTwoframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _drivetrain_info.switch_K3=msg->OpenK3;
                _drivetrain_info.Reducer_lubricating_oil_temperature=msg->ROiltemp;
                _drivetrain_info.reducer_lubricating_oil_pressure=msg->ROilPress;

            }

            void timer_1s_cb(const ros::TimerEvent &event)
            {
                main_ins_publish();
            }

          private:
            void main_ins_publish()
            {
                std::string strDrivetrain;
                {
                    lock_guard lock(_data_mutex);
                    if (ros::Time::now() - _drivetrain_info.update_time > ros::Duration(3.0f))
                        return;

                    cJSON *root = generate_json_root("drivetrain_td550_t1400");
                    cJSON *data = cJSON_CreateObject();
                    if (root == NULL || data == NULL)
                    {
                        cJSON_Delete(root);
                        cJSON_Delete(data);
                        return;
                    }

                    cJSON_AddNumberToObject(data, "switch_K4", _drivetrain_info.switch_K4);
                    cJSON_AddNumberToObject(data, "switch_K1", _drivetrain_info.switch_K1);
                    cJSON_AddNumberToObject(data, "tension_state", _drivetrain_info.tension_state);
                    cJSON_AddNumberToObject(data, "switch_K3", _drivetrain_info.switch_K3);
                    cJSON_AddNumberToObject(data, "switch_K2", _drivetrain_info.switch_K2);
                    cJSON_AddNumberToObject(data, "rotor_speed", _drivetrain_info.rotor_speed);
                    cJSON_AddNumberToObject(data, "Reducer_lubricating_oil_temperature", _drivetrain_info.Reducer_lubricating_oil_temperature);
                    cJSON_AddNumberToObject(data, "reducer_lubricating_oil_pressure", _drivetrain_info.reducer_lubricating_oil_pressure);
                    cJSON_AddNumberToObject(data, "main_steer_state_D1", _drivetrain_info.main_steer_state_D1);
                    cJSON_AddNumberToObject(data, "backup_steer_state_D1", _drivetrain_info.backup_steer_state_D1);
                    cJSON_AddNumberToObject(data, "main_steer_state_D2", _drivetrain_info.main_steer_state_D2);
                    cJSON_AddNumberToObject(data, "backup_steer_state_D2", _drivetrain_info.backup_steer_state_D2);
                    cJSON_AddNumberToObject(data, "main_steer_state_D3", _drivetrain_info.main_steer_state_D3);
                    cJSON_AddNumberToObject(data, "backup_steer_state_D3", _drivetrain_info.backup_steer_state_D3);
                    cJSON_AddNumberToObject(data, "main_steer_state_U1", _drivetrain_info.main_steer_state_U1);
                    cJSON_AddNumberToObject(data, "backup_steer_state_U1", _drivetrain_info.backup_steer_state_U1);
                    cJSON_AddNumberToObject(data, "main_steer_state_U2", _drivetrain_info.main_steer_state_U2);
                    cJSON_AddNumberToObject(data, "backup_steer_state_U2", _drivetrain_info.backup_steer_state_U2);
                    cJSON_AddNumberToObject(data, "main_steer_state_U3", _drivetrain_info.main_steer_state_U3);
                    cJSON_AddNumberToObject(data, "backup_steer_state_U3", _drivetrain_info.backup_steer_state_U3);
                    cJSON_AddNumberToObject(data, "main_steer_state_door", _drivetrain_info.main_steer_state_door);
                    cJSON_AddNumberToObject(data, "backup_steer_state_door", _drivetrain_info.backup_steer_state_door);
                    cJSON_AddNumberToObject(data, "main_steer_state_direction", _drivetrain_info.main_steer_state_direction);
                    cJSON_AddNumberToObject(data, "backup_steer_state_direction", _drivetrain_info.backup_steer_state_direction);

                    cJSON_AddItemToObject(root, "data", data);
                    char *jsonString = cJSON_Print(root);
                    strDrivetrain = jsonString;
                    cJSON_Delete(root);
                    free(jsonString);
                }
                std_msgs::String rosString;
                rosString.data = strDrivetrain.c_str();
                //std::cout<<strDrivetrain<<std::endl;
                publishRos2mqtt(rosString);
            }

          private:
            Drivetrain_Info _drivetrain_info;
            ros::Subscriber first_four_sub;
            ros::Subscriber first_three_sub;
            ros::Subscriber third_one_sub;
            ros::Subscriber third_two_sub;
            ros::Timer _timer_call_1s;

            std::mutex _data_mutex;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::DrivetrainPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
