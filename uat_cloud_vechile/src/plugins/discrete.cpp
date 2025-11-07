
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
        struct Discrete_Info
        {
            int fault_code=0;                           //故障编码
            int fc_computer_status=0;                   //飞控计算机工作状态字
            int fault_status_level=0;                   //故障状态等级
            int control_mode_status=0;                  //控制模式状态字
            int mobility_modal_status=0;                //机动功能模态状态字
            int Modal_input_status=0;                   //模态投入状态字
            int engine_status=0;                        //发动机状态字
            long signal_source_selection_state=0;        //信号源选择字
            int fault_comprehensive_character=0;        //故障综合字
            int airborne_equipment_status=0;            //机载设备状态字1
            int onboard_equipment_alarm_character1=0;   //机载设备告警字1
            int onboard_equipment_alarm_character2=0;   //机载设备告警字2
            int onboard_equipment_alarm_character3=0;   //机载设备告警字3
            int onboard_equipment_alarm_character4=0;   //机载设备告警字4
            ros::Time update_time = ros::Time::now();
        };

        class DiscretePlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            DiscretePlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~DiscretePlugin() {};
            virtual void initialize()
            {
                ROS_INFO("DiscretePlugin  initialize");
                _ros2mqtt_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/telemetry", 10);
                first_four_sub = _nh.subscribe<uat_msg::TD550FirstsubFourframe>("/telemeter_data_firstsubfourframe", 100, &DiscretePlugin::firstsubfourframe_cb, this);
                second_three_sub=_nh.subscribe<uat_msg::TD550SecondsubThreeframe>("/telemeter_data_secondsubthreeframe", 100, &DiscretePlugin::secondsubthreeframe_cb, this);
                second_four_sub=_nh.subscribe<uat_msg::TD550SecondsubFourframe>("/telemeter_data_secondsubfourframe", 100, &DiscretePlugin::secondsubfourframe_cb, this);
                _timer_call_1s = _nh.createTimer(ros::Duration(1), &DiscretePlugin::timer_1s_cb, this, false);
            }

          private:
            void firstsubfourframe_cb(const uat_msg::TD550FirstsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                
                _discrete_info.signal_source_selection_state=msg->SignalSource5*0x100000000+msg->SignalSource4*0x1000000+
                                                             msg->SignalSource3*0x10000+msg->SignalSource2*0x100+msg->SignalSource1;
                _discrete_info.fault_comprehensive_character=msg->FaultCode;
            
                _discrete_info.update_time = ros::Time::now();
            }

            void secondsubthreeframe_cb(const uat_msg::TD550SecondsubThreeframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _discrete_info.fault_code=msg->FCCError;
                _discrete_info.fc_computer_status=msg->FCCState;
                _discrete_info.fault_status_level=msg->FCCLevel;
                _discrete_info.control_mode_status=msg->ControlMode;
                _discrete_info.mobility_modal_status=msg->ManeuMode;
                _discrete_info.Modal_input_status=msg->ModeUsed;
                _discrete_info.engine_status=msg->EngineMode;
                
            }


            void secondsubfourframe_cb(const uat_msg::TD550SecondsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                _discrete_info.airborne_equipment_status=msg->EquipmentState;
                _discrete_info.onboard_equipment_alarm_character1=msg->EquipmentAlarm1;
                _discrete_info.onboard_equipment_alarm_character2=msg->EquipmentAlarm2;
                _discrete_info.onboard_equipment_alarm_character3=msg->EquipmentAlarm3;
                _discrete_info.onboard_equipment_alarm_character4=msg->EquipmentAlarm4;
            }

            void timer_1s_cb(const ros::TimerEvent &event)
            {
                discrete_publish();
            }

          private:
            void discrete_publish()
            {
                std::string strDiscrete;
                {
                    lock_guard lock(_data_mutex);
                    if (ros::Time::now() - _discrete_info.update_time > ros::Duration(3.0f))
                        return;

                    cJSON *root = generate_json_root("discrete_td550_t1400");
                    cJSON *data = cJSON_CreateObject();
                    if (root == NULL || data == NULL)
                    {
                        cJSON_Delete(root);
                        cJSON_Delete(data);
                        return;
                    }

                    cJSON_AddNumberToObject(data, "fault_code", _discrete_info.fault_code);
                    cJSON_AddNumberToObject(data, "fc_computer_status", _discrete_info.fc_computer_status);
                    cJSON_AddNumberToObject(data, "fault_status_level", _discrete_info.fault_status_level);
                    cJSON_AddNumberToObject(data, "control_mode_status", _discrete_info.control_mode_status);
                    cJSON_AddNumberToObject(data, "mobility_modal_status", _discrete_info.mobility_modal_status);
                    cJSON_AddNumberToObject(data, "Modal_input_status", _discrete_info.Modal_input_status);
                    cJSON_AddNumberToObject(data, "engine_status", _discrete_info.engine_status);
                    cJSON_AddNumberToObject(data, "signal_source_selection_state", _discrete_info.signal_source_selection_state);
                    cJSON_AddNumberToObject(data, "fault_comprehensive_character", _discrete_info.fault_comprehensive_character);
                    cJSON_AddNumberToObject(data, "airborne_equipment_status", _discrete_info.airborne_equipment_status);
                    cJSON_AddNumberToObject(data, "onboard_equipment_alarm_character1", _discrete_info.onboard_equipment_alarm_character1);
                    cJSON_AddNumberToObject(data, "onboard_equipment_alarm_character2", _discrete_info.onboard_equipment_alarm_character2);
                    cJSON_AddNumberToObject(data, "onboard_equipment_alarm_character3", _discrete_info.onboard_equipment_alarm_character3);
                    cJSON_AddNumberToObject(data, "onboard_equipment_alarm_character4", _discrete_info.onboard_equipment_alarm_character4);

                    cJSON_AddItemToObject(root, "data", data);
                    char *jsonString = cJSON_Print(root);
                    strDiscrete = jsonString;
                    cJSON_Delete(root);
                    free(jsonString);
                }
                std_msgs::String rosString;
                rosString.data = strDiscrete.c_str();
                //std::cout<<strDiscrete<<std::endl;
                publishRos2mqtt(rosString);
            }

          private:
            Discrete_Info _discrete_info;
            ros::Subscriber first_four_sub;
            ros::Subscriber second_three_sub;
            ros::Subscriber second_four_sub;
            ros::Timer _timer_call_1s;

            std::mutex _data_mutex;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::DiscretePlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
