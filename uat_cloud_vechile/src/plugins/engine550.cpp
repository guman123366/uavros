
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
        struct Engine550_Info
        {
            float intake_manifold_temperature=0;        //进气歧管温度
            float ECUB_Engine_speed=0;                  //ECUB发动机转速
            float fuel_pressure=0;                      //燃油压力
            float fuel_quantity=0;                      //燃油油量
            int engine_inair_rate=0;                  //发动机空气流量
            float ECUA_Engine_speed=0;                  //ECUA发动机转速
            float fuel_flow_rate=0;                     //燃油流量
            float intake_manifold_pressure=0;           //进气歧管压力
            float engine_oil_pressure=0;                //发动机滑油压力
            float engine_oil_temperature=0;             //发动机滑油温度
            float coolant_temperature=0;                  //冷却液温度
            float exhaust_temperature1=0;                 //ECU排气温度1
            float exhaust_temperature2=0;                 //ECU排气温度2
            float exhaust_temperature3=0;                 //ECU排气温度3
            float exhaust_temperature4=0;                 //ECU排气温度4
            float wind_gate_position=0;                   //风门位置
            float relief_valve_position=0;                 //放气阀位置
            float ECUA_ECU_BUS_voltage=0;                 //ECUA ECU-BUS电压
            float ECUB_ECU_BUS_voltage=0;                 //ECUB ECU-BUS电压
            float boost_pressure=0;                       //增压压力
            float intake_pressure=0;                      //发动机进气压力
            int water_cooling_fan_control=0;            //发动机水散热风扇控制
            int intercooler_fan_control=0;              //发动机中冷风扇控制
            float cooling_fan_current=0;                  //中冷风扇供电电流
            float water_cooled_fan_current=0;             //水冷风扇供电电流
            float cylinder_temperature=0;                 //缸体温度
            float minimum_coolant_temperature=0;          //冷却液温度最低值
            float maximum_temperature=0;                  //排温最高值
            float minimum_temperature=0;                  //排温最低值
            int fly_remaind_time=0;                     //剩余航行时间
            int engine_runtime=0;                       //发动机运行时间
            ros::Time update_time = ros::Time::now();
        };

        class Engine550Plugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            Engine550Plugin() : cloud_vehicle_bridge::PluginBase() {};
            ~Engine550Plugin() {};
            virtual void initialize()
            {
                ROS_INFO("Engine550Plugin initialize");
                _ros2mqtt_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/telemetry", 10);
                first_three_sub=_nh.subscribe<uat_msg::TD550FirstsubThreeframe>("/telemeter_data_firstsubthreeframe", 100, &Engine550Plugin::firstsubthreeframe_cb, this);
                second_four_sub=_nh.subscribe<uat_msg::TD550SecondsubFourframe>("/telemeter_data_secondsubfourframe", 100, &Engine550Plugin::secondsubfourframe_cb, this);
                third_one_sub = _nh.subscribe<uat_msg::TD550ThirdsubOneframe>("/telemeter_data_thirdsuboneframe", 100, &Engine550Plugin::thirdsuboneframe_cb, this);
                third_two_sub = _nh.subscribe<uat_msg::TD550ThirdsubTwoframe>("/telemeter_data_thirdsubtwoframe", 100, &Engine550Plugin::thirdsubtwoframe_cb, this);
                third_three_sub = _nh.subscribe<uat_msg::TD550ThirdsubThreeframe>("/telemeter_data_thirdsubthreeframe", 100, &Engine550Plugin::thirdsubthreeframe_cb, this);
                third_four_sub = _nh.subscribe<uat_msg::TD550ThirdsubFourframe>("/telemeter_data_thirdsubfourframe", 100, &Engine550Plugin::thirdsubfourframe_cb, this);
                _timer_call_1s = _nh.createTimer(ros::Duration(1), &Engine550Plugin::timer_1s_cb, this, false);
            }

          private:
            void firstsubthreeframe_cb(const uat_msg::TD550FirstsubThreeframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                
                engine550_info.ECUA_Engine_speed=msg->EngineR;

                engine550_info.update_time = ros::Time::now();
            }

            void secondsubfourframe_cb(const uat_msg::TD550SecondsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                engine550_info.water_cooling_fan_control=msg->WaterFan;
                engine550_info.intercooler_fan_control=msg->MiddleFan;
                engine550_info.cooling_fan_current=msg->MidFanCurrent;
                engine550_info.water_cooled_fan_current=msg->WaterFacCurrent;
                engine550_info.fly_remaind_time=msg->FlightSurTime;
            }

            void thirdsuboneframe_cb(const uat_msg::TD550ThirdsubOneframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                
                engine550_info.fuel_flow_rate=msg->OilVolume;
            }

            void thirdsubtwoframe_cb(const uat_msg::TD550ThirdsubTwoframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                
                engine550_info.fuel_pressure=msg->OilPress;
                engine550_info.engine_oil_pressure=msg->EngineOilPre;
                engine550_info.coolant_temperature=msg->CoolantTemp;
                engine550_info.exhaust_temperature1=msg->ExhaustTemp1;
                engine550_info.exhaust_temperature2=msg->ExhaustTemp2;
                engine550_info.exhaust_temperature3=msg->ExhaustTemp3;
                engine550_info.exhaust_temperature4=msg->ExhaustTemp4;
                engine550_info.boost_pressure=msg->TurboPressure;
                engine550_info.cylinder_temperature=msg->Motortemp2;
            }

            void thirdsubthreeframe_cb(const uat_msg::TD550ThirdsubThreeframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);

                engine550_info.intake_manifold_temperature=msg->InairTemp;
                engine550_info.ECUB_Engine_speed=msg->EngineRAM;
                engine550_info.fuel_quantity=msg->OilUsed;
                engine550_info.engine_inair_rate=msg->EnineInAirTemp;
                engine550_info.intake_manifold_pressure=msg->InairPre;
                engine550_info.engine_oil_temperature=msg->EngineOilTemp;
                engine550_info.wind_gate_position=msg->DamperPostion;
                engine550_info.relief_valve_position=msg->OutAirPostion;
                engine550_info.ECUA_ECU_BUS_voltage=msg->EcuAbus;
                engine550_info.ECUB_ECU_BUS_voltage=msg->EcuBbus;
                engine550_info.minimum_coolant_temperature=msg->CoolingTempMin;
                engine550_info.maximum_temperature=msg->ExhaustTempmax;
                engine550_info.minimum_temperature=msg->ExhaustTempmin;
                engine550_info.engine_runtime=msg->EngineTime;
            }

            void thirdsubfourframe_cb(const uat_msg::TD550ThirdsubFourframe::ConstPtr &msg)
            {
                lock_guard lock(_data_mutex);
                
                engine550_info.intake_pressure=msg->EngineAirPre;
            }

            void timer_1s_cb(const ros::TimerEvent &event)
            {
                addition550_publish();
            }

          private:
            void addition550_publish()
            {
                std::string strEngine550;
                {
                    lock_guard lock(_data_mutex);
                    if (ros::Time::now() - engine550_info.update_time > ros::Duration(3.0f))
                        return;

                    cJSON *root = generate_json_root("engine_td550");
                    cJSON *data = cJSON_CreateObject();
                    if (root == NULL || data == NULL)
                    {
                        cJSON_Delete(root);
                        cJSON_Delete(data);
                        return;
                    }

                    cJSON_AddNumberToObject(data, "intake_manifold_temperature", engine550_info.intake_manifold_temperature);
                    cJSON_AddNumberToObject(data, "ECUB_Engine_speed", engine550_info.ECUB_Engine_speed);
                    cJSON_AddNumberToObject(data, "fuel_pressure", engine550_info.fuel_pressure);
                    cJSON_AddNumberToObject(data, "fuel_quantity", engine550_info.fuel_quantity);
                    cJSON_AddNumberToObject(data, "engine_inair_rate", engine550_info.engine_inair_rate);
                    cJSON_AddNumberToObject(data, "ECUA_Engine_speed", engine550_info.ECUA_Engine_speed);
                    cJSON_AddNumberToObject(data, "fuel_flow_rate", engine550_info.fuel_flow_rate);
                    cJSON_AddNumberToObject(data, "intake_manifold_pressure", engine550_info.intake_manifold_pressure);
                    cJSON_AddNumberToObject(data, "engine_oil_pressure", engine550_info.engine_oil_pressure);
                    cJSON_AddNumberToObject(data, "engine_oil_temperature", engine550_info.engine_oil_temperature);
                    cJSON_AddNumberToObject(data, "coolant_temperature", engine550_info.coolant_temperature);
                    cJSON_AddNumberToObject(data, "exhaust_temperature1", engine550_info.exhaust_temperature1);
                    cJSON_AddNumberToObject(data, "exhaust_temperature2", engine550_info.exhaust_temperature2);
                    cJSON_AddNumberToObject(data, "exhaust_temperature3", engine550_info.exhaust_temperature3);
                    cJSON_AddNumberToObject(data, "exhaust_temperature4", engine550_info.exhaust_temperature4);
                    cJSON_AddNumberToObject(data, "wind_gate_position", engine550_info.wind_gate_position);
                    cJSON_AddNumberToObject(data, "relief_valve_position", engine550_info.relief_valve_position);
                    cJSON_AddNumberToObject(data, "ECUA_ECU_BUS_voltage", engine550_info.ECUA_ECU_BUS_voltage);
                    cJSON_AddNumberToObject(data, "ECUB_ECU_BUS_voltage", engine550_info.ECUB_ECU_BUS_voltage);
                    cJSON_AddNumberToObject(data, "boost_pressure", engine550_info.boost_pressure);
                    cJSON_AddNumberToObject(data, "intake_pressure", engine550_info.intake_pressure);
                    cJSON_AddNumberToObject(data, "water_cooling_fan_control", engine550_info.water_cooling_fan_control);
                    cJSON_AddNumberToObject(data, "intercooler_fan_control", engine550_info.intercooler_fan_control);
                    cJSON_AddNumberToObject(data, "cooling_fan_current", engine550_info.cooling_fan_current);
                    cJSON_AddNumberToObject(data, "water_cooled_fan_current", engine550_info.water_cooled_fan_current);
                    cJSON_AddNumberToObject(data, "cylinder_temperature", engine550_info.cylinder_temperature);
                    cJSON_AddNumberToObject(data, "minimum_coolant_temperature", engine550_info.minimum_coolant_temperature);
                    cJSON_AddNumberToObject(data, "maximum_temperature", engine550_info.maximum_temperature);
                    cJSON_AddNumberToObject(data, "minimum_temperature", engine550_info.minimum_temperature);
                    cJSON_AddNumberToObject(data, "fly_remaind_time", engine550_info.fly_remaind_time);
                    cJSON_AddNumberToObject(data, "engine_runtime", engine550_info.engine_runtime);

                    cJSON_AddItemToObject(root, "data", data);
                    char *jsonString = cJSON_Print(root);
                    strEngine550 = jsonString;
                    cJSON_Delete(root);
                    free(jsonString);
                }
                std_msgs::String rosString;
                rosString.data = strEngine550.c_str();
                //std::cout<<strAddition1400<<std::endl;
                publishRos2mqtt(rosString);
            }

          private:
            Engine550_Info engine550_info;
            ros::Subscriber first_three_sub;
            ros::Subscriber second_four_sub;
            ros::Subscriber third_one_sub;
            ros::Subscriber third_two_sub;
            ros::Subscriber third_three_sub;
            ros::Subscriber third_four_sub;
            ros::Timer _timer_call_1s;

            std::mutex _data_mutex;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::Engine550Plugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
