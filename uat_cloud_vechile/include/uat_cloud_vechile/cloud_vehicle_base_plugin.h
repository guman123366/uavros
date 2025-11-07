
#ifndef UAT_CLOUD_VEHICLE_BRIDGE_BASE_PLUGIN_H_
#define UAT_CLOUD_VEHICLE_BRIDGE_BASE_PLUGIN_H_

#include <uuid/uuid.h>

#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mavconn/interface.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/UAT_MqttAuthenStatus.h>
#include <map>
#include <mutex>
#include "cJSON.h"

namespace uat
{
namespace plugin
{
    #define UAT_MOUNT_A30         3342355
    #define UAT_MOUNT_Z40T        3407891
    #define UAT_MOUNT_ZT30        3538963
    #define UAT_MOUNT_A8Mini      1212452 //3604499 A8mini会重新适配，先用一个假的
    #define UAT_MOUNT_ZR30        3670035
    #define UAT_MOUNT_C30N        4063251
    #define UAT_MOUNT_U10         4390931 //(甲烷)
    #define UAT_MOUNT_NVG900      4587539
    // 探照灯:
    #define UAT_MOUNT_GL60P       3932179
    // 抛投器:
    #define UAT_MOUNT_4C          3997715
    // 升降锁:
    #define UAT_MOUNT_SZY         4128787
    #define UAT_MOUNT_YKX         4194323
    // 物流箱
    #define UAT_MOUNT_LC          3604499 
    
    

namespace cloud_vehicle_bridge
{
    class PluginBase
    {
    public:
        virtual void initialize() = 0;
        using Ptr = boost::shared_ptr<PluginBase>;
        virtual ~PluginBase(){};
        void set_device_sn(const std::string &serialNumber)
        {
            _device_sn = serialNumber;
        }
        
        std::string get_device_sn() const
        {
            return _device_sn;
        }

        virtual void set_uat_msgcode_data(const std::map<std::string, std::string> data)
        {
        }       

        
        void initializePluginBase()
        {
            _mqtt_authen_status_sub = _nh.subscribe<mavros_msgs::UAT_MqttAuthenStatus>("/uat/UAT_MqttAuthenStatus", 10, &PluginBase::mqtt_authen_status_cb, this); 
            //_mqtt2ros_services_sub = _nh.subscribe<std_msgs::String>("/uat_cloud_ros/command", 10, &PluginBase::mqtt_services_cb, this);
            //_mqtt2ros_telemetry_reply_sub = _nh.subscribe<std_msgs::String>("/uat_cloud_ros/telemetry_reply", 10, &PluginBase::mqtt_services_cb, this);
            
            _ros2mqtt_services_reply_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/command_ack", 10);
            _ros2mqtt_events_pub = _nh.advertise<std_msgs::String>("/uat_cloud_ros/events", 10);
        }

        std::string getMountType(int device_id)
        {
            std::string mountType = "unknown";
            switch (device_id)
            {
            case UAT_MOUNT_A30:
                mountType = "A30";
                break;
            case UAT_MOUNT_Z40T:
                mountType = "Z40T";
                break;
            case UAT_MOUNT_ZT30:
                mountType = "ZT30";
                break;
            case UAT_MOUNT_A8Mini:
                mountType = "A8Mini";
                break;
            case UAT_MOUNT_ZR30:
                mountType = "ZR30";
                break;
            case UAT_MOUNT_C30N:
                mountType = "C30N";
                break;
            case UAT_MOUNT_U10:
                mountType = "U10";
                break;
            case UAT_MOUNT_NVG900:
                mountType = "NVG900";
                break;
                // 探照灯:
            case UAT_MOUNT_GL60P:
                mountType = "GL60P";
                break;
                // 抛投器:
            case UAT_MOUNT_4C:
                mountType = "4C";
                break;
                // 升降锁:
            case UAT_MOUNT_SZY:
                mountType = "SZY";
                break;
            case UAT_MOUNT_YKX:
                mountType = "YKX";
                break;
                // 物流箱
            case UAT_MOUNT_LC:
                mountType = "LC";
                break;
            
            default:
                break;
            }
            return mountType;
        }
        static std::string getUUid()
        {
            uuid_t uuid;
            char uuid_str[37];
            uuid_generate(uuid);
            uuid_unparse(uuid, uuid_str);
            return std::string(uuid_str);
        }

        bool getStringValue(const cJSON* json, std::string strKey, std::string& strValue)
        {
            cJSON* value = cJSON_GetObjectItemCaseSensitive(json, strKey.c_str());
            if (cJSON_IsString(value) && (value->valuestring != NULL))
            {
                strValue = value->valuestring;
                return true;
            }
            return false;
        }

        bool getIntValue(const cJSON* json, std::string strKey, int& nValue)
        {
            if(nullptr==json)
                return false;

            cJSON* value = cJSON_GetObjectItemCaseSensitive(json, strKey.c_str());
            if(nullptr==value)
                return false;

            if (cJSON_IsNumber(value))
            {
                nValue = value->valueint;
                return true;
            }
            return false;
        }

        bool getDoubleValue(const cJSON* json, std::string strKey, double& dValue)
        {
            cJSON* value = cJSON_GetObjectItemCaseSensitive(json, strKey.c_str());
            if (cJSON_IsNumber(value))
            {
                dValue = value->valuedouble;
                return true;
            }
            return false;
        }

        bool getBoolValue(const cJSON* json, std::string strKey, bool& bValue)
        {
            cJSON* value = cJSON_GetObjectItemCaseSensitive(json, strKey.c_str());
            if (cJSON_IsBool(value))
            {
                if(value->valueint != 0)
                    bValue = true;
                else 
                    bValue = false;
                return true;
            }
            return false;
        }

        cJSON* generate_json_root(std::string method, std::string bid = getUUid(), std::string tid = getUUid(),
            double timestamp = ros::Time::now().toNSec() / 1000000) 
        {
            cJSON *root = cJSON_CreateObject();
            if (root == NULL)
                return NULL;
            cJSON_AddStringToObject(root, "tid", tid.c_str());
            cJSON_AddStringToObject(root, "bid", bid.c_str());
            cJSON_AddNumberToObject(root, "timestamp", timestamp);
            cJSON_AddStringToObject(root, "gateway", get_device_sn().c_str());
            cJSON_AddStringToObject(root, "method", method.c_str());
            return root;
        }   
        
        bool setParamValue(std::string paramName, int paramValue)
        {
            ros::ServiceClient param_client = _nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
            mavros_msgs::ParamSet paramSet;
            paramSet.request.param_id = paramName;
            paramSet.request.value.integer = paramValue;
            bool paramSetResult = false;
            for (int i = 0; i < 3; i++)
            {
                if (param_client.call(paramSet) && paramSet.response.success)
                {
                    paramSetResult = true;
                    break;
                }
                ros::Duration(0.1).sleep();
            }
            return paramSetResult;
        }

        bool setParamValue(std::string paramName, float paramValue)
        {
            ros::ServiceClient param_client = _nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
            mavros_msgs::ParamSet paramSet;
            paramSet.request.param_id = paramName;
            paramSet.request.value.real = paramValue;
            bool paramSetResult = false;
            for (int i = 0; i < 3; i++)
            {
                if (param_client.call(paramSet) && paramSet.response.success)
                {
                    paramSetResult = true;
                    break;
                }
                ros::Duration(0.1).sleep();
            }
            return paramSetResult;
        }

    protected:
        void publishRos2mqtt(const std_msgs::String& rosString)
        {
            lock_guard lock(_mutex);
            if (_mqtt_authen_status.MqttConnectState == mavros_msgs::UAT_MqttAuthenStatus::CONNECT_OK)
            {
                _ros2mqtt_pub.publish(rosString);

                ROS_INFO("send publishRos2mqtt!!!!!!!!!!!!!!");
            }
        }

    protected: 
        virtual void mqtt_authen_status_cb(const mavros_msgs::UAT_MqttAuthenStatus::ConstPtr &msg)
        {
            lock_guard lock(_mutex);
            _mqtt_authen_status = *msg;
        }

        virtual void mqtt_services_cb(const std_msgs::String::ConstPtr &msg)
        {
            cJSON *json = cJSON_Parse(msg->data.c_str());
            if (NULL == json)
            {
                const char *error_ptr = cJSON_GetErrorPtr();
                if (error_ptr != NULL)
                {
                    fprintf(stderr, "Error before: %s\n", error_ptr);
                }
                cJSON_Delete(json);
                return;
            }
            std::string strMethod = "";
            if (!getStringValue(json, "method", strMethod) ||  "" == strMethod)
                return;
            std::string strBid = "";
            if(!getStringValue(json, "bid", strBid) || "" == strBid)
            {
                return;
            }
            cJSON *jsonData = cJSON_GetObjectItemCaseSensitive(json, "data");
            if(jsonData == NULL)
            {
                do_need_reply(strMethod, strBid, -1, 0, "data error");
                return;
            }  

            handle_services_command(strBid, strMethod, jsonData);

            cJSON_Delete(json);
        }

        virtual void handle_services_command(const std::string& strBid, const std::string& strMethod, const cJSON *jsonData)
        {
            
        }

        void do_need_reply(std::string strMethod, std::string strBid, int type, int status, std::string message)
        {
            ROS_INFO("send mqtt services method is %s",strMethod.c_str());
            ROS_INFO("send mqtt services bid is %s",strBid.c_str());

            if (strMethod == "" || strBid == "")
            {
                return;
            }

            ROS_INFO("send mqtt services bid is %s",strBid.c_str());
            cJSON *jsonRoot = generate_json_root(strMethod, strBid);
            cJSON *jsonData = cJSON_CreateObject();
            if (jsonRoot == NULL || jsonData == NULL)
            {
                cJSON_Delete(jsonRoot);
                cJSON_Delete(jsonData);
                return;
            }
            cJSON_AddNumberToObject(jsonData, "type", type);
            cJSON_AddNumberToObject(jsonData, "code", status);
            cJSON_AddStringToObject(jsonData, "msg", message.c_str());
            cJSON_AddItemToObject(jsonRoot, "data", jsonData);
            char *jsonString = cJSON_Print(jsonRoot);
            std_msgs::String rosString;
            rosString.data = jsonString;
            _ros2mqtt_services_reply_pub.publish(rosString);
            cJSON_Delete(jsonRoot);
            free(jsonString);

            ROS_INFO("send mqtt services!!!!!!!!");
        }

        void do_events(std::string strMethod, std::string strBid, std::string status, std::string message, int progress, int need_reply)
        {
            cJSON *jsonRoot = generate_json_root(strMethod, strBid);
            cJSON *jsonData = cJSON_CreateObject();
            cJSON *jsonProgress = cJSON_CreateObject();
            if (jsonRoot == NULL || jsonData == NULL || jsonProgress == NULL)
            {
                cJSON_Delete(jsonRoot);
                cJSON_Delete(jsonData);
                cJSON_Delete(jsonProgress);
                return;
            }
            cJSON_AddStringToObject(jsonData, "status", status.c_str()); 
            cJSON_AddStringToObject(jsonData, "message", message.c_str());
            cJSON_AddStringToObject(jsonProgress, "step_key", strMethod.c_str());
            cJSON_AddNumberToObject(jsonProgress, "percent", progress);
            cJSON_AddItemToObject(jsonData, "progress", jsonProgress);
            cJSON_AddItemToObject(jsonRoot, "data", jsonData);
            cJSON_AddNumberToObject(jsonRoot, "need_reply", need_reply);
            char *jsonString = cJSON_Print(jsonRoot);
            std_msgs::String rosString;
            rosString.data = jsonString;
            _ros2mqtt_events_pub.publish(rosString);
            cJSON_Delete(jsonRoot);
            free(jsonString);
        }

    protected:
        PluginBase(): _nh() {}

    protected:
        std::string _device_sn;
        

        ros::NodeHandle _nh;

        using lock_guard = std::lock_guard<std::mutex>;
        std::mutex _mutex;

        mavros_msgs::UAT_MqttAuthenStatus _mqtt_authen_status;
        ros::Subscriber _mqtt_authen_status_sub;
        ros::Subscriber _mqtt2ros_services_sub;
        ros::Subscriber _mqtt2ros_telemetry_reply_sub;

        ros::Publisher _ros2mqtt_pub;
        ros::Publisher _ros2mqtt_services_reply_pub;
        ros::Publisher _ros2mqtt_events_pub;

        std::string _command_method;
        std::string _bid;
    };
};
};
};
#endif