#include "CloudExchangeNx.hpp"
#include <string>
#include <sstream>

//#define  TEST_IDENTIFY 
namespace CLOUDEXCHANGENX {  
    // static const char *s_url = "http://airport.uatair.com:9999/driver/auth/authenticate";
    struct DRIVER_DATA_CODE::mg_authen_cmd mg_authen_cmd;
    struct DRIVER_DATA_CODE::mg_authen_state mg_authen_state;
    std::string res_string;
    ros::ServiceClient HangarCtlClient;
    ros::Subscriber uat_hangerstatus_sub;
    // Print HTTP response and signal that we're done
    static void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
        if (ev == MG_EV_CONNECT) {
            // Connected to server. Extract host name from URL
            struct mg_str host = mg_url_host(mg_authen_cmd.authen_url.c_str());  //s_url

            // If s_url is https://, tell client connection to use TLS
            if (mg_url_is_ssl(mg_authen_cmd.authen_url.c_str())) { //s_url
                struct mg_tls_opts opts = {.ca = "ca.pem", .srvname = host};
                mg_tls_init(c, &opts);
            }

            // Send request
            // mg_printf(c, "GET %s HTTP/1.0\r\n"
            //         "Host: %.*s\r\n"
            //         "\r\n",
            //         mg_url_uri(s_url), (int) host.len, host.ptr);          "Content-Type: application/x-www-form-urlencoded\r\n"

            CLOUD_AUTHEN_EVENT::cloud_warehouse_encode(mg_authen_cmd, res_string);
            // mg_printf(c, "POST /driver/auth/authenticate HTTP/1.1\r\n"
            //             "Connection: Keep-Alive\r\n"
            //             "Content-Type: application/json\r\n"
            //             "Accept: application/json, text/plain, */*\r\n"
            //             "Accept-Encoding: gzip, deflate\r\n"
            //             "User-Agent: PostmanRuntime/7.39.0\r\n"
            //             "Cache-Control: no-cache\r\n"
            //             "Postman-Token: 82f941cf-fcec-4b03-b0a8-3204b1a50742\r\n"
            //             "Host: airport.uatair.com:9999\r\n"
            //             "Content-Length: %d\r\n\r\n"
            //             "%s",
            //             res_string.length() + 1, res_string.c_str());  //s_url
            mg_printf(c, "POST %s HTTP/1.1\r\n"
                        "Connection: Keep-Alive\r\n"
                        "Content-Type: application/json\r\n"
                        "Accept: application/json, text/plain, */*\r\n"
                        "Accept-Encoding: gzip, deflate\r\n"
                        "Content-Length: %d\r\n\r\n"  //must use \r\n\r\n  
                        "%s",
                        mg_url_uri(mg_authen_cmd.authen_url.c_str()), res_string.length() + 1, res_string.c_str());  //s_url          
            // mg_send(c, res_string.c_str(), res_string.length() + 1);     
            // mg_printf(c, "POST %s HTTP/1.0\r\n"
            //             "Host: %.*s\r\n"
            //             "User-Agent: Mozilla/5.0 (Windows; U; Windows NT 5.1; en-US; rv:1.7.6)\r\n"
            //             "Gecko/20050225 Firefox/1.0.1\r\n"
            //             "Content-Type: application/json\r\n"
            //             "Content-Length: %d\r\n"
            //             "Accept: application/json, text/plain, */*\r\n"
            //             "Connection: Keep-Alive\r\n"
            //             "\r\n"
            //             "%s\r\n",
            //             mg_url_uri(mg_authen_cmd.authen_url.c_str()), (int) host.len, host.ptr, res_string.length() + 1, res_string.c_str());  //s_url
        }else if (ev == MG_EV_HTTP_MSG) {
            // Response is received. Print it
            struct mg_http_message *hm = (struct mg_http_message *) ev_data;
            ROS_INFO("http_recv\n %.*s\n", (int) hm->message.len, hm->message.ptr);
#ifdef TEST_HTTP
            res_string = "{\"msg\":\"\",\"data\":{\"url\": null,\"clientid\": \"device_UAQ200000_1718364596429\",\
            \"username\": \"UAQ20000\",\"password\": \"CjZ8ZoaTFo6059vYzVVSXHaUOw94Mhbd\",\
            \"key\": \"MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEA20h9b0oxzjjqQfT3YMZD3EGSE+9X6Xtnn8wy77cQBPM5PnNIDBNidrUQRaNdpoY5YzMkvf98X3cHRIeD1f2qwJmgj่mEVq8LcPWAAWthr6Bx/L+nJNx/pylyBBRt/dY1c9469S67IxAotS55PNx60ypP65SVw0CBcds1zxGgvqiWs4ffQe2sbE1U1alSDciJzsjz3GhpKqCxAvf1FGPKxHcufIb091JN8mFRes3KR0cHt6xmxFxHGcajAL17udm72zVyC1Y1fAZgx7FDimX01MnFEIps+Av1SLkVB2/H8Ue5a6D5BcTpy8iruN+G2IQJ+d9NQR+D+A2uwg89G9TRXWIDAQAB\"},\"code\": 0}";
#endif      
            int i = 0; 
            while(i < hm->message.len) {
                if(*(hm->message.ptr + i) == '{' && *(hm->message.ptr + i + 1) == '\"') {
                    CLOUD_AUTHEN_EVENT::cloud_warehouse_decode(hm->message.ptr + i, mg_authen_state);
                    ROS_INFO_STREAM(" msg " << mg_authen_state.msg << " url " << mg_authen_state.url << " clientid " << mg_authen_state.clientid << " username " << mg_authen_state.username
                        << " password " << mg_authen_state.password << " key " << mg_authen_state.key << " code " << mg_authen_state.code);
                    if(mg_authen_state.url.length() > 0) {
                        RosserviceSend();
                    }
                    i = 0; 
                    break;
                }else if(hm->message.ptr[i] == '{' && hm->message.ptr[i + 1] == '\n') {
                    CLOUD_AUTHEN_EVENT::cloud_warehouse_decode(hm->message.ptr + i, mg_authen_state);
                    ROS_INFO_STREAM(" msg " << mg_authen_state.msg << " url " << mg_authen_state.url << " clientid " << mg_authen_state.clientid << " username " << mg_authen_state.username
                        << " password " << mg_authen_state.password << " key " << mg_authen_state.key << " code " << mg_authen_state.code);
                    if(mg_authen_state.url.length() > 0) {
                        RosserviceSend();
                    }
                    i = 0; 
                    break;
                }else if(hm->message.ptr[i] == '{' && hm->message.ptr[i + 1] == '\r' && hm->message.ptr[i + 1] == '\n') {
                    CLOUD_AUTHEN_EVENT::cloud_warehouse_decode(hm->message.ptr + i, mg_authen_state);
                    ROS_INFO_STREAM(" msg " << mg_authen_state.msg << " url " << mg_authen_state.url << " clientid " << mg_authen_state.clientid << " username " << mg_authen_state.username
                        << " password " << mg_authen_state.password << " key " << mg_authen_state.key << " code " << mg_authen_state.code);
                    if(mg_authen_state.url.length() > 0) {
                        RosserviceSend();
                    }
                    i = 0; 
                    break;
                }
                i++;
            }
            if(mg_authen_state.url.length() == 0) mg_authen_state.connect_state = mavros_msgs::UAT_MqttAuthenStatus::CONNECT_FAIL;
            c->is_closing = 1;         // Tell mongoose to close this connection
            *(bool *) fn_data = true;  // Tell event loop to stop
        }else if (ev == MG_EV_ERROR) {
            *(bool *) fn_data = true;  // Error, tell event loop to stop
            c->is_closing = 1;         // Tell mongoose to close this connection
            if(mg_authen_state.url.length() == 0) mg_authen_state.connect_state = mavros_msgs::UAT_MqttAuthenStatus::CONNECT_FAIL;
        }
    }
    void funcThread() {            
        ROS_INFO("CloudExchangeNx is running");
        int ret = -1;
        struct mg_mgr mgr;                        // Event manager
        bool done = true;                        // Event handler flips it to true
        mg_log_set("3");                          // Set to 0 to disable debug
        mg_authen_state.connect_state = mavros_msgs::UAT_MqttAuthenStatus::CONNECT_WAIT;
        mg_mgr_init(&mgr);                        // Initialise event manager
// #ifdef TEST_IDENTIFY
//         mg_authen_state.connect_state = mavros_msgs::UAT_MqttAuthenStatus::DISCONNECT;
//         mg_authen_cmd.cloud_en = true;
// 		mg_authen_cmd.need_identify = true;
// 	    mg_authen_cmd.authen_url = "http://airport.uatair.com:29999/driver/auth/authenticate";        //url
// 		mg_authen_cmd.username = "admin";          //username
// 		mg_authen_cmd.password = "I5rtj/cgBACo/Vf/lcJunA==";          //password
// 		mg_authen_cmd.type = "hangar";              //type 
// 		mg_authen_cmd.SnCode = "324148393720";            //sncode 
//         ROS_ERROR("1111111111111111111111");
// #endif        
        ros::Rate rate(10);  // ros::Rate loop_rate(500);  //10hz 100ms
        while(ros::ok()) {       
            if(mg_authen_cmd.SnCode.length() == 0) {
                ret = ROS_FILE_OPERATE::ReadVersionFile(filepath, mg_authen_cmd.SnCode);
                ret = ROS_FILE_OPERATE::ReadVersionFile(snfilepath_backup, mg_authen_cmd.SnCode);  //flash new sn_code
                ret = ROS_FILE_OPERATE::ReadVersionFile(snfilepath, mg_authen_cmd.SnCode);  //flash new sn_code
                if(mg_authen_cmd.SnCode.length() > 12)  {
                    mg_authen_cmd.SnCode = mg_authen_cmd.SnCode.substr(mg_authen_cmd.SnCode.length() - 12, 12);
                    ROS_INFO_STREAM("sn_code " << mg_authen_cmd.SnCode);  
                }else if(mg_authen_cmd.SnCode.length() > 0 && mg_authen_cmd.SnCode.length() <= 12)  {
                    ROS_INFO_STREAM("sn_code " << mg_authen_cmd.SnCode);
                }
            }  
            if(mg_authen_state.connect_state == mavros_msgs::UAT_MqttAuthenStatus::DISCONNECT || mg_authen_state.connect_state == mavros_msgs::UAT_MqttAuthenStatus::CONNECT_FAIL) {
                if(mg_authen_cmd.cloud_en == true && mg_authen_cmd.need_identify == true) {
                    mg_mgr_free(&mgr);                                   // Free resources
                    mg_mgr_init(&mgr);                        // Initialise event manager
                    done = false; 
                    mg_http_connect(&mgr, mg_authen_cmd.authen_url.c_str(), fn, &done);  // Create client connection  s_url
                    mg_authen_state.connect_state = mavros_msgs::UAT_MqttAuthenStatus::CONNECT_WAIT;
                }
            }else if(mg_authen_state.connect_state == mavros_msgs::UAT_MqttAuthenStatus::DISCONNECT && mg_authen_state.url.length() > 0) {
                RosserviceSend();
            }
            if(!done) mg_mgr_poll(&mgr, 1000);    // Infinite event loop             
            rate.sleep();
            //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        mg_mgr_free(&mgr);                                   // Free resources
    }
    void RosserviceSend() {  //rosservice client
        mavros_msgs::UAT_AuthenCtl msg;
        msg.request.source = msg.request.CLOUD_SERVER;
        msg.request.url = mg_authen_state.url;
        msg.request.clientid = mg_authen_state.clientid;
        msg.request.username = mg_authen_state.username;
        msg.request.password = mg_authen_state.password;
        msg.request.key = mg_authen_state.key;

        char host_ip[16] = {0};
        mg_get_host_ip(host_ip);
        msg.request.host_ip = host_ip;
        std::string host_port = mg_authen_state.url.substr(mg_authen_state.url.find_last_of(".com:") + 1, mg_authen_state.url.length() - mg_authen_state.url.find_last_of(".com:"));  
        msg.request.host_port = atoi(host_port.c_str());
        std::cout << " host_ip " << msg.request.host_ip << " port " << msg.request.host_port << std::endl;
        if (HangarCtlClient.call(msg)) {
            mg_authen_state.connect_state = mavros_msgs::UAT_MqttAuthenStatus::CONNECT_OK;
        }else {
            //ROS_ERROR("Failed to call service");
            mg_authen_state.connect_state = mavros_msgs::UAT_MqttAuthenStatus::DISCONNECT;
        }
    }
    void RosTopicRecieve(const mavros_msgs::UAT_MqttAuthenStatus::ConstPtr &msg) {
        mg_authen_state.connect_state = msg->MqttConnectState;
        mg_authen_state.device_type = msg->device_type;
        // ROS_INFO("RosTopicRecieve MqttConnectState %d device_type %d", msg->MqttConnectState, msg->device_type);
    }
    std::string getPlatformName()
    {
        std::ifstream file("/proc/device-tree/model");
        if (file.is_open())
        {
            std::string modelName;
            std::getline(file, modelName);
            if (modelName.find("Xavier") != std::string::npos)
            {
                return "Xavier";
            }
            else if(modelName.find("Rockchip") != std::string::npos)
            {
                return "Rockchip";
            }
        }
        return "unknown";
    }       
    void GetNode(ros::NodeHandle &nh) {  
        HangarCtlClient = nh.serviceClient<mavros_msgs::UAT_AuthenCtl>("/uat/UAT_AuthenCtl");
        uat_hangerstatus_sub = nh.subscribe<mavros_msgs::UAT_MqttAuthenStatus>("uat/UAT_MqttAuthenStatus", 10, RosTopicRecieve);
        nh.param <std::string>("type", mg_authen_cmd.type, "device");
        nh.param <bool>("/cloud_en", mg_authen_cmd.cloud_en, false);
        nh.param <bool>("/cloud_identify/need_identify", mg_authen_cmd.need_identify, false);
        nh.param <std::string>("/cloud_identify/authen_url", mg_authen_cmd.authen_url, "");
        nh.param <std::string>("/cloud_identify/username", mg_authen_cmd.username, "");
        nh.param <std::string>("/cloud_identify/password", mg_authen_cmd.password, "");
        ROS_INFO_STREAM(" mg_authen_cmd.type " << mg_authen_cmd.type);
        ROS_INFO_STREAM("mg_authen_cmd.cloud_en " << mg_authen_cmd.cloud_en << "mg_authen_cmd.need_identify " << mg_authen_cmd.need_identify);
        ROS_INFO_STREAM("mg_authen_cmd.authen_url " << mg_authen_cmd.authen_url);
        ROS_INFO_STREAM("mg_authen_cmd.username " << mg_authen_cmd.username);
        ROS_INFO_STREAM("mg_authen_cmd.password " << mg_authen_cmd.password);
        std::string platform = getPlatformName();
        if(platform == "Xavier")  {
            ROS_INFO("mg_authen_cmd.platform Xavier");
            setPlatformName(XAVIER_PLATFORM);
        }else if(platform == "Rockchip") {
            ROS_INFO("mg_authen_cmd.platform Rockchip");
            setPlatformName(ROCKCHIP_PLATFORM);
        }else {
            ROS_INFO("mg_authen_cmd.platform other");
            setPlatformName(OTHER_PLATFORM);
        }
    }  
}