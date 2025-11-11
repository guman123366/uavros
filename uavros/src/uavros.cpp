#include <ros/console.h>
#include <fnmatch.h>
#include <mavconn/udp.h>
#include <std_msgs/String.h>
#include <uavros.h>
#include <chrono>

using namespace uavros;
using mavconn::MAVConnInterface;

uavros::UavRos::UavRos()
: uav_nh("~")
{
    // 读取参数
    std::string fcu_url, gcs_url;
    std::string fcu_protocol;
    int system_id, component_id;
    int tgt_system_id, tgt_component_id;
    bool px4_usb_quirk;
    double conn_timeout_d;
    ros::V_string plugin_blacklist{}, plugin_whitelist{};

    uav_nh.param<std::string>("fcu_url", fcu_url, "serial:///dev/ttyACM0");
    uav_nh.param<std::string>("gcs_url", gcs_url, "udp://@");
    uav_nh.param("conn/timeout", conn_timeout_d, 30.0);

    uav_nh.param<std::string>("fcu_protocol", fcu_protocol, "v2.0");
    uav_nh.param("system_id", system_id, 1);
    uav_nh.param<int>("component_id", component_id, mavconn::MAV_COMP_ID_UDP_BRIDGE);
    uav_nh.param("target_system_id", tgt_system_id, 1);
    uav_nh.param("target_component_id", tgt_component_id, 1);
    uav_nh.param("startup_px4_usb_quirk", px4_usb_quirk, false);
    uav_nh.getParam("plugin_blacklist", plugin_blacklist);
    uav_nh.getParam("plugin_whitelist", plugin_whitelist);

    // 飞参存储通信
    std::string flycontrol_url;
    if (!uav_nh.getParam("/flycontrol_param/flycontrol_connect", flycontrol_url)) {
        ROS_WARN("Param /flycontrol_param/flycontrol_connect does not exist, using default: /dev/ttyS0:921600");
        flycontrol_url = "/dev/ttyS0:921600";
    }

    ROS_INFO_STREAM("flycontrol_connect param is " << flycontrol_url);
    ROS_INFO_STREAM("fcu_url param is " << fcu_url);

    // 初始化发布器和订阅器
    uav_pub = uav_nh.advertise<std_msgs::UInt8MultiArray>("/telemeter_data", 1000);
    uav_fc_sub = uav_nh.subscribe<std_msgs::UInt8MultiArray>("/remote_control_data", 1000, &UavRos::uav_fc_sub_cb, this);
    fc_param_pub = uav_nh.advertise<std_msgs::UInt8MultiArray>("/fly_control_data", 1000);
    main_mems_pub = uav_nh.advertise<std_msgs::UInt8MultiArray>("/main_mems_data", 1000);
    backup_mems_pub = uav_nh.advertise<std_msgs::UInt8MultiArray>("/backup_mems_data", 1000);
    sense_pub = uav_nh.advertise<std_msgs::UInt8MultiArray>("/sense_data", 1000);

    // 创建FCU连接
    if (!createConnection(fcu_url, system_id, component_id, fcu_link, ConnectionType::FCU)) {
        ros::shutdown();
        return;
    }

    // 创建飞参连接
    if (!createConnection(flycontrol_url, system_id, component_id, fc_param_link, ConnectionType::FC_PARAM)) {
        ros::shutdown();
        return;
    }

    // 创建MAIN_MEMS连接
    std::string main_mems_url;
    if (!uav_nh.getParam("/main_mems_param/mems_connect", main_mems_url)) {
        ROS_WARN("Param /main_mems_param/mems_connect does not exist, using default: udp://@");
        main_mems_url = "udp://@";
    }
    ROS_INFO_STREAM(" main_mems param is " << main_mems_url);
    if (!createConnection(main_mems_url, system_id, component_id, main_mems_link, ConnectionType::MAIN_MEMS)) {
        ros::shutdown();
        return;
    }

    // 创建BACKUP_MEMS连接
    std::string backup_mems_url;
    if (!uav_nh.getParam("/backup_mems_param/mems_connect", backup_mems_url)) {
        ROS_WARN("Param /backup_mems_param/mems_connect does not exist, using default: udp://@");
        backup_mems_url = "udp://@";
    }
    ROS_INFO_STREAM(" backup_mems param is " << backup_mems_url);
    if (!createConnection(backup_mems_url, system_id, component_id, backup_mems_link, ConnectionType::BACKUP_MEMS)) {
        ros::shutdown();
        return;
    } 

    // 创建sense连接
    std::string sense_url;
    if (!uav_nh.getParam("/sense_param/sense_connect", sense_url)) {
        ROS_WARN("Param /sense_param/sense_connect does not exist, using default: udp://@");
        sense_url = "udp://@";
    }
    ROS_INFO_STREAM("sense param is " << sense_url);
    if (!createConnection(sense_url, system_id, component_id, sense_link, ConnectionType::SENSE)) {
        ros::shutdown();
        return;
    }

    // 初始化数据记录文件
    _saveFile.open("/home/uatair/Vehicle_Firmware/FTI_Data.dat", std::ios::binary);
    if (_saveFile.is_open()) {
        ROS_INFO("FTI_Data.dat opened successfully!");
    } else {
        ROS_ERROR("Failed to open FTI_Data.dat!");
    }

    // 初始化成员变量
    serial_rev_counts = 0;
    data_nums = 0;
    vecBufData.reserve(1024); // 预分配内存
    vecFcParamData.reserve(1024);
    vecMainMemsData.reserve(1024);
    vecBackupMemsData.reserve(1024);
    vecSenseData.reserve(1024);
}

uavros::UavRos::~UavRos()
{
    if (_saveFile.is_open()) {
        _saveFile.close();
    }
}

bool UavRos::createConnection(const std::string& url, int& system_id, int& component_id, 
                              mavconn::MAVConnInterface::Ptr& conn, ConnectionType conn_type)
{
    try {
        conn = MAVConnInterface::open_url_no_connect(url, system_id, component_id);
        system_id = conn->get_system_id();
        component_id = conn->get_component_id();

        // 根据连接类型设置不同的回调函数
        switch (conn_type) {
            case ConnectionType::FCU:
                conn->connectHex(
                    [this](const uint8_t* buf, const size_t bufsize) {
                        uav_pub_cb(buf, bufsize);
                    },
                    []() {
                        ROS_ERROR("FCU connection closed, mavros will be terminated.");
                        ros::requestShutdown();
                    });
                break;
                
            case ConnectionType::FC_PARAM:
                conn->connectHex(
                    [this](const uint8_t* buf, const size_t bufsize) {
                        fc_param_cb(buf, bufsize);
                    },
                    []() {
                        ROS_ERROR("FC param connection closed, mavros will be terminated.");
                        ros::requestShutdown();
                    });
                break;
                
            case ConnectionType::MAIN_MEMS:
                conn->connectHex(
                    [this](const uint8_t* buf, const size_t bufsize) {
                        main_mems_pub_cb(buf, bufsize);
                    },
                    []() {
                        ROS_ERROR("MAIN_MEMS connection closed, mavros will be terminated.");
                        ros::requestShutdown();
                    });
                break;

            case ConnectionType::BACKUP_MEMS:
                conn->connectHex(
                    [this](const uint8_t* buf, const size_t bufsize) {
                        backup_mems_pub_cb(buf, bufsize);
                    },
                    []() {
                        ROS_ERROR("BACKUP_MEMS connection closed, mavros will be terminated.");
                        ros::requestShutdown();
                    });
                break;

            case ConnectionType::SENSE:
                conn->connectHex(
                    [this](const uint8_t* buf, const size_t bufsize) {
                        sense_pub_cb(buf, bufsize);
                    },
                    []() {
                        ROS_ERROR("SENSE connection closed, mavros will be terminated.");
                        ros::requestShutdown();
                    });
                break;
        }

        ROS_INFO_STREAM("Connection created successfully for type: " << static_cast<int>(conn_type));
        return true;
    } catch (const mavconn::DeviceError& ex) {
        ROS_FATAL_STREAM("Connection failed for type " << static_cast<int>(conn_type) << ": " << ex.what());
        return false;
    }
}

void UavRos::spin()
{
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    ROS_INFO("Stopping mavros...");
    spinner.stop();
}

void uavros::UavRos::uav_pub_cb(const uint8_t* buf, const size_t bufsize)
{
    static auto last_time = std::chrono::system_clock::now();
    
    // 追加数据到缓冲区
    vecBufData.insert(vecBufData.end(), buf, buf + bufsize);

    // 发布数据
    std_msgs::UInt8MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].label = "length";
    msg.layout.dim[0].size = vecBufData.size();
    msg.layout.dim[0].stride = 1;
    msg.data.swap(vecBufData); // 使用swap避免拷贝

    uav_pub.publish(msg);

    // 记录数据到文件
    if (_saveFile.is_open()) {
        _saveFile.write(reinterpret_cast<const char*>(buf), bufsize);
    }

    // 输出调试信息
    data_nums = msg.data.size();
    ROS_DEBUG_STREAM("Serial rev datasize is " << data_nums);
    uint8_to_hex_string(msg.data.data(), std::min(msg.data.size(), size_t(10))); // 只打印前10个字节

    // 计算时间间隔
    auto now = std::chrono::system_clock::now();
    auto time_span = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time);
    ROS_DEBUG_STREAM("Data interval: " << time_span.count() << " ms");
    last_time = now;

    // 清空缓冲区
    vecBufData.clear();
}

void uavros::UavRos::uav_fc_sub_cb(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    fcu_link->send_bytes(msg->data.data(), msg->data.size());
}

void uavros::UavRos::fc_param_cb(const uint8_t* buf, const size_t bufsize)
{
    vecFcParamData.insert(vecFcParamData.end(), buf, buf + bufsize);
    
    std_msgs::UInt8MultiArray fc_param_msg;
    fc_param_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    fc_param_msg.layout.dim[0].label = "length";
    fc_param_msg.layout.dim[0].size = vecFcParamData.size();
    fc_param_msg.layout.dim[0].stride = 1;
    fc_param_msg.data.swap(vecFcParamData); // 使用swap避免拷贝

    fc_param_pub.publish(fc_param_msg);
    
    ROS_DEBUG("Received fc_param_data successfully!");
}

void uavros::UavRos::sense_pub_cb(const uint8_t* buf, const size_t bufsize)
{
    vecSenseData.insert(vecSenseData.end(), buf, buf + bufsize);
    
    std_msgs::UInt8MultiArray sense_msg;
    sense_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    sense_msg.layout.dim[0].label = "length";
    sense_msg.layout.dim[0].size = vecSenseData.size();
    sense_msg.layout.dim[0].stride = 1;
    sense_msg.data.swap(vecSenseData); // 使用swap避免拷贝

    sense_pub.publish(sense_msg);
    
    ROS_DEBUG("Received sense_data successfully!");
}

void uavros::UavRos::main_mems_pub_cb(const uint8_t* buf, const size_t bufsize)
{
    vecMainMemsData.insert(vecMainMemsData.end(), buf, buf + bufsize);
    
    std_msgs::UInt8MultiArray main_mems_msg;
    main_mems_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    main_mems_msg.layout.dim[0].label = "length";
    main_mems_msg.layout.dim[0].size = vecMainMemsData.size();
    main_mems_msg.layout.dim[0].stride = 1;
    main_mems_msg.data.swap(vecMainMemsData); // 使用swap避免拷贝

    main_mems_pub.publish(main_mems_msg);
    
    uint8_to_hex_string(main_mems_msg.data.data(), std::min(main_mems_msg.data.size(), size_t(10))); // 只打印前10个字节
    ROS_DEBUG("Received main_mems_data successfully!");
}

void uavros::UavRos::backup_mems_pub_cb(const uint8_t* buf, const size_t bufsize)
{
    vecBackupMemsData.insert(vecBackupMemsData.end(), buf, buf + bufsize);
    
    std_msgs::UInt8MultiArray backup_mems_msg;
    backup_mems_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    backup_mems_msg.layout.dim[0].label = "length";
    backup_mems_msg.layout.dim[0].size = vecBackupMemsData.size();
    backup_mems_msg.layout.dim[0].stride = 1;
    backup_mems_msg.data.swap(vecBackupMemsData); // 使用swap避免拷贝

    backup_mems_pub.publish(backup_mems_msg);
    
    uint8_to_hex_string(backup_mems_msg.data.data(), std::min(backup_mems_msg.data.size(), size_t(10))); // 只打印前10个字节
    ROS_DEBUG("Received backup_mems_data successfully!");
}

void uavros::UavRos::uint8_to_hex_string(const uint8_t *data, size_t length)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < length; i++) {
        ss << std::setw(2) << static_cast<int>(data[i]) << " ";
    }
    //ROS_DEBUG_STREAM("Received data: " << ss.str());
	std::cout<<"Receive serial Data is "<<ss.str()<<std::endl;
}
