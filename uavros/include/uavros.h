#pragma once

#include <array>
#include <vector>
#include <ros/ros.h>
#include <mavconn/interface.h>
#include <std_msgs/UInt8MultiArray.h>
#include <fstream>
#include <memory>
#include <iostream>
//#include <saveDataToFile.h>

namespace uavros{

//class SaveDataToFile;

class UavRos
{
public:
    UavRos();
    ~UavRos();

    void spin();

private:
    // 连接类型枚举
    enum class ConnectionType {
        FCU,
        FC_PARAM,
        MAIN_MEMS,
        BACKUP_MEMS,
        SENSE
    };

    // 连接创建辅助函数
    bool createConnection(const std::string& url, int& system_id, int& component_id, 
                         mavconn::MAVConnInterface::Ptr& conn, ConnectionType conn_type);
    
    // 回调函数
    void uav_pub_cb(const uint8_t* buf, size_t bufsize);
    void uav_fc_sub_cb(const std_msgs::UInt8MultiArray::ConstPtr& msg);
    void fc_param_cb(const uint8_t* buf, size_t bufsize);
    void main_mems_pub_cb(const uint8_t* buf, size_t bufsize);
    void backup_mems_pub_cb(const uint8_t* buf, size_t bufsize);
    void sense_pub_cb(const uint8_t* buf, size_t bufsize);
    
    // 工具函数
    void uint8_to_hex_string(const uint8_t* data, size_t length);
private:
    ros::NodeHandle uav_nh;
    ros::Duration conn_timeout;
    mavconn::MAVConnInterface::Ptr fcu_link;

    mavconn::MAVConnInterface::Ptr fc_param_link;

    //MEMS传感器数据
    mavconn::MAVConnInterface::Ptr main_mems_link;
    mavconn::MAVConnInterface::Ptr backup_mems_link;
    mavconn::MAVConnInterface::Ptr sense_link;

    ros::Publisher uav_pub;
    ros::Subscriber uav_fc_sub;

    //! fcu link -> ros
	//void uav_pub_cb(const uint8_t *buf,const size_t bufsize);
	//! ros -> fcu link
    //void uav_fc_sub_cb(std_msgs::UInt8MultiArray);

    //飞参数据
    //void fc_param_cb(const uint8_t *buf,const size_t bufsize);
    ros::Publisher fc_param_pub;

    //MEMS处理和发布
    //void mems_pub_cb(const uint8_t *buf,const size_t bufsize);
    ros::Publisher main_mems_pub;
    ros::Publisher backup_mems_pub;
    ros::Publisher sense_pub;
    //void createMemsConnect(std::string url,int systemid,int componentid);
    
    //void uint8_to_hex_string(const uint8_t* data, size_t length);

    //std::shared_ptr<SaveDataToFile> _saveFile;
    std::ofstream _saveFile;

    long serial_rev_counts;

    long data_nums;
    std::vector<uint8_t> vecBufData;
    std::vector<uint8_t> vecFcParamData;
    std::vector<uint8_t> vecMainMemsData;
    std::vector<uint8_t> vecBackupMemsData;
    std::vector<uint8_t> vecSenseData;
};
}