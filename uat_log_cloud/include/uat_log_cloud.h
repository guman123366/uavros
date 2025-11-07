#pragma once

#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <condition_variable>
#include <stack>

class CurlUploader;

struct logfileinfo
{
    int data_chunk_size=0;                      //每个文件大小
    std::string data_directory="";               //文件存储路径
    std::vector<uint8_t> current_chunk;         //文件数据
    std::mutex queue_mutex;                     //文件存储队列互斥锁
    std::stack<std::string> filename_queue;    //文件名称队列
    std::string data_type;                      //数据类型标识
    std::atomic<bool> upload_enabled{false};      // 该数据类型上传使能
    std::thread upload_thread;                    // 专属上传线程
    std::condition_variable queue_cv;             // 专属条件变量
    std::atomic<uint64_t> files_uploaded{0};      // 已上传文件计数
    std::atomic<uint64_t> upload_errors{0};       // 上传错误计数
};

class uat_log_cloud
{
public:
    uat_log_cloud();
    ~uat_log_cloud();

private:
    
    void upfile_cloud_cb(const ros::TimerEvent &event);
    void fc_param_data_sub_cb(std_msgs::UInt8MultiArray msg);
    void mems_data_sub_cb(std_msgs::UInt8MultiArray msg);
    void sense_data_sub_cb(std_msgs::UInt8MultiArray msg);

    void processDataChunk(const std::vector<uint8_t>& new_data, 
                                   logfileinfo& fileinfo);

    //读取日志文件参数
    void readFileInfo();

    // 文件存储相关方法
    bool createDataDirectory(const std::string& filePath);
    std::string generateFilename(const std::string& data_type);
    void writeToFile(logfileinfo& fileinfo, const std::string& filename);

    void startThread();
    void stopThread();

    void uploadThread(logfileinfo& fileinfo);

    //获取指定路径下的所有文件
    void listFiles(logfileinfo& fileinfo);
    bool validateFile(const std::string& filepath);
    bool uploadFile(const std::string& filepath, const std::string& data_type);

    //计算MD5
    std::string calculateFileMD5(const std::string& filename);
private:
    ros::NodeHandle uat_log_nh;

    //std::shared_ptr<CurlUploader> _curlUploader;

    ros::Timer _up_file_timer;

    ros::Subscriber fc_param_sub;
    ros::Subscriber mems_sub;
    ros::Subscriber sense_sub;

    // 全局运行状态
    std::atomic<bool> running_{false};
    
    // 三种数据类型的文件信息
    logfileinfo fc_data_file_info;
    logfileinfo mems_data_file_info;
    logfileinfo sense_data_file_info;
    
    // 全局统计信息
    std::atomic<uint64_t> bytes_received_{0};

    std::string strUrl;
    
    // 性能监控
    std::map<std::string, ros::Time> last_stat_time_;
    std::map<std::string, size_t> data_received_stats_;
};