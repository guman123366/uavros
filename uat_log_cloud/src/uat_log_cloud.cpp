#include "uat_log_cloud.h"
#include "CurlUploader.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <sys/stat.h>
#include <dirent.h>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <atomic>

uat_log_cloud::uat_log_cloud()
:uat_log_nh("~"),running_(false),bytes_received_(0)
{
    //_curlUploader=std::make_shared<CurlUploader>();

    fc_param_sub = uat_log_nh.subscribe<std_msgs::UInt8MultiArray>("/fly_control_data", 1000,&uat_log_cloud::fc_param_data_sub_cb,this);
    mems_sub = uat_log_nh.subscribe<std_msgs::UInt8MultiArray>("/mems_data", 1000,&uat_log_cloud::mems_data_sub_cb,this);
    sense_sub = uat_log_nh.subscribe<std_msgs::UInt8MultiArray>("/sense_data", 1000,&uat_log_cloud::sense_data_sub_cb,this);

    _up_file_timer = uat_log_nh.createTimer(ros::Duration(1), &uat_log_cloud::upfile_cloud_cb, this, false);

    readFileInfo();

    //读取目录下的初始文件
    listFiles(fc_data_file_info);
    listFiles(mems_data_file_info);
    listFiles(sense_data_file_info);

    startThread();

    ROS_INFO("uat_log_cloud initialized successfully");
}

uat_log_cloud::~uat_log_cloud()
{
    stopThread();
}

void uat_log_cloud::upfile_cloud_cb(const ros::TimerEvent &event)
{
    

}

void uat_log_cloud::fc_param_data_sub_cb(std_msgs::UInt8MultiArray msg)
{
    processDataChunk(msg.data, fc_data_file_info);
}

void uat_log_cloud::mems_data_sub_cb(std_msgs::UInt8MultiArray msg)
{
    processDataChunk(msg.data, mems_data_file_info);
    ROS_INFO("uat_log_cloud receive mems data successfully");
}

void uat_log_cloud::sense_data_sub_cb(std_msgs::UInt8MultiArray msg)
{
    processDataChunk(msg.data, sense_data_file_info);
}

void uat_log_cloud::processDataChunk(const std::vector<uint8_t>& new_data, logfileinfo& fileinfo)
{
    if (new_data.empty()) {
            return;
        }

    // 统计数据接收速率
    auto now = ros::Time::now();
    if (last_stat_time_.find(fileinfo.data_type) == last_stat_time_.end()) {
        last_stat_time_[fileinfo.data_type] = now;
        data_received_stats_[fileinfo.data_type] = 0;
    }
    
    data_received_stats_[fileinfo.data_type] += new_data.size();
    bytes_received_ += new_data.size();
    
    auto duration = (now - last_stat_time_[fileinfo.data_type]).toSec();
    if (duration > 10.0) { // 每10秒统计一次
        double rate = data_received_stats_[fileinfo.data_type] / duration;
        ROS_DEBUG_THROTTLE(10, "%s data rate: %.2f KB/s", fileinfo.data_type.c_str(), rate / 1024.0);
        data_received_stats_[fileinfo.data_type] = 0;
        last_stat_time_[fileinfo.data_type] = now;
    }
    
    // 添加数据到当前块
    fileinfo.current_chunk.insert(fileinfo.current_chunk.end(), new_data.begin(), new_data.end());

    ROS_INFO("fileinfo.current_chunk.size is %d",fileinfo.current_chunk.size());
    
    // 如果达到块大小，处理并重置
    if (fileinfo.current_chunk.size() >= fileinfo.data_chunk_size) {
        std::string filename = generateFilename(fileinfo.data_type);
        ROS_INFO("generateFilename is %s",filename.c_str());
        writeToFile(fileinfo, filename);
        fileinfo.current_chunk.clear();
        fileinfo.current_chunk.reserve(fileinfo.data_chunk_size); // 重新预留空间
    }
}

void uat_log_cloud::readFileInfo()
{
    if(uat_log_nh.getParam("/flycontrol_param/data_chunk_size",fc_data_file_info.data_chunk_size))
    {
        ROS_INFO("data_chunk_size is %d",fc_data_file_info.data_chunk_size);
    }
    else
    {
        fc_data_file_info.data_chunk_size=1024*1024;
        ROS_INFO("data_chunk_size read failed, using default: %d", fc_data_file_info.data_chunk_size);
    }
    fc_data_file_info.current_chunk.reserve(fc_data_file_info.data_chunk_size);
    fc_data_file_info.data_type="fc";
    fc_data_file_info.upload_enabled=true;
    
    if(uat_log_nh.getParam("/flycontrol_param/data_save_path",fc_data_file_info.data_directory))
    {
        ROS_INFO("data_save_path is %s",fc_data_file_info.data_directory.c_str());
    }
    else{
        fc_data_file_info.data_directory="/tmp/uat_fc";
        ROS_INFO("data_save_path read failed, using default: %s", fc_data_file_info.data_directory.c_str());
    }
    createDataDirectory(fc_data_file_info.data_directory);

    if(uat_log_nh.getParam("/flycontrol_param/url",strUrl))
    {
        ROS_INFO("url is %s",strUrl);
    }
    else{

        ROS_INFO("url read failed!");
    }

    if(uat_log_nh.getParam("/mems_param/data_chunk_size",mems_data_file_info.data_chunk_size))
    {
        ROS_INFO("data_chunk_size is %d",mems_data_file_info.data_chunk_size);
    }
    else
    {
        mems_data_file_info.data_chunk_size=1024*1024;
        ROS_INFO("data_chunk_size read failed, using default: %d", mems_data_file_info.data_chunk_size);
    }
    mems_data_file_info.current_chunk.reserve(mems_data_file_info.data_chunk_size);
    mems_data_file_info.data_type="mems";
    mems_data_file_info.upload_enabled=true;
    
    if(uat_log_nh.getParam("/mems_param/data_save_path",mems_data_file_info.data_directory))
    {
        ROS_INFO("data_save_path is %s",mems_data_file_info.data_directory.c_str());
    }
    else{
        mems_data_file_info.data_directory="/tmp/uat_logs";
        ROS_INFO("data_save_path read failed, using default: %s", mems_data_file_info.data_directory.c_str());
    }
    createDataDirectory(mems_data_file_info.data_directory);

    if(uat_log_nh.getParam("/sense_param/data_chunk_size",sense_data_file_info.data_chunk_size))
    {
        ROS_INFO("data_chunk_size is %d",sense_data_file_info.data_chunk_size);
    }
    else
    {
        sense_data_file_info.data_chunk_size=1024*1024;
        ROS_INFO("data_chunk_size read failed, using default: %d", sense_data_file_info.data_chunk_size);
    }
    sense_data_file_info.current_chunk.reserve(sense_data_file_info.data_chunk_size);
    sense_data_file_info.data_type="sense";
    sense_data_file_info.upload_enabled=true;
    
    if(uat_log_nh.getParam("/sense_param/data_save_path",sense_data_file_info.data_directory))
    {
        ROS_INFO("data_save_path is %s",sense_data_file_info.data_directory.c_str());
    }
    else{
        sense_data_file_info.data_directory="/tmp/uat_logs";
        ROS_INFO("data_save_path read failed, using default: %s", sense_data_file_info.data_directory.c_str());
    }
    createDataDirectory(sense_data_file_info.data_directory);
}

bool uat_log_cloud::createDataDirectory(const std::string& filePath)
{
    struct stat info;
    if (stat(filePath.c_str(), &info) != 0) {
        // 目录不存在，创建它
        if (mkdir(filePath.c_str(), 0755) != 0) {
            ROS_ERROR("Failed to create directory: %s", filePath.c_str());
            return false;
        }
        ROS_INFO("Created data directory: %s", filePath.c_str());
    } else if (!(info.st_mode & S_IFDIR)) {
        ROS_ERROR("Path exists but is not a directory: %s", filePath.c_str());
        return false;
    }
    return true;
}

void uat_log_cloud::writeToFile(logfileinfo& fileinfo, const std::string &filename)
{
    if (fileinfo.current_chunk.empty()) {
        ROS_WARN("Attempted to write empty data to file: %s", filename.c_str());
        return;
    }
    
    std::string filepath = fileinfo.data_directory + "/" + filename;

    ROS_INFO("filepath is %s",filepath.c_str());
    
    // 使用更高效的文件写入方式
    std::ofstream file(filepath, std::ios::binary | std::ios::out);
    if (!file) {
        ROS_ERROR("Failed to open file for writing: %s", filepath.c_str());
        return;
    }
    
    // 一次性写入所有数据
    if (!file.write(reinterpret_cast<const char*>(fileinfo.current_chunk.data()), fileinfo.current_chunk.size())) {
        ROS_ERROR("Failed to write data to file: %s", filepath.c_str());
        file.close();
        return;
    }
    
    file.close();
    
    // 添加到上传队列
    {
        std::lock_guard<std::mutex> lock(fileinfo.queue_mutex);
        fileinfo.filename_queue.push(filepath);
        
        // 限制队列大小，防止内存爆炸
        if (fileinfo.filename_queue.size() > 1000) {
            ROS_WARN_THROTTLE(60, "Upload queue size exceeded limit: %zu", fileinfo.filename_queue.size());
            // 可以选择移除最老的文件或者暂停接收新数据
            if (fileinfo.filename_queue.size() > 2000) {
                std::string old_file = fileinfo.filename_queue.top();
                fileinfo.filename_queue.pop();
                if (remove(old_file.c_str()) == 0) {
                    ROS_WARN("Removed old file due to queue overflow: %s", old_file.c_str());
                }
            }
        }
    }
    
    fileinfo.queue_cv.notify_one();
    ROS_INFO("Upload queue size: %d", fileinfo.filename_queue.size());
}

void uat_log_cloud::stopThread()
{
    running_ = false;
    
    // 停止所有数据类型的上传
    fc_data_file_info.upload_enabled = false;
    mems_data_file_info.upload_enabled = false;
    sense_data_file_info.upload_enabled = false;
    
    // 通知所有等待的线程
    fc_data_file_info.queue_cv.notify_all();
    mems_data_file_info.queue_cv.notify_all();
    sense_data_file_info.queue_cv.notify_all();
    
    // 等待所有线程结束
    if (fc_data_file_info.upload_thread.joinable()) {
        fc_data_file_info.upload_thread.join();
    }
    if (mems_data_file_info.upload_thread.joinable()) {
        mems_data_file_info.upload_thread.join();
    }
    if (sense_data_file_info.upload_thread.joinable()) {
        sense_data_file_info.upload_thread.join();
    }
    
    ROS_INFO("All upload threads stopped");
}

void uat_log_cloud::startThread()
{
    if (running_) {
        ROS_WARN("DataUploader already running");
        return;
    }
    
    running_ = true;
    
    // 启动上传线程
    // 启动三个独立的上传线程
    fc_data_file_info.upload_thread = std::thread(&uat_log_cloud::uploadThread, this, std::ref(fc_data_file_info));
    mems_data_file_info.upload_thread = std::thread(&uat_log_cloud::uploadThread, this, std::ref(mems_data_file_info));
    sense_data_file_info.upload_thread = std::thread(&uat_log_cloud::uploadThread, this, std::ref(sense_data_file_info));
    
    ROS_INFO("Started 3 upload threads for FC, MEMS, and SENSE data");
}

void uat_log_cloud::uploadThread(logfileinfo& fileinfo) {
    int consecutive_failures = 0;
    const int max_consecutive_failures = 10;
    const std::string data_type = fileinfo.data_type;
    
    ROS_INFO("%s upload thread started", data_type.c_str());
    
    while (running_ && fileinfo.upload_enabled) {
        std::string filepath;

        {
            std::unique_lock<std::mutex> lock(fileinfo.queue_mutex);
            
            if (fileinfo.filename_queue.empty()) {
                // 使用较短超时时间，及时响应停止信号
                fileinfo.queue_cv.wait_for(lock, std::chrono::seconds(1));
                if (fileinfo.filename_queue.empty()) {
                    //检查文件夹是否有残存文件
                    listFiles(fileinfo);
                    continue;
                }
            }
            
            filepath = fileinfo.filename_queue.top();
        }

        // 验证文件
        if (!validateFile(filepath)) {
            ROS_ERROR("[%s] Invalid file, removing from queue: %s", data_type.c_str(), filepath.c_str());
            {
                std::lock_guard<std::mutex> lock(fileinfo.queue_mutex);
                if (!fileinfo.filename_queue.empty()) {
                    fileinfo.filename_queue.pop();
                }
            }
            fileinfo.upload_errors++;
            continue;
        }

        // 上传文件
        if (uploadFile(filepath, data_type)) {
            consecutive_failures = 0;
            fileinfo.files_uploaded++;
            
            {
                std::lock_guard<std::mutex> lock(fileinfo.queue_mutex);
                if (!fileinfo.filename_queue.empty()) {
                    fileinfo.filename_queue.pop();
                }
            }

            
            
            // 控制上传速率，不同类型可以有不同的速率
            if (data_type == "fc") {
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); // FC数据优先级较高
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 其他数据
            }
            
        } else {
            consecutive_failures++;
            fileinfo.upload_errors++;
            ROS_ERROR("[%s] Upload failed for file: %s (failures: %d)", 
                     data_type.c_str(), filepath.c_str(), consecutive_failures);
            
            if (consecutive_failures >= max_consecutive_failures) {
                ROS_ERROR("[%s] Too many upload failures, pausing for 10 seconds", data_type.c_str());
                std::this_thread::sleep_for(std::chrono::seconds(10));
                consecutive_failures = 0;
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
    
    ROS_INFO("%s upload thread stopped", data_type.c_str());
}

bool uat_log_cloud::validateFile(const std::string& filepath) {
    struct stat file_stat;
    if (stat(filepath.c_str(), &file_stat) != 0) {
        return false;
    }
    
    if (file_stat.st_size == 0) {
        ROS_WARN("File is empty: %s", filepath.c_str());
        return false;
    }
    
    if (access(filepath.c_str(), R_OK) != 0) {
        return false;
    }
    
    return true;
}

bool uat_log_cloud::uploadFile(const std::string& filepath, const std::string& data_type) {
    try {
        std::vector<std::string> tokens;
        std::istringstream stream(filepath);
        std::string token;
        while (std::getline(stream, token, '/')) {
            if (!token.empty()) {
                tokens.push_back(token);
            }
        }

        if (tokens.empty()) {
            ROS_ERROR("[%s] Invalid file path: %s", data_type.c_str(), filepath.c_str());
            return false;
        }

        std::string filename = tokens.back();

        std::ifstream file(filepath, std::ios::binary | std::ios::ate);
        if (!file) {
            ROS_ERROR("[%s] Failed to open file: %s", data_type.c_str(), filepath.c_str());
            return false;
        }

        int filesize = file.tellg();
        file.seekg(0, std::ios::beg);

        CurlUploader curlUploader;

        std::string upload_url = curlUploader.requestLogUrl(filename, filesize);
        if (upload_url.empty()) {
            ROS_ERROR("[%s] Failed to get upload URL for file: %s", data_type.c_str(), filename.c_str());
            file.close();
            return false;
        }

        ROS_INFO("requestLogUrl is %s",upload_url.c_str());

        std::string content;
        content.resize(filesize);
        if (!file.read(&content[0], filesize)) {
            ROS_ERROR("[%s] Failed to read file content: %s", data_type.c_str(), filepath.c_str());
            file.close();
            return false;
        }
        file.close();

        curlUploader.setMethod(CurlUploader::Method::PUT);
        curlUploader.setVerifySSL(false);

        if (!curlUploader.uploadData(upload_url, content, filename)) {
            ROS_ERROR("[%s] Upload failed for file: %s", data_type.c_str(), filename.c_str());
            return false;
        }

        std::string calculated_md5 = calculateFileMD5(filepath);
        std::transform(calculated_md5.begin(), calculated_md5.end(), 
                      calculated_md5.begin(), ::toupper);

        ROS_INFO("calculateFileMD5 is %s",calculated_md5.c_str());

        std::string response_md5 = curlUploader.getResponseMD5();
        if (response_md5 != calculated_md5) {
            ROS_ERROR("[%s] MD5 verification failed for file: %s", data_type.c_str(), filename.c_str());
            return false;
        }

        if (remove(filepath.c_str()) != 0) {
            ROS_WARN("[%s] Failed to delete uploaded file: %s", data_type.c_str(), filepath.c_str());
            return false;
        }

        ROS_INFO("[%s] Successfully uploaded file: %s", data_type.c_str(), filename.c_str());
        return true;

    } catch (const std::exception& e) {
        ROS_ERROR("[%s] Exception during upload: %s", data_type.c_str(), e.what());
        return false;
    }
}

void uat_log_cloud::listFiles(logfileinfo& fileinfo) {
    DIR* dir = opendir(fileinfo.data_directory.c_str());
    if (!dir) {
        std::cerr << "Cannot open directory: " << fileinfo.data_directory << '\n';
        return;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if (name == "." || name == "..") continue;

        std::string fullPath = fileinfo.data_directory + "/" + name;
        struct stat st;
        if (stat(fullPath.c_str(), &st) == 0 && S_ISREG(st.st_mode)) {
            //std::cout << "read file name is "<<fullPath << '\n';
            fileinfo.filename_queue.push(fullPath);
        }
    }

    closedir(dir);
}

std::string uat_log_cloud::generateFilename(const std::string& data_type)
{
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    static std::atomic<uint32_t> sequence{0};
    uint32_t seq = sequence++;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
    ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
    ss << "_" << data_type;
    ss << "_" << std::setfill('0') << std::setw(6) << seq;
    ss << ".dat";
    
    return ss.str();
}

std::string uat_log_cloud::calculateFileMD5(const std::string &filename)
{
    std::ifstream file(filename, std::ifstream::binary);
    if (!file) {
        ROS_ERROR("Cannot open file: %s", filename.c_str());
        return "";
    }

    MD5_CTX md5Context;
    MD5_Init(&md5Context);

    char buffer[1024 * 16]; // 16KB buffer
    while (file.good()) {
        file.read(buffer, sizeof(buffer));
        MD5_Update(&md5Context, buffer, file.gcount());
    }

    unsigned char result[MD5_DIGEST_LENGTH];
    MD5_Final(result, &md5Context);

    // 转换为十六进制字符串
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (const auto &byte : result) {
        ss << std::setw(2) << (int)byte;
    }

    return ss.str();
}
