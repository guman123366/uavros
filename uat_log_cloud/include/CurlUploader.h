#pragma once

#include <string>
#include <map>
#include <vector>
#include <functional>
#include <curl/curl.h>
#include <ros/ros.h>
#include <openssl/md5.h>

// 内部结构体
struct UploadData {
    FILE* file;
    std::string filename;
    curl_off_t filesize;
    size_t data_offset;
    std::string data_content;
};

class CurlUploader {
public:
    // 上传方法枚举
    enum class Method {
        PUT,
        POST
    };

    // 进度回调类型
    using ProgressCallback = std::function<void(double progress, curl_off_t uploaded, curl_off_t total)>;
    
    // 响应回调类型
    using ResponseCallback = std::function<void(const std::string& response, long status_code)>;

    CurlUploader();
    ~CurlUploader();

    // 禁用拷贝构造和赋值
    CurlUploader(const CurlUploader&) = delete;
    CurlUploader& operator=(const CurlUploader&) = delete;

    // 设置基本选项
    void setMethod(Method method);
    void setVerifySSL(bool verify);
    void setTimeout(long timeout_seconds);
    void setVerbose(bool verbose);
    void setCaPath(const std::string& path);

    // 设置自定义头
    void setHeader(const std::string& key, const std::string& value);
    void removeHeader(const std::string& key);

    // 设置回调函数
    void setProgressCallback(ProgressCallback callback);
    void setResponseCallback(ResponseCallback callback);

    // 文件上传方法
    bool uploadFile(const std::string& url, const std::string& file_path, const std::string& field_name = "");
    bool uploadData(const std::string& url, const std::string& data, const std::string& field_name = "");

    // 获取错误信息
    std::string getLastError() const;
    long getLastResponseCode() const;

    // 请求日志上传链接
    std::string requestLogUrl(const std::string& fileName,const long& fileSize,const std::string& uav_type,const std::string& sn,
                             const std::string& sortie,const std::string& logType);

    //获取返回MD5
    std::string getResponseMD5();
private:   

    // libcurl回调函数
    static int progressCallback(void* clientp, curl_off_t dltotal, curl_off_t dlnow, 
                               curl_off_t ultotal, curl_off_t ulnow);
    static size_t writeCallback(char* ptr, size_t size, size_t nmemb, void* userdata);
    static size_t readCallback(char* ptr, size_t size, size_t nmemb, void* userdata);
    // 回调函数，用于处理从服务器接收到的数据
    static size_t logUrlCallback(void *contents, size_t size, size_t nmemb, void *userp);
    static size_t headerCallback(char* buffer, size_t size, size_t nitems, void* userdata);

    // 内部方法
    bool initCurl();
    bool setupCommonOptions(const std::string& url, CURL* curl, UploadData& upload_data);
    bool setupPutOptions(CURL* curl, UploadData& upload_data);
    bool setupPostOptions(CURL* curl, UploadData& upload_data, const std::string& field_name);
    void cleanup();
private:
    // 成员变量
    CURL* curl_;
    Method method_;
    bool verify_ssl_;
    long timeout_;
    bool verbose_;
    std::string last_error_;
    long last_response_code_;
    std::string ca_certificate_dir_;
 
    std::map<std::string, std::string> headers_;
    
    ProgressCallback progress_callback_;
    ResponseCallback response_callback_;
    std::string response_content_;

    std::string send_url;
    std::string response_MD5;
};