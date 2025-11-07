#include "CurlUploader.h"
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <cstring>
#include <memory>
#include "cJSON.h"

CurlUploader::CurlUploader() 
    : curl_(nullptr), 
      method_(Method::POST),
      verify_ssl_(true),
      timeout_(30),
      verbose_(false),
      send_url(""),
      last_response_code_(0) {
    curl_global_init(CURL_GLOBAL_DEFAULT);
}

CurlUploader::~CurlUploader() {
    cleanup();
    curl_global_cleanup();
}

void CurlUploader::setMethod(Method method) {
    method_ = method;
}

void CurlUploader::setVerifySSL(bool verify) {
    verify_ssl_ = verify;
}

void CurlUploader::setTimeout(long timeout_seconds) {
    timeout_ = timeout_seconds;
}

void CurlUploader::setVerbose(bool verbose) {
    verbose_ = verbose;
}

void CurlUploader::setCaPath(const std::string& path) {
    ca_certificate_dir_ = path;
}

void CurlUploader::setHeader(const std::string& key, const std::string& value) {
    headers_[key] = value;
}

void CurlUploader::removeHeader(const std::string& key) {
    headers_.erase(key);
}

void CurlUploader::setProgressCallback(ProgressCallback callback) {
    progress_callback_ = callback;
}

void CurlUploader::setResponseCallback(ResponseCallback callback) {
    response_callback_ = callback;
}

bool CurlUploader::uploadFile(const std::string& url, const std::string& file_path, const std::string& field_name) {
    struct stat file_info;
    if (stat(file_path.c_str(), &file_info) != 0) {
        last_error_ = "文件不存在或无法访问: " + file_path;
        return false;
    }

    if (!initCurl()) {
        return false;
    }

    UploadData upload_data;
    upload_data.file = nullptr;
    upload_data.filename = file_path;
    upload_data.filesize = file_info.st_size;
    upload_data.data_offset = 0;
    std::string readBuffer; // 用于存储服务器响应

    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &readBuffer);

    setHeader("Content-Length", std::to_string(file_info.st_size));
    setHeader("Content-Type", "application/octet-stream");

    // 设置通用选项
    if (!setupCommonOptions(url, curl_, upload_data)) {
        return false;
    }

    // 根据方法设置特定选项
    bool success = false;
    if (method_ == Method::PUT) {
        success = setupPutOptions(curl_, upload_data);
    } else {
        success = setupPostOptions(curl_, upload_data, field_name);
    }

    if (!success) {
        cleanup();
        return false;
    }

    // 执行上传
    response_content_.clear();
    CURLcode res = curl_easy_perform(curl_);

    // 清理资源
    if (upload_data.file) {
        fclose(upload_data.file);
    }

    // 检查结果
    if (res != CURLE_OK) {
        last_error_ = curl_easy_strerror(res);
        cleanup();
        return false;
    }

    // 获取响应码
    curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &last_response_code_);

    // 调用响应回调
    if (response_callback_) {
        response_callback_(response_content_, last_response_code_);
    }

    cleanup();
    return true;
}

bool CurlUploader::uploadData(const std::string& url, const std::string& data, const std::string& field_name) {
    if (!initCurl()) {
        return false;
    }
    UploadData upload_data;
    upload_data.file = nullptr;
    upload_data.filename = "";
    upload_data.filesize = data.size();
    upload_data.data_content = data;
    upload_data.data_offset = 0;

    setHeader("Content-Type", "application/octet-stream");

    // 设置通用选项
    if (!setupCommonOptions(url, curl_, upload_data)) {
        return false;
    }
   
    // 根据方法设置特定选项
    bool success = false;
    if (method_ == Method::PUT) {
        success = setupPutOptions(curl_, upload_data);
    } else {
        success = setupPostOptions(curl_, upload_data, field_name);
    }

    if (!success) {
        cleanup();
        return false;
    }   

    // 执行上传
    response_content_.clear();
    CURLcode res = curl_easy_perform(curl_);

    // 检查结果
    if (res != CURLE_OK) {
        last_error_ = curl_easy_strerror(res);
        cleanup();
        return false;
    }

    // 获取响应码
    curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &last_response_code_);

    // 调用响应回调
    if (response_callback_) {
        response_callback_(response_content_, last_response_code_);
    }

    cleanup();

    ROS_INFO("last_response_code_ is %d",last_response_code_);

    if(200==last_response_code_)
    {
        return true;
    }
    else
    {
       return false; 
    }
}

std::string CurlUploader::getLastError() const {
    return last_error_;
}

long CurlUploader::getLastResponseCode() const {
    return last_response_code_;
}

std::string CurlUploader::requestLogUrl(const std::string &fileName, const int &fileSize)
{
    CURLcode res;
    std::string readBuffer; // 用于存储服务器响应

    if (!initCurl()) {
        return "";
    }

    if (curl_) {
        // 1. 设置目标 URL (你的云平台 API 端点)
        curl_easy_setopt(curl_, CURLOPT_URL, "https://cloud.uatair.com/lfy-api/manage/logFile/getUploadUrl");

        // 2. 设置 SSL 验证（重要！生产环境应保持开启）
        curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYPEER, verify_ssl_ ? 1L : 0L);
        curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYHOST, verify_ssl_ ? 2L : 0L);
        // 如果你的服务器使用自签名证书，可以临时关闭验证（不推荐用于生产）
        // curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYPEER, 0L);
        // curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYHOST, 0L);

        // 3. 设置回调函数来处理响应数据
        curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, logUrlCallback);
        curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &readBuffer);

        // 4. 如果是 POST 请求，设置方法和数据
        // 例如，发送 JSON 数据
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);
        
        cJSON *data = cJSON_CreateObject();
        cJSON_AddStringToObject(data,"model","T1400");
        cJSON_AddStringToObject(data,"sn","05KBBE000015");
        cJSON_AddStringToObject(data,"filename",fileName.c_str());
        cJSON_AddNumberToObject(data,"size",fileSize);

        char *jsonString = cJSON_Print(data);
        std::string postData = jsonString;
        cJSON_Delete(data);
        free(jsonString);
        curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, postData.c_str());

        // 5. 执行请求
        res = curl_easy_perform(curl_);

        // 6. 检查错误
        if (res != CURLE_OK) {
            ROS_ERROR("curl_easy_perform() failed: %s", curl_easy_strerror(res));
        } else {
            // 请求成功，处理 readBuffer 中的数据
            //ROS_INFO("Server Response: %s", readBuffer.c_str());

            cJSON *json = cJSON_Parse(readBuffer.c_str());
            if (NULL == json)
            {
                const char *error_ptr = cJSON_GetErrorPtr();
                if (error_ptr != NULL)
                {
                    fprintf(stderr, "Error before: %s\n", error_ptr);
                }
                cJSON_Delete(json);
                cleanup();
                return "";
            }

            cJSON *jsonData = cJSON_GetObjectItemCaseSensitive(json, "data");
            if(jsonData == NULL)
            {
                cleanup();
                return "";
            }

            std::string strKey="url";
            cJSON* value = cJSON_GetObjectItemCaseSensitive(jsonData, strKey.c_str());
            if (cJSON_IsString(value) && (value->valuestring != NULL))
            {
                send_url = value->valuestring;
            }
        }
    }

    cleanup();

    return send_url;
}

std::string CurlUploader::getResponseMD5()
{
    return response_MD5;
}

// libcurl回调函数实现
size_t CurlUploader::readCallback(char* ptr, size_t size, size_t nmemb, void* userdata) {
    UploadData* upload_data = static_cast<UploadData*>(userdata);
    size_t buffer_size = size * nmemb;
    if (!upload_data->filename.empty()) {
        // 文件上传
        if (!upload_data->file) {
            upload_data->file = fopen(upload_data->filename.c_str(), "rb");
            if (!upload_data->file) {
                return CURL_READFUNC_ABORT;
            }
        }
        return fread(ptr, size, nmemb, upload_data->file);
    } else {
        // 数据上传
        size_t data_remaining = upload_data->data_content.size() - upload_data->data_offset;
        if (data_remaining == 0) {
            return 0; // EOF
        }

        size_t bytes_to_copy = std::min(buffer_size, data_remaining);
        memcpy(ptr, upload_data->data_content.data() + upload_data->data_offset, bytes_to_copy);
        upload_data->data_offset += bytes_to_copy;
        return bytes_to_copy;
    }
}

size_t CurlUploader::logUrlCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);

    cJSON *json = cJSON_Parse((char*)contents);
    if (NULL == json)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            fprintf(stderr, "Error before: %s\n", error_ptr);
        }
        cJSON_Delete(json);
        return size * nmemb;;
    }

    cJSON *jsonData = cJSON_GetObjectItemCaseSensitive(json, "data");
    if(jsonData == NULL)
        return size * nmemb;;

    std::string logUrl="";
    std::string strKey="url";
    cJSON* value = cJSON_GetObjectItemCaseSensitive(jsonData, strKey.c_str());
    if (cJSON_IsString(value) && (value->valuestring != NULL))
    {
        logUrl = value->valuestring;
    }
    return size * nmemb;
}

int CurlUploader::progressCallback(void* clientp, curl_off_t dltotal, curl_off_t dlnow, 
                                  curl_off_t ultotal, curl_off_t ulnow) {
    CurlUploader* uploader = static_cast<CurlUploader*>(clientp);
    if (uploader->progress_callback_ && ultotal > 0) {
        double progress = static_cast<double>(ulnow) / static_cast<double>(ultotal) * 100.0;
        uploader->progress_callback_(progress, ulnow, ultotal);
    }
    return 0;
}

// 回调函数用于写入响应头
size_t CurlUploader::headerCallback(char* buffer, size_t size, size_t nitems, void* userdata) {
    CurlUploader* uploader = static_cast<CurlUploader*>(userdata);
    size_t totalSize = size * nitems;
    std::string headerLine(buffer, totalSize);

    ROS_INFO("headerCallback is %s",headerLine.c_str());

    if(std::string::npos!=headerLine.find("ETag"))
    {
        //查找冒号的位置
        size_t first_quote=headerLine.find('\"');
        if(std::string::npos!=first_quote)
        {
            size_t second_quote=headerLine.find('\"',first_quote+1);
            if(std::string::npos!=second_quote)
            {
                if(uploader)
                {
                    uploader->response_MD5=headerLine.substr(first_quote+1,second_quote-first_quote-1);
                }
                else
                {
                    ROS_INFO("uploader is created fialed!!!!");
                }
            }
        }
    }
    
    return totalSize;
}


size_t CurlUploader::writeCallback(char* ptr, size_t size, size_t nmemb, void* userdata) {
    CurlUploader* uploader = static_cast<CurlUploader*>(userdata);
    size_t total_size = size * nmemb;
    uploader->response_content_.append(ptr, total_size);
    return total_size;
}

// 内部方法实现
bool CurlUploader::initCurl() {
    cleanup();
    curl_ = curl_easy_init();
    if (!curl_) {
        last_error_ = "无法初始化libcurl";
        return false;
    }
    return true;
}

bool CurlUploader::setupCommonOptions(const std::string& url, CURL* curl, UploadData& upload_data) {
    // 设置URL
    int outlen;
    char *decodedUrl = curl_easy_unescape(NULL, url.c_str(), 0, &outlen);

    std::string responseHeaders;

    if (!decodedUrl) {
        last_error_ = "decode url fail: " + url;
        return false;
    }
    curl_easy_setopt(curl, CURLOPT_URL, decodedUrl);

    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_DEFAULT_PROTOCOL, "https");

    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, verify_ssl_ ? 1L : 0L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, verify_ssl_ ? 2L : 0L);

    if (!ca_certificate_dir_.empty()) {
        curl_easy_setopt(curl, CURLOPT_CAPATH, ca_certificate_dir_.c_str());
    }  
    
    // 设置超时
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_);
    
    // 设置详细模式
    curl_easy_setopt(curl, CURLOPT_VERBOSE, verbose_ ? 1L : 0L);
    
    // 设置进度回调
    curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, progressCallback);
    curl_easy_setopt(curl, CURLOPT_XFERINFODATA, this);
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);

    // 设置头回调函数
    curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, headerCallback);
    curl_easy_setopt(curl, CURLOPT_HEADERDATA, this);
    
    // 设置响应回调
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, this);    
    
    // 设置自定义头
    if (!headers_.empty()) {
        struct curl_slist* header_list = nullptr;
        for (const auto& header : headers_) {
            std::string header_str = header.first + ": " + header.second;
            header_list = curl_slist_append(header_list, header_str.c_str());
        }
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, header_list);
    } 
    
    return true;
}

bool CurlUploader::setupPutOptions(CURL* curl, UploadData& upload_data) {
    curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
    curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, upload_data.filesize);
    curl_easy_setopt(curl, CURLOPT_READFUNCTION, readCallback);
    curl_easy_setopt(curl, CURLOPT_READDATA, &upload_data);
    return true;
}

bool CurlUploader::setupPostOptions(CURL* curl, UploadData& upload_data, const std::string& field_name) {
    if (upload_data.filename.empty()) {
        // 数据上传
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, upload_data.filesize);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, upload_data.data_content.c_str());
    } else {
        // 文件上传（使用multipart/form-data）
        curl_mime* mime = curl_mime_init(curl);
        curl_mimepart* part = curl_mime_addpart(mime);
        
        curl_mime_name(part, field_name.empty() ? "file" : field_name.c_str());
        curl_mime_filedata(part, upload_data.filename.c_str());
        
        curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
    }
    return true;
}

void CurlUploader::cleanup() {
    if (curl_) {
        curl_easy_cleanup(curl_);
        curl_ = nullptr;
    }
}
