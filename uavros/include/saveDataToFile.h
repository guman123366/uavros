#pragma once

#include <iostream>
#include <fstream>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <vector>

namespace uavros{

class SaveDataToFile
{
public:
    SaveDataToFile(const std::string& base_name);
    ~SaveDataToFile();
    // 生成带时间戳的新文件名
    std::string generate_filename();
    // 创建新文件
    void open_new_file();
    // 写入数据（自动处理文件切换）
    void write_data(const std::vector<uint8_t>& data);
private:
    std::ofstream outfile;
    std::string base_filename;  // 基础文件名（如 "serial_data"）
    size_t current_file_size;   // 当前文件大小（字节）
    const size_t MAX_FILE_SIZE = 20 * 1024 * 1024; // 20MB
    /* data */
};
}
