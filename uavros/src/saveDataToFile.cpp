#include "saveDataToFile.h"

using namespace uavros;

SaveDataToFile::SaveDataToFile(const std::string& base_name)
: base_filename(base_name), current_file_size(0) {
    open_new_file(); // 初始化时创建第一个文件
}

SaveDataToFile::~SaveDataToFile()
{
    if (outfile.is_open()) {
        outfile.close();
    }
}

std::string SaveDataToFile::generate_filename()
{
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << base_filename << "_" 
        << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".dat";
    return ss.str();
}

void SaveDataToFile::open_new_file()
{
    if (outfile.is_open()) {
        outfile.close();
    }
    std::string new_filename = generate_filename();
    outfile.open(new_filename, std::ios::binary);
    current_file_size = 0;
    std::cout << "切换到新文件: " << new_filename << std::endl;
}

void SaveDataToFile::write_data(const std::vector<uint8_t> &data)
{
    if (!outfile.is_open()) {
        std::cerr << "文件未打开！" << std::endl;
        return;
    }

    // 检查是否需要切换文件
    if (current_file_size + data.size() > MAX_FILE_SIZE) {
        open_new_file();
    }

    // 写入数据
    outfile.write(reinterpret_cast<const char*>(data.data()), data.size());
    outfile.flush(); // 确保数据写入磁盘
    current_file_size += data.size();
}
