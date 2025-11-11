#include <ros/console.h>
#include <ros/ros.h>
#include "uat_log_cloud.h"
#include <fstream>
#include <iostream>
#include <string>

std::string trim(const std::string& str) {
  // 移除开头的空格及换行符
  auto start = std::find_if_not(str.begin(), str.end(), [](unsigned char ch) {
      return std::isspace(ch);
  });
  // 移除结尾的空格及换行符
  auto end = std::find_if_not(str.rbegin(), str.rend(), [](unsigned char ch) {
      return std::isspace(ch);
  }).base();

  // 返回处理后的字符串
  return (start < end) ? std::string(start, end) : std::string();
}

void replace_substring(std::string& str, const std::string& from, const std::string& to) {
  size_t pos = 0;
  while ((pos = str.find(from, pos)) != std::string::npos) { // 查找子串
      str.replace(pos, from.length(), to); // 替换子串
      pos += to.length(); // 更新查找位置
  }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "uat_log_cloud");
    ros::NodeHandle nh("~");

    uat_log_cloud log_cloud;

    //vehicle_sn
    std::string vehicle_sn;
    bool finded_vehicle_sn = false;
    while(!finded_vehicle_sn)
    {
        std::ifstream in_file("/home/uatair/Vehicle_Firmware/vehicle_version.txt");
        if(in_file)
        {
        std::string line;
        if (std::getline(in_file, line))
        {
            replace_substring(line, "vehicle uid:", "");
            vehicle_sn = trim(line);
            if (vehicle_sn.length() > 12)
            vehicle_sn = vehicle_sn.substr(vehicle_sn.length() - 12);
            std::cout << "cloud_vehicle vehicle_sn: " << vehicle_sn << std::endl;
            finded_vehicle_sn = true;
        }      
        // 关闭文件
        in_file.close();      
        }
        if(!finded_vehicle_sn)
        ros::Duration(1).sleep(); 
    }

    log_cloud.set_sn(vehicle_sn);
    
    ros::spin();
    
    return 0;
}