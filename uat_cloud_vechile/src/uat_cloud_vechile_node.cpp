#include <ros/ros.h>
#include <uat_cloud_vechile/cloud_vehicle_base_plugin.h>
#include <pluginlib/class_loader.hpp>

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

int main(int argc, char **argv)
{
  std::cout << "------------------------------------------ua cloud vehicle main" <<std::endl;
  ros::init(argc, argv, "uat_cloud_vechile");
  ros::NodeHandle nh;

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

  bool uat_cloud_en = false;
  do
  {
      nh.getParam("/cloud_en", uat_cloud_en);
      ros::Duration(1).sleep(); 
  } while (!uat_cloud_en);

  pluginlib::ClassLoader<uat::plugin::cloud_vehicle_bridge::PluginBase> plugin_loader("uat_cloud_vechile", "uat::plugin::cloud_vehicle_bridge::PluginBase");
  std::vector<uat::plugin::cloud_vehicle_bridge::PluginBase::Ptr> loaded_plugins;
  std::cout << "uat_cloud_vechile  Loading plugins..." << std::endl;
  std::vector<std::string> vecNames=plugin_loader.getDeclaredClasses();
  std::cout<<"---------------loading plugins size is"<<vecNames.size()<<std::endl;
  for (const auto &plugin_name : plugin_loader.getDeclaredClasses())
  {
    try
    {
      auto plugin = plugin_loader.createInstance(plugin_name);
      ROS_INFO_STREAM("Plugin " << plugin_name << " loaded");
      
      if (plugin_name.compare("uat::plugin::MesssagePlugin")==0)
      {
        //plugin->set_uat_msgcode_data(uat_data);
        ROS_INFO("Transmit the content of MSGCODE to the OSD.");
      }      
      plugin->initializePluginBase();
      plugin->initialize();
      plugin->set_device_sn(vehicle_sn); 
      loaded_plugins.push_back(plugin);
    }
    catch (const std::exception &e)
    {      
      std::cerr << e.what() << '\n';
    }
  }

  ros::AsyncSpinner spinner(4 /* threads */);
  spinner.start();
  ros::waitForShutdown();
  ROS_INFO("Stopping cloud_vehicle_bridge...");
  spinner.stop();
  return 0;
}