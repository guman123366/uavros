#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

class OSDSubscriber {
public:
    OSDSubscriber() {
        // 初始化订阅者
        sub_ = nh_.subscribe("/uat_cloud_ros/osd", 10, &OSDSubscriber::dataCallback, this);
        ROS_INFO("OSD订阅者已启动，等待数据...");
    }

    // 数据接收回调函数
    void dataCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
        // 打印接收时间
        ros::Time now = ros::Time::now();
        ROS_INFO("\n--- 收到数据 [%f] ---", now.toSec());
        
        // 打印基本信息
        ROS_INFO("数据维度: %lu", msg->layout.dim.size());
        ROS_INFO("数据长度: %lu", msg->data.size());
        
        // 打印所有数据
        for(size_t i = 0; i < msg->data.size(); ++i) {
            ROS_INFO("元素[%lu]: %u (0x%02X)", i, msg->data[i], msg->data[i]);
        }
        
        // 添加分隔线
        ROS_INFO("---------------------");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "osd_subscriber_node");
    OSDSubscriber subscriber;
    ros::spin();
    return 0;
}