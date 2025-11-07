// TD550DataPublisher.h
#include "DataPublisherInterface.h"
#include "../DataDefine/TD550TelemetryData.h"
#include <ros/ros.h>

class TD550DataPublisher : public DataPublisherInterface {
public:
    TD550DataPublisher(ros::NodeHandle& nh);
    void publish(const std::shared_ptr<DataDefineInterface>& data) override;

private:
    ros::Publisher first_one_pub;
    ros::Publisher first_two_pub;
    ros::Publisher first_three_pub;
    ros::Publisher first_four_pub;
    ros::Publisher second_one_pub;
    ros::Publisher second_two_pub;
    ros::Publisher second_three_pub;
    ros::Publisher second_four_pub;
    ros::Publisher third_one_pub;
    ros::Publisher third_two_pub;
    ros::Publisher third_three_pub;
    ros::Publisher third_four_pub;
};