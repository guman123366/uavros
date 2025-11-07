#include <uavros.h>

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "uavros");

	uavros::UavRos uavros;
	uavros.spin();

	return 0;
}