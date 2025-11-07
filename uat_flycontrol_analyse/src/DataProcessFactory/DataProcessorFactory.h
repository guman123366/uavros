// DataProcessorFactory.h
#pragma once
#include "../DataAnalysis/DataAnalysisInterface.h"
#include "../DataPublisher/DataPublisherInterface.h"
#include <ros/ros.h>

enum class DataType {
    TD550,
    T1400
};

class DataProcessorFactory {
public:
    static std::shared_ptr<DataAnalysisInterface> createAnalyzer(DataType type);
    static std::shared_ptr<DataPublisherInterface> createPublisher(DataType type, ros::NodeHandle& nh);
};