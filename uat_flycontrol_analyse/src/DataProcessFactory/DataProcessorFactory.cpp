#include "DataProcessorFactory.h"
#include "../DataAnalysis/T1400DataAnalysis.h"
#include "../DataAnalysis/TD550DataAnalysis.h"
#include "../DataPublisher/TD550DataPublisher.h"


std::shared_ptr<DataAnalysisInterface> DataProcessorFactory::createAnalyzer(DataType type)
{
    // 根据类型创建新的分析器
    std::shared_ptr<DataAnalysisInterface> analyzer;
    
    switch (type) {
        case DataType::TD550:
            analyzer = std::make_shared<TD550DataAnalysis>();
            break;
        case DataType::T1400:
            //analyzer = std::make_shared<T1400DataAnalysis>();
            break;
        default:
            throw std::invalid_argument("Unknown data type");
    }

    return analyzer;
}

std::shared_ptr<DataPublisherInterface> DataProcessorFactory::createPublisher(DataType type, ros::NodeHandle &nh)
{
    // 根据类型创建新的发布器
    std::shared_ptr<DataPublisherInterface> publisher;
    
    switch (type) {
        case DataType::TD550:
            publisher = std::make_shared<TD550DataPublisher>(nh);
            break;
        case DataType::T1400:
            // 这里需要根据实际的 T1400DataPublisher 实现
            // publisher = std::make_shared<T1400DataPublisher>(nh);
            throw std::runtime_error("T1400DataPublisher not implemented yet");
            break;
        default:
            throw std::invalid_argument("Unknown data type");
    }

    return publisher;
}

