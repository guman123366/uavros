// DataPublisherInterface.h
#pragma once
#include <memory>
#include "../DataDefine/DataDefineInterface.h"

class DataPublisherInterface {
public:
    virtual ~DataPublisherInterface() = default;
    virtual void publish(const std::shared_ptr<DataDefineInterface>& data) = 0;
};