/*
	������������������ݽӿ�
*/

#pragma once
#include "../DataDefine/DataDefineInterface.h"
#include <memory>

class DataAnalysisInterface
{
public:
	virtual ~DataAnalysisInterface()= default;

	virtual std::shared_ptr<DataDefineInterface> AnalyseData(unsigned char* buf, int nLength) = 0;
};
