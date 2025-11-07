/*
	数据定义
*/
#pragma once

#include <iostream>

struct DataDefineInterface
{
	DataDefineInterface()
	{
		DataType = "";
	}

	virtual ~DataDefineInterface()=default;
	
	std::string DataType;
};