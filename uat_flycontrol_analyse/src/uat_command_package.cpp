#include "uat_command_package.h"
#include <iostream>

uat_command_package::uat_command_package()
:m_bSendControl(false)
{
    memset(_send_buff,0,40);

}

uat_command_package::~uat_command_package()
{
    
}

void uat_command_package::package_switch_order(unsigned char order)
{
    std::lock_guard<std::mutex> lock(_buf_mutex);
    
    memset(&_send_buff[0],order&0xFF,3);
    m_bSendControl=true;
}

void uat_command_package::package_combine_order(unsigned char orderType,std::vector<double> vecData)
{
    std::lock_guard<std::mutex> lock(_buf_mutex);
    
    unsigned char* cZHBuffer;
	ushort nData = 0;
	int iData = 0;
	double dData = 0.0;
	int nWLNum = 0, nWPNum = 0, nLon = 0, nLat = 0, nAlt = 0, nSpeed = 0, nTime = 0;
	switch (orderType)
	{
	case 0x10://航点插入
		memset(&_send_buff[6], 0x10, 3);
		break;
	case 0x11://航点删除
		memset(&_send_buff[6], 0x11, 3);
		break;
	case 0x12://航点修改
		memset(&_send_buff[6], 0x12, 3);
		break;
	case 0x13://航点查询
		if (vecData.size() < 2)
		{
			return;
		}
		memset(&_send_buff[6], 0x13, 3);
		nWLNum = (int)vecData.at(0);
		nWPNum = (int)vecData.at(1);
		_send_buff[18] = nWLNum & 0xFF;
		_send_buff[19] = nWPNum & 0xFF;
		break;
	case 0x14://航点任务关闭
		memset(&_send_buff[6], 0x14, 3);
		break;
	case 0x15://航线查询
		memset(&_send_buff[6], 0x15, 3);
		break;
	case 0x21://点号遥调
		if (vecData.size() < 2)
		{
			return;
		}
		memset(&_send_buff[6], 0x21, 3);
		nWLNum = (int)vecData.at(0);
		nWPNum = (int)vecData.at(1);
		_send_buff[18] = nWLNum & 0xFF;
		_send_buff[19] = nWPNum & 0xFF;
		break;
	case 0x16://航线装订
		if (vecData.size() < 8)
		{
			return;
		}

		memset(&_send_buff[6], 0x16, 3);
		_send_buff[18] = (int)vecData.at(0);
		_send_buff[19] = (int)vecData.at(1);
		_send_buff[20] = (int)vecData.at(2);
		nLon = (int)(vecData.at(3) * 1.0e7);
		nLat = (int)(vecData.at(4) * 1.0e7);
		nAlt = (int)(vecData.at(5));
		nSpeed = (int)(vecData.at(6) * 2);
		nTime = (int)(vecData.at(7));
		memcpy(&_send_buff[21], &nLon, 4);
		memcpy(&_send_buff[25], &nLat, 4);
		memcpy(&_send_buff[29], &nAlt, 2);
		memcpy(&_send_buff[31], &nSpeed, 2);
		memcpy(&_send_buff[33], &nTime, 2);
		break;
	case 0x17://飞机起飞重量装订
		memset(&_send_buff[6], 0x17, 3);
		break;
	case 0x22://纵向位置遥调
		memset(&_send_buff[6], 0x22, 3);
		cZHBuffer = new unsigned char[2];
		dData = vecData.at(0);
		nData = (ushort)(dData * 10);
		memcpy(cZHBuffer, &nData, 2);
		memcpy(&_send_buff[29], cZHBuffer, 2);
		break;
	case 0x23://横向位置遥调
		memset(&_send_buff[6], 0x23, 3);
		cZHBuffer = new unsigned char[2];
		dData = vecData.at(0);
		nData = (ushort)(dData * 10);
		memcpy(cZHBuffer, &nData, 2);
		memcpy(&_send_buff[29], cZHBuffer, 2);
		break;
	case 0x24://高度遥调给定
		memset(&_send_buff[6], 0x24, 3);
		cZHBuffer = new unsigned char[2];
		dData = vecData.at(0);
		nData = (ushort)(dData * 2);
		memcpy(cZHBuffer, &nData, 2);
		memcpy(&_send_buff[29], cZHBuffer, 2);
		break;
	case 0x25://航向遥调给定
		memset(&_send_buff[6], 0x25, 3);
		cZHBuffer = new unsigned char[2];
		dData = vecData.at(0);
		nData = (ushort)(dData * 10);
		memcpy(cZHBuffer, &nData, 2);
		memcpy(&_send_buff[29], cZHBuffer, 2);
		break;
	case 0x26://纵向速度遥调给定
		memset(&_send_buff[6], 0x26, 3);
		cZHBuffer = new unsigned char[2];
		dData = vecData.at(0);
		nData = (ushort)(dData * 10);
		memcpy(cZHBuffer, &nData, 2);
		memcpy(&_send_buff[29], cZHBuffer, 2);
		break;
	case 0x27://垂直速度遥调给定
		memset(&_send_buff[6], 0x27, 3);
		cZHBuffer = new unsigned char[2];
		dData = vecData.at(0);
		nData = (ushort)(dData * 10);
		memcpy(cZHBuffer, &nData, 2);
		memcpy(&_send_buff[29], cZHBuffer, 2);
		break;
	case 0x28://侧向速度遥调给定
		memset(&_send_buff[6], 0x28, 3);
		cZHBuffer = new unsigned char[2];
		dData = vecData.at(0);
		nData = (ushort)(dData * 10);
		memcpy(cZHBuffer, &nData, 2);
		memcpy(&_send_buff[29], cZHBuffer, 2);
		break;
	case 0x31://位置偏差注入（预留）
		memset(&_send_buff[6], 0x31, 3);
		break;
	case 0x32://场高注入
		memset(&_send_buff[6], 0x32, 3);
		break;
	case 0x33://磁偏角注入（预留）
		memset(&_send_buff[6], 0x33, 3);
		break;
	case 0x34://A点坐标装订
		memset(&_send_buff[6], 0x34, 3);
		cZHBuffer = new unsigned char[10];
		iData = vecData.at(0) * 1.0e7;
		memcpy(cZHBuffer, &iData, 4);
		iData = vecData.at(1) * 1.0e7;
		memcpy(cZHBuffer + 4, &iData, 4);
		nData = (ushort)(vecData.at(2));
		memcpy(cZHBuffer + 8, &nData, 2);
		memcpy(&_send_buff[21], cZHBuffer, 10);
		break;
	case 0x35://B点坐标装订
		memset(&_send_buff[6], 0x35, 3);
		cZHBuffer = new unsigned char[10];
		iData = vecData.at(0) * 1.0e7;
		memcpy(cZHBuffer, &iData, 4);
		iData = vecData.at(1) * 1.0e7;
		memcpy(cZHBuffer + 4, &iData, 4);
		nData = (ushort)(vecData.at(2));
		memcpy(cZHBuffer + 8, &nData, 2);
		memcpy(&_send_buff[21], cZHBuffer, 10);
		break;
	case 0x36://C点坐标装订
		memset(&_send_buff[6], 0x36, 3);
		cZHBuffer = new unsigned char[10];
		iData = vecData.at(0) * 1.0e7;
		memcpy(cZHBuffer, &iData, 4);
		iData = vecData.at(1) * 1.0e7;
		memcpy(cZHBuffer + 4, &iData, 4);
		nData = (ushort)(vecData.at(2));
		memcpy(cZHBuffer + 8, &nData, 2);
		memcpy(&_send_buff[21], cZHBuffer, 10);
		break;
	case 0x37://迫降点1坐标装订
		memset(&_send_buff[6], 0x37, 3);
		cZHBuffer = new unsigned char[10];
		iData = vecData.at(0) * 1.0e7;
		memcpy(cZHBuffer, &iData, 4);
		iData = vecData.at(1) * 1.0e7;
		memcpy(cZHBuffer + 4, &iData, 4);
		nData = vecData.at(2) * 2;
		memcpy(cZHBuffer + 8, &nData, 2);
		memcpy(&_send_buff[21], cZHBuffer, 10);
		break;
	case 0x38://迫降点2坐标装订
		memset(&_send_buff[6], 0x38, 3);
		cZHBuffer = new unsigned char[10];
		iData = vecData.at(0) * 1.0e7;
		memcpy(cZHBuffer, &iData, 4);
		iData = vecData.at(1) * 1.0e7;
		memcpy(cZHBuffer + 4, &iData, 4);
		nData = vecData.at(2) * 2;
		memcpy(cZHBuffer + 8, &nData, 2);
		memcpy(&_send_buff[21], cZHBuffer, 10);
		break;
	case 0x39://迫降点3坐标装订
		memset(&_send_buff[6], 0x39, 3);
		cZHBuffer = new unsigned char[10];
		iData = vecData.at(0) * 1.0e7;
		memcpy(cZHBuffer, &iData, 4);
		iData = vecData.at(1) * 1.0e7;
		memcpy(cZHBuffer + 4, &iData, 4);
		nData = vecData.at(2) * 2;
		memcpy(cZHBuffer + 8, &nData, 2);
		memcpy(&_send_buff[21], cZHBuffer, 10);
		break;
	case 0x40://迫降点4坐标装订
		memset(&_send_buff[6], 0x40, 3);
		cZHBuffer = new unsigned char[10];
		iData = vecData.at(0) * 1.0e7;
		memcpy(cZHBuffer, &iData, 4);
		iData = vecData.at(1) * 1.0e7;
		memcpy(cZHBuffer + 4, &iData, 4);
		nData = vecData.at(2) * 2;
		memcpy(cZHBuffer + 8, &nData, 2);
		memcpy(&_send_buff[21], cZHBuffer, 10);
		break;
	default:
		break;
	}

	m_bSendControl = true;
}

void uat_command_package::package_send_buf(unsigned char *buf, int len)
{
    unsigned char sendbuf[128],tempbuf[39];
	int i=0;
    memset(sendbuf,0,128);
    memset(tempbuf,0,39);

    sendbuf[0x00] = 0x6F;
	sendbuf[0x01] = 0xEB;
	sendbuf[0x02] = 0x90;
	if (m_bSendControl)
	{
		memcpy(tempbuf, _send_buff, 39);
		memset(&tempbuf[3], 0x01, 3);
		unsigned short nCRCCheck = CalCRC16_CCITT(tempbuf, 37);
		memcpy(&tempbuf[37], &nCRCCheck, 2);
		m_bSendControl = false;
		memset(_send_buff, 0, 39);
	}
	else
	{
		int nSize = sendEmptyControlFrame(tempbuf);
	}
	for (int j = 0x03; j <= 0x12; j++)
	{
		sendbuf[j] = tempbuf[i++];
	}
	sendbuf[0x1B] = tempbuf[i++];
	sendbuf[0x1C] = tempbuf[i++];
	sendbuf[0x1D] = 0xAA;
	sendbuf[0x1E] = 0x55;
	sendbuf[0x1F] = 0xAA;
	sendbuf[0x20] = 0x6F;
	for (int k = 0x21; k <= 0x32; k++)
	{
		sendbuf[k] = tempbuf[i++];
	}
	sendbuf[0x3B] = tempbuf[i++];
	sendbuf[0x3C] = tempbuf[i++];
	sendbuf[0x3D] = tempbuf[i++];
	sendbuf[0x3E] = 0xEB;
	sendbuf[0x3F] = 0x90;

    len=128;
    memcpy(buf,sendbuf,128);
}

int uat_command_package::sendEmptyControlFrame(unsigned char* buffer)
{
	int nIndex = 0;
	memset(&buffer[nIndex], 0, 3); nIndex += 3;
	memset(&buffer[nIndex], 0x01, 3); nIndex += 3;
	memset(&buffer[nIndex], 0, 31); nIndex += 31;
	unsigned short nCheck = CalCRC16_CCITT(buffer, 37);
	memcpy(&buffer[37], &nCheck, 2);

	return 39;
}

unsigned short uat_command_package::CalCRC16_CCITT(unsigned char *chkbuf, int len)
{
	unsigned char byte_up = 0;
	unsigned char byte_temp = 0;
	unsigned short CRC_temp = 0;
	unsigned short CRC_code = 0xffff;
	unsigned short tempL = 0;

	for (int i = 0; i < len; i++)
	{
		tempL = chkbuf[i];

		byte_up = (unsigned char)(CRC_code >> 12);

		byte_temp = byte_up ^ (tempL >> 4);

		CRC_temp = CRC16_CCIT_table[byte_temp];

		CRC_code = ((CRC_code << 4) ^ CRC_temp) & 0xffff;

		byte_up = (unsigned char)(CRC_code >> 12);

		byte_temp = byte_up ^ (tempL & 0xf);

		CRC_temp = CRC16_CCIT_table[byte_temp];

		CRC_code = ((CRC_code << 4) ^ CRC_temp);
	}
	return CRC_code;
}
