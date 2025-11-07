#pragma once

#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <mutex>

class uat_command_package
{
public:
    uat_command_package();
    ~uat_command_package();

    void package_switch_order(unsigned char order);
    void package_combine_order(unsigned char orderType,std::vector<double> vecData);

    void package_send_buf(unsigned char* buf,int len);

private:
    unsigned char _send_buff[40];
    bool m_bSendControl;
    std::mutex _buf_mutex;

    unsigned short CRC16_CCIT_table[16] = { 0x0, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5,
    0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b,
    0xc18c, 0xd1ad, 0xe1ce, 0xf1ef };
    unsigned short CalCRC16_CCITT(unsigned char *chkbuf, int len);
    int sendEmptyControlFrame(unsigned char* buffer);
};