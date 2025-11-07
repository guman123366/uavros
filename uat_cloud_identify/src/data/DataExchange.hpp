#pragma once
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <thread>
// #include <mutex> 
#include <atomic>


#define SN_LEN 33
#define FRAME_MAX_SIZE 256

static char * filepath = "/home/uatair/Data/ftp/vehicle_version.txt";
static char * snfilepath = "/tmp/vehicle_version.txt";
static char * snfilepath_backup = "/home/uatair/Vehicle_Firmware/vehicle_version.txt";
