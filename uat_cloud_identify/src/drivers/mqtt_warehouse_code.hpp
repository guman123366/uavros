#pragma once
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <iostream>
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <time.h>
#include <stdio.h>
#include <sys/time.h>
#include "cJSON.h"
#include "driver_data_code.hpp"
#include <string>

namespace CLOUD_AUTHEN_EVENT {
	static int cloud_warehouse_encode(struct DRIVER_DATA_CODE::mg_authen_cmd &mg_authen_cmd, std::string &res_string) {
		cJSON *temp = NULL;          

		/* 创建一个JSON对象，｛｝扩起来 */
		cJSON *obj = cJSON_CreateObject();
		if (obj == NULL) {
			cJSON_Delete(obj);
			return -1;
		}
	    
		temp = cJSON_CreateString(mg_authen_cmd.username.c_str());
		if (temp == NULL) {
			cJSON_Delete(obj);
			return -1;
		}
		cJSON_AddItemToObject(obj, "username", temp);

		temp = cJSON_CreateString(mg_authen_cmd.password.c_str()); 
		cJSON_AddItemToObject(obj, "password", temp);

		temp = cJSON_CreateString(mg_authen_cmd.type.c_str()); 
		cJSON_AddItemToObject(obj, "type", temp);

		temp = cJSON_CreateString(mg_authen_cmd.SnCode.c_str()); 
		cJSON_AddItemToObject(obj, "gateway", temp);

		// res_string = cJSON_Print(obj);          // 呈现为JSON格式 
		char *str = cJSON_PrintUnformatted(obj);   // 呈现为无格式
		res_string = str;   
		std::cout << res_string << std::endl;
		cJSON_free(str);
		cJSON_Delete(obj);
		return 0;
	}
	static int cloud_warehouse_decode(const char *res_string, struct DRIVER_DATA_CODE::mg_authen_state &mg_authen_state) {
		cJSON *pause_json = NULL;   // pause_json 操作对象，可代表 {} 扩起来的内容
		cJSON *temp_1 = NULL;   
		cJSON *temp = NULL;             

		pause_json = cJSON_Parse(res_string); /* 开始解析 */
		if (NULL == pause_json) {
			const char *error_ptr = cJSON_GetErrorPtr();
			if (error_ptr != NULL) {
				fprintf(stderr, "Error before: %s\n", error_ptr);
			}
			cJSON_Delete(pause_json);
		}
	
		temp = cJSON_GetObjectItemCaseSensitive(pause_json, "msg"); 
		if (cJSON_IsString(temp) && (temp->valuestring != NULL)) {
			mg_authen_state.msg = temp->valuestring;
		}
	
		temp_1 =  cJSON_GetObjectItemCaseSensitive(pause_json, "data");  //获取result键对应的值
		temp = cJSON_GetObjectItemCaseSensitive(temp_1, "url"); 
		if (cJSON_IsString(temp) && (temp->valuestring != NULL)) {
			mg_authen_state.url = temp->valuestring;
		}

		temp = cJSON_GetObjectItemCaseSensitive(temp_1, "clientid"); 
		if (cJSON_IsString(temp) && (temp->valuestring != NULL)) {
			mg_authen_state.clientid = temp->valuestring;
		}

		temp = cJSON_GetObjectItemCaseSensitive(temp_1, "username"); 
		if (cJSON_IsString(temp) && (temp->valuestring != NULL)) {
			mg_authen_state.username = temp->valuestring;
		}

		temp = cJSON_GetObjectItemCaseSensitive(temp_1, "password"); 
		if (cJSON_IsString(temp) && (temp->valuestring != NULL)) {
			mg_authen_state.password = temp->valuestring;
		}

		temp = cJSON_GetObjectItemCaseSensitive(temp_1, "key"); 
		if (cJSON_IsString(temp) && (temp->valuestring != NULL)) {
			mg_authen_state.key = temp->valuestring;
		}

		temp =  cJSON_GetObjectItemCaseSensitive(pause_json, "code");  //获取result键对应的值
		if (cJSON_IsNumber(temp)) {
			mg_authen_state.code = temp->valueint;
		}
		cJSON_Delete(pause_json);//释放内存
		return 0;
	}
};