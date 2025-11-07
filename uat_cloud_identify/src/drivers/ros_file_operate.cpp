#include "ros_file_operate.hpp"
namespace ROS_FILE_OPERATE {
   int FindVersion(char *s, version_info *target, int len) {  
        int pulse_num = 0;
        memset(target->name, 0, sizeof(target->name));
        memset(target->version, 0, sizeof(target->version));
        target->name_type = NAME_DEFAULT;
        if(strstr(s, "vehicle uid: ") != NULL) { 
            pulse_num = 20;
        }else if(strstr(s, "authen_url: ") != NULL) { 
            pulse_num = 21;
        }else if(strstr(s, "username: ") != NULL) { 
            pulse_num = 22;
        }else if(strstr(s, "password: ") != NULL) { 
            pulse_num = 23;
        }
        if(pulse_num == 20) {
            memcpy(target->name, "vehicle uid: ", strlen("vehicle uid: "));
            if(s[strlen("vehicle uid: ")] == '"') {
                memcpy(&target->version[0], s + strlen("vehicle uid: ") + 1, len - strlen("vehicle uid: ") - 2);
            }else {
                memcpy(&target->version[0], s + strlen("vehicle uid: "), len - strlen("vehicle uid: "));
            }
            target->name_type = NAME_SN_Q20;
            ROS_INFO("Find %s Version %s\n", target->name, target->version);     
            return UNZIP_OK; 
        }else if(pulse_num == 21) {
            memcpy(target->name, "authen_url: ", strlen("authen_url: "));
            if(s[strlen("authen_url: ")] == '"') {
                memcpy(&target->version[0], s + strlen("authen_url: ") + 1, len - strlen("authen_url: ") - 2);
            }else {
                memcpy(&target->version[0], s + strlen("authen_url: "), len - strlen("authen_url: "));
            }
            target->name_type = NAME_AUTHEN_URL;
            ROS_INFO("Find %s Version %s\n", target->name, target->version);     
            return UNZIP_OK; 
        }else if(pulse_num == 22) {
            memcpy(target->name, "username: ", strlen("username: "));
            if(s[strlen("username: ")] == '"') {
                memcpy(&target->version[0], s + strlen("username: ") + 1, len - strlen("username: ") - 2);
            }else {
                memcpy(&target->version[0], s + strlen("username: "), len - strlen("username: "));
            }
            target->name_type = NAME_USERNAME;
            ROS_INFO("Find %s Version %s\n", target->name, target->version);     
            return UNZIP_OK; 
        }else if(pulse_num == 23) {
            memcpy(target->name, "password: ", strlen("password: "));
            if(s[strlen("password: ")] == '"') {
                memcpy(&target->version[0], s + strlen("password: ") + 1, len - strlen("password: ") - 2);
            }else {
                memcpy(&target->version[0], s + strlen("password: "), len - strlen("password: "));
            }
            target->name_type = NAME_PASSWORD;
            ROS_INFO("Find %s Version %s\n", target->name, target->version);     
            return UNZIP_OK; 
        }
        return UNZIP_NULL_FILE;
    }
    int ReadVersionFile(const char * SourcePath, std::string &sn_code) {
        std::ifstream in;
        version_info target;
        char readbuf[FRAME_MAX_SIZE] = {0};
        ROS_INFO("read version file is %s\n", SourcePath);
        in.open(SourcePath, std::ios::in); //打开源文件
        
        if(in.fail()) {//打开源文件失败
            ROS_INFO("ReadVersionFile Fail\n");
            in.close();
            return UNZIP_NULL_FILE;
        }
        while(in.getline(readbuf, sizeof(readbuf))) {
            FindVersion(readbuf, &target, strlen(readbuf));  
            if(target.name_type == NAME_SN_Q20) { 
                sn_code = target.version;
            }
            //ROS_INFO("len is %s \n", readbuf);
        }
        in.close();
        return UNZIP_OK;
    }
    int ReadVersionFile(const char * SourcePath, struct DRIVER_DATA_CODE::mg_authen_cmd &mg_authen_cmd) {
        std::ifstream in;
        version_info target;
        char readbuf[FRAME_MAX_SIZE] = {0};
        ROS_INFO("read version file is %s\n", SourcePath);
        in.open(SourcePath, std::ios::in); //打开源文件
        
        if(in.fail()) {//打开源文件失败
            ROS_INFO("ReadVersionFile Fail\n");
            in.close();
            return UNZIP_NULL_FILE;
        }
        while(in.getline(readbuf, sizeof(readbuf))) {
            FindVersion(readbuf, &target, strlen(readbuf));  
            if(target.name_type == NAME_SN_Q20) { 
                mg_authen_cmd.SnCode = target.version;
            }else if(target.name_type == NAME_AUTHEN_URL) { 
                mg_authen_cmd.authen_url = target.version;
            }else if(target.name_type == NAME_USERNAME) { 
                mg_authen_cmd.username = target.version;
            }else if(target.name_type == NAME_PASSWORD) { 
                mg_authen_cmd.password = target.version;
            }
            //ROS_INFO("len is %s \n", readbuf);
        }
        in.close();
        return UNZIP_OK;
    }
}


