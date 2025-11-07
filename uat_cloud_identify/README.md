# uat_cloud_identify

联飞云平台认证节点

本节点实现功能主要针对云平台认证，主要从文件获取认证的网址和账号密码，完成对mqtt认证的过程。

第一步：创建identify.txt 并将文件放在
路径1："/home/uatair/Data/ftp/identify.txt";
路径2："/tmp/identify.txt"  
路径3： "/home/uatair/Vehicle_Firmware/identify.txt";
内容：
authen_url: "http://airport.uatair.com:9999/driver/auth/authenticate"
username: "admin"
password: "I5rtj/cgBACo/Vf/lcJunA=="
vehicle uid: 0b20373948

第二步：可创建vehicle_version.txt 并将文件放在以下路径，或者不创建该文件
路径1："/home/uatair/Data/ftp/vehicle_version.txt"
路径2： "/tmp/vehicle_version.txt"
路径3： "/home/uatair/Vehicle_Firmware/vehicle_version.txt"
vehicle uid: 0b20373948

第三步：区分不同的参数设备，通过ROS1运行
如果获取飞机的账号密码，输入roslaunch identify identify.launch 或者roslaunch identify identify.launch type:=device 或者roslaunch identify identify.launch type:="device" 
如果获取机库的账号密码，输入roslaunch identify identify.launch type:=hangar

第四步：当连接成功的时候，会通过mavros的rosservice主题 /uat/UAT_AuthenCtl.srv发送以下内容
uint8 GCS_CTRL = 1
uint8 CLOUD_SERVER = 2
uint8 source

string url
string clientid
string username
string password
string key 
string host_ip
int32 host_port
---
bool ok

或者可以通过以下命令模拟认证包对外发送mqtt消息：
rosservice call /uat/UAT_AuthenCtl 2 mqtt://airport.uatair.com:11883 device_UAQ20231220_1719285962423 UAQ20231220 5hXnb8cFbmtfqwSfGA8fgT2kiMQDxCqu hi 47.113.104.44 11883

第五步：当mqtt交互包运行错误的时候，可以通过mavros提供的uat/UAT_MqttAuthenStatus.msg将自身链接状态发送给认证包
uint8 CONNECT_DEFAULT = 0
uint8 CONNECT_OK = 1
uint8 CONNECT_PARAM_GET = 2
uint8 CONNECT_WAIT = 3
uint8 CONNECT_FAIL = 4
uint8 MqttConnectState
当认证包获取到 MqttConnectState = CONNECT_FAIL的时候，将会重新跟云端进行交互重新根据authen.txt获取新的账号和密码