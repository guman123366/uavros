std_msgs/Header header

uint8 NORMAL_STATE = 1
uint8 ERROR_STATE = 0
uint8 BITMASK_IOT = 1
uint8 BITMASK_UALINK = 2
uint8 STREAM_TYPE_FPV = 1
uint8 STREAM_TYPE_MAIN = 2
uint8 STREAM_TYPE_MAIN_WITH_FPV = 3
uint8 STREAM_TYPE_FPV_WITH_MAIN = 4

uint8 NETWORK_TYPE_4G = 4
uint8 NETWORK_TYPE_5G = 5

uint8 Network_type  #current type 4g/5g 
uint8 NIC_state #Network Interface Card state
uint8 SIM_state #sim card state
uint8 IP_state #ip address state
uint8 single_quality #means 4g/5g single strength
uint8 fpv_service_state   #fpv function service working state
uint8 trans_work_state     #transmit function service working state
uint8 server_connect_state  #Cloud services connnect state
int8 temperature   #temperature of device

uint8 push_stream_enable #push stream function state
uint8 stream_type   # 0:disable 1:only fpv 2:only main video 3:main with fpv 4:fpv with main
uint32 mount_devid  #device id of main stream   
