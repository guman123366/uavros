std_msgs/Header header

int8 tempratue
uint8 cpuload
bool ofb_unix_valid

uint32 radar_status
uint32 vision_status
uint32 offb_status
uint32 sensor_status
uint16 uat_status

uint32 RADAR_BIT_FRONT_DISCONNECT = 1
uint32 RADAR_BIT_BACK_DISCONNECT = 2
uint32 RADAR_BIT_BOT_DISCONNECT = 4

uint32 RADAR_BIT_FRONT_DATA_ABNORMAL = 65536  # 0x10000
uint32 RADAR_BIT_BACK_DATA_ABNORMAL = 131072 # 0x20000
uint32 RADAR_BIT_BOT_DATA_ABNORMAL = 262144 # 0x40000