std_msgs/Header header

uint32 devid
uint16 valid_type_mask #true if target valid

uint8 VALID_MODE = 0
uint8 VALID_ROLL = 1	
uint8 VALID_PITCH = 2
uint8 VALID_YAW = 3
uint8 VALID_ROLL_RATE = 4	
uint8 VALID_PITCH_RATE = 5
uint8 VALID_YAW_RATE = 6
uint8 VALID_ZOOM = 7
uint8 VALID_ZOOM_RATE = 8

uint8 mode
uint8 MODE_FIX_YAW = 1
uint8 MODE_FOLLOW_YAW = 2
uint8 MODE_TARGET_TRACK = 3

float32 roll
float32 pitch
float32 yaw
float32 roll_rate
float32 pitch_rate
float32 yaw_rate
float32 zoom
float32 zoom_rate