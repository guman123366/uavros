std_msgs/Header header

uint8 TYPE_APRILTAG = 1
uint8 TYPE_IRLOCK = 2
# uint8 sensor_type

# uint8 ERROR_SENSOR_PRESENT = 1  #0x01
# uint8 ERROR_SENSOR_HEALTH = 1   #0x02
# uint8 state #sensor state

int16 theta_x       #theta is the angle between target and the vehicle along the vehicle body x-axis
int16 theta_y       #theta is the angle between target and the vehicle along the vehicle body y-axis
uint8 dis_vaild     #true if the distance offset is valid
int16 dist_x        #the distance between target and the vehicle along the vehicle body x-axis
int16 dist_y        #the distance between target and the vehicle along the vehicle body y-axis
int16 dist_z        #the distance between target and the vehicle along the vehicle body y-axis
int16 offset_yaw    #Euler yaw angle between vehicle head relative to tag positive direction