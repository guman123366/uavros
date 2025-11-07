std_msgs/Header header

uint8 TYPE_VISION = 1
uint8 TYPE_opticflow = 2
uint8 sensor_type

# uint8 ERROR_SENSOR_PRESENT = 1  #0x01
# uint8 ERROR_SENSOR_HEALTH = 1   #0x02
uint8 state #sensor state

float32 pixel_flow_x_integral	# accumulated optical flow in radians where a positive value is produced by a RH rotation about the X body axis
float32 pixel_flow_y_integral	# accumulated optical flow in radians where a positive value is produced by a RH rotation about the Y body axis
float32 gyro_x_rate_integral	# accumulated gyro value in radians where a positive value is produced by a RH rotation about the X body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
float32 gyro_y_rate_integral	# accumulated gyro value in radians where a positive value is produced by a RH rotation about the Y body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
float32 gyro_z_rate_integral	# accumulated gyro value in radians where a positive value is produced by a RH rotation about the Z body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
uint8 quality	# Average of quality of accumulated frames, 0: bad quality, 255: maximum quality

float32 body_x_vel_m
float32 body_y_vel_m
float32 ground_distance_m	# Altitude / distance to ground in meters