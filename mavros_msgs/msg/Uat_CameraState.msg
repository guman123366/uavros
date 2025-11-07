std_msgs/Header header

uint32 devid
uint8 visible_state #bit[0]:output enable/disable 
uint8 ir_state      #bit[0]:output enable/disable bit[1:7]:enum IR_PALETTES
uint8 laser_state   #bit[0]:output enable/disable 
uint16 visible_zoom #visible zoom number * 10
uint16 ir_zoom         #ir zoom number * 10
uint16 laser_distance #distance unit:m
uint8 camera_state #bit[0]:recording bit[1:3] stream_type </field>
int32 target_lon_deg_1e7
int32 target_lat_deg_1e7