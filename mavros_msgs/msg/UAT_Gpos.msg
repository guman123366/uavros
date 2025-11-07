std_msgs/Header header

int32 latitude      #units="degE7">latitude
int32 longitude     #units="degE7">longitude
int32 altitude      #units="cm" 
uint16 ground_speed #units="cm/s"
int16[3] ned_vel   #units="cm/s"
uint16[3] pos_std  #units="cm"
uint16[3] vel_std  #units="cm/s"

# <enum name="GPS_FIX_TYPE">
#     <description>Type of GPS fix</description>
#     <entry value="0" name="GPS_FIX_TYPE_NO_GPS">
#     <description>No GPS connected</description>
#     </entry>
#     <entry value="1" name="GPS_FIX_TYPE_NO_FIX">
#     <description>No position information, GPS is connected</description>
#     </entry>
#     <entry value="2" name="GPS_FIX_TYPE_2D_FIX">
#     <description>2D position</description>
#     </entry>
#     <entry value="3" name="GPS_FIX_TYPE_3D_FIX">
#     <description>3D position</description>
#     </entry>
#     <entry value="4" name="GPS_FIX_TYPE_DGPS">
#     <description>DGPS/SBAS aided 3D position</description>
#     </entry>
#     <entry value="5" name="GPS_FIX_TYPE_RTK_FLOAT">
#     <description>RTK float, 3D position</description>
#     </entry>
#     <entry value="6" name="GPS_FIX_TYPE_RTK_FIXED">
#     <description>RTK Fixed, 3D position</description>
#     </entry>
#     <entry value="7" name="GPS_FIX_TYPE_STATIC">
#     <description>Static fixed, typically used for base stations</description>
#     </entry>
#     <entry value="8" name="GPS_FIX_TYPE_PPP">
#     <description>PPP, 3D position.</description>
#     </entry>
# </enum>

uint8 fix_type
uint8 satellites_visible