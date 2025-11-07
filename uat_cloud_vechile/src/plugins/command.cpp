
#include <pluginlib/class_list_macros.hpp>
#include <uat_cloud_vechile/cloud_vehicle_base_plugin.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <std_msgs/Float64.h>
#include "cJSON.h"
#include <cmath>
#include "uat_msg/TD550CommandSwitch.h"
#include "uat_msg/TD550ThirdsubFourframe.h"

namespace uat
{
    namespace plugin
    {
        class CommandPlugin : public cloud_vehicle_bridge::PluginBase
        {
          public:
            CommandPlugin() : cloud_vehicle_bridge::PluginBase() {};
            ~CommandPlugin() {};
            virtual void initialize()
            {
                ROS_INFO("CommandPlugin  initialize");
                _command_switch_client = _nh.serviceClient<uat_msg::TD550CommandSwitch>("/uavros/cmd/command_switch");
                //_timer_call_1s = _nh.createTimer(ros::Duration(1), &CommandPlugin::timer_1s_cb, this, false);
                //third_four_sub = _nh.subscribe<uat_msg::ThirdsubFourframe>("/telemeter_data_thirdsubfourframe", 100, &CommandPlugin::thirdsubfourframe_cb, this);

                _mqtt2ros_command_sub = _nh.subscribe<std_msgs::String>("/uat_cloud_ros/command", 10, &CommandPlugin::mqtt_command_cb, this);
            }

          private:
            void mqtt_command_cb(const std_msgs::String::ConstPtr &msg)
            {
                ROS_INFO("receive command data!!!!!!!!!!!!");
                mqtt_services_cb(msg);
            }

          private:
            void handle_services_command(const std::string& strBid, const std::string& strMethod, const cJSON *jsonData)
            {
                _command_method=strMethod;
                _bid=strBid;

                ROS_INFO("receive command method is %s",strMethod.c_str());
                ROS_INFO("receive command bid is %s",strBid.c_str());

                if("cmd_switch"==strMethod)
                {
                    handle_switch_message(strMethod,strBid,jsonData);
                }
                else if("cmd_switch_550_1200"==strMethod)
                {
                    handle_switch_message(strMethod,strBid,jsonData);
                }
                else if("cmd_switch_1200"==strMethod)
                {
                    handle_switch_message(strMethod,strBid,jsonData);
                }
                else if("longitudinal_pos_remote_adjustment"==strMethod)
                {
                    
                }
                else if("lateral_pos_remote_adjustment"==strMethod)
                {
                    
                }
                else if("alt_remote_adjustment"==strMethod)
                {
                    
                }
                else if("yaw_remote_adjustment"==strMethod)
                {
                    
                }
                else if("longitudinal_speed_remote_adjustment"==strMethod)
                {
                    
                }
                else if("vertical_speed_remote_adjustment"==strMethod)
                {
                    
                }
                else if("lateral_speed_remote_adjustment"==strMethod)
                {
                    
                }
                else if("pos_deviation_injection"==strMethod)
                {
                    
                }
                else if("height_injection"==strMethod)
                {
                    
                }
                else if("magnetic_injection"==strMethod)
                {
                    
                }
                else if("longitudinal_cyclic_control"==strMethod)
                {
                    
                }
                else if("lateral_cyclic_control"==strMethod)
                {
                    
                }
                else if("total_distance_control"==strMethod)
                {
                    
                }
                else if("uav_takeoff_weight_load"==strMethod)
                {
                    
                }
                else if("a_coordinate_load"==strMethod)
                {
                    
                }
                else if("b_coordinate_load"==strMethod)
                {
                    
                }
                else if("c_coordinate_load"==strMethod)
                {
                    
                }
                else if("alternate_point1"==strMethod)
                {
                    
                }
                else if("alternate_point2"==strMethod)
                {
                    
                }
                else if("alternate_point3"==strMethod)
                {
                    
                }
                else if("alternate_point4"==strMethod)
                {
                    
                }
                else if("tail_rotor_control"==strMethod)
                {
                    
                }
                else if("damper_opening_control"==strMethod)
                {
                    
                }
                else if("longitudinal_cyclic_control"==strMethod)
                {
                    
                }
            }

            void handle_switch_message(const std::string& strMethod,const std::string& strBid, const cJSON *jsonData) 
            {
                int switchValue=0;
                if(!getIntValue(jsonData,"value",switchValue))
                {
                    do_need_reply(strMethod, strBid, 0, 1, "switch param error");
                    return;
                }

                uat_msg::TD550CommandSwitch cmd;
                cmd.request.value=switchValue;

                if(_command_switch_client.call(cmd))
                {
                    do_need_reply(strMethod, strBid, 0, 0, "switch success");
                }
                else 
                {
                    do_need_reply(strMethod, strBid, 0, 1, "switch failed");
                }
            }

            void handle_combine_message(const std::string& strMethod,const std::string& strBid, const cJSON *jsonData)
            {
                
            }

            void thirdsubfourframe_cb(const uat_msg::TD550ThirdsubFourframe::ConstPtr &msg)
            {
                ROS_INFO("_last_command_reply is %d",msg->KgReply);

                if((msg->KgReply!=0)&&(msg->KgReply!=_last_command_reply))
                {
                    std::string strReplyMsg=KgOrderReply((unsigned short)msg->KgReply);
                    do_need_reply(_command_method,_bid,1,0,strReplyMsg);

                    ROS_INFO("send KgOrderReply!!!!!!!!!!!!!");
                }

                _last_command_reply=msg->KgReply;
            }

            std::string KgOrderReply(unsigned short reply)
            {
                std::string strReplyText = "";
                unsigned short temp = reply;

                switch (temp)
                {
                case 0x0101:
                    strReplyText =  u8"遥控指令接收成功";
                    break;
                case 0x0102:
                    strReplyText =  u8"遥控指令接收失败";
                    break;
                case 0x0103:
                    strReplyText =  u8"混控指令接收成功";
                    break;
                case 0x0104:
                    strReplyText =  u8"混控指令接收败";
                    break;
                case 0x0105:
                    strReplyText =  u8"程控指令接收成功";
                    break;
                case 0x0106:
                    strReplyText = u8"程控指令接收失败";
                    break;
                case 0x0107:
                    strReplyText = u8"前发开车指令接收成功";
                    break;
                case 0x0108:
                    strReplyText = u8"前发开车指令接收失败";
                    break;
                case 0x0109:
                    strReplyText = u8"前发怠速指令接收成功";
                    break;
                case 0x010A:
                    strReplyText = u8"前发怠速指令接收失败";
                    break;
                case 0x010B:
                    strReplyText = u8"双发暖车指令接收成功";
                    break;
                case 0x010C:
                    strReplyText = u8"双发暖车指令接收失败";
                    break;
                case 0x010D:
                    strReplyText = u8"双发额定指令接收成功";
                    break;
                case 0x010E:
                    strReplyText = u8"双发额定指令接收失败";
                    break;
                case 0x010F:
                    strReplyText = u8"前发停车指令接收成功";
                    break;
                case 0x0110:
                    strReplyText = u8"前发停车指令接收失败";
                    break;
                case 0x016D:
                    strReplyText = u8"一键关车指令接收成功";
                    break;
                case 0x016E:
                    strReplyText = u8"一键关车指令接收失败";
                    break;
                case 0x0111:
                    strReplyText = u8"绝对高度保持与给定指令接收成功";
                    break;
                case 0x0112:
                    strReplyText = u8"绝对高度保持与给定指令接收失败";
                    break;
                case 0x0113:
                    strReplyText = u8"空速保持与给定指令接收成功";
                    break;
                case 0x0114:
                    strReplyText = u8"空速保持与给定指令接收失败";
                    break;
                case 0x0115:
                    strReplyText = u8"相对高度保持与给定指令接收成功";
                    break;
                case 0x0116:
                    strReplyText = u8"相对高度保持与给定指令接收失败";
                    break;
                case 0x0117:
                    strReplyText = u8"地速保持与给定指令接收成功";
                    break;
                case 0x0118:
                    strReplyText = u8"地速保持与给定指令接收失败";
                    break;
                case 0x0119:
                    strReplyText = u8"位置保持与给定指令接收成功";
                    break;
                case 0x011A:
                    strReplyText = u8"位置保持与给定指令接收失败";
                    break;
                case 0x011B:
                    strReplyText = u8"垂直速度保持与给定指令接收成功";
                    break;
                case 0x011C:
                    strReplyText = u8"垂直速度保持与给定指令接收失败";
                    break;
                case 0x011D:
                    strReplyText = u8"航向保持与给定指令接收成功";
                    break;
                case 0x011E:
                    strReplyText = u8"航向保持与给定指令接收失败";
                    break;
                case 0x011F:
                    strReplyText = u8"自动导航指令接收成功";
                    break;
                case 0x0120:
                    strReplyText = u8"自动导航指令接收失败";
                    break;
                case 0x0121:
                    strReplyText = u8"自主起飞指令接收成功";
                    break;
                case 0x0122:
                    strReplyText = u8"自主起飞指令接收失败";
                    break;
                case 0x0123:
                    strReplyText = u8"自主着陆指令接收成功";
                    break;
                case 0x0124:
                    strReplyText = u8"自主着陆指令接收失败";
                    break;
                case 0x0125:
                    strReplyText = u8"自主悬停指令接收成功";
                    break;
                case 0x0126:
                    strReplyText = u8"自主悬停指令接收失败";
                    break;
                case 0x0127:
                    strReplyText = u8"直线归航指令接收成功";
                    break;
                case 0x0128:
                    strReplyText = u8"直线归航指令接收失败";
                    break;
                case 0x0129:
                    strReplyText = u8"原路归航指令接收成功";
                    break;
                case 0x012A:
                    strReplyText = u8"原路归航指令接收失败";
                    break;
                case 0x012B:
                    strReplyText = u8"直线归航2指令接收成功";
                    break;
                case 0x012C:
                    strReplyText = u8"直线归航2指令接收失败";
                    break;
                case 0x012D:
                    strReplyText = u8"断链路应急返航指令接收成功";
                    break;
                case 0x012E:
                    strReplyText = u8"断链路应急返航指令接收失败";
                    break;
                case 0x012F:
                    strReplyText = u8"左盘旋指令接收成功";
                    break;
                case 0x0130:
                    strReplyText = u8"左盘旋指令接收失败";
                    break;
                case 0x0131:
                    strReplyText = u8"右盘旋指令接收成功";
                    break;
                case 0x0132:
                    strReplyText = u8"右盘旋指令接收失败";
                    break;
                case 0x0133:
                    strReplyText = u8"向上过渡指令接收成功";
                    break;
                case 0x0134:
                    strReplyText = u8"向上过渡指令接收失败";
                    break;
                case 0x0135:
                    strReplyText = u8"向下过渡指令接收成功";
                    break;
                case 0x0136:
                    strReplyText = u8"向下过渡指令接收失败";
                    break;
                case 0x0137:
                    strReplyText = u8"自主避障指令接收成功";
                    break;
                case 0x0138:
                    strReplyText = u8"自主避障指令接收失败";
                    break;
                case 0x0139:
                    strReplyText = u8"退出机动指令接收成功";
                    break;
                case 0x013A:
                    strReplyText = u8"退出机动指令接收失败";
                    break;
                case 0x013B:
                    strReplyText = u8"解除高度保持指令接收成功";
                    break;
                case 0x013C:
                    strReplyText = u8"解除高度保持指令接收失败";
                    break;
                case 0x0141:
                    strReplyText = u8"解除定向指令接收成功";
                    break;
                case 0x0142:
                    strReplyText = u8"解除定向指令接收成功";
                    break;
                case 0x013D:
                    strReplyText = u8"解除速度指令接收成功";
                    break;
                case 0x013E:
                    strReplyText = u8"解除速度指令接收成功";
                    break;
                case 0x013F:
                    strReplyText = u8"解除位置指令接收成功";
                    break;
                case 0x0140:
                    strReplyText = u8"解除位置指令接收失败";
                    break;
                case 0x0143:
                    strReplyText = u8"解除导航指令接收成功";
                    break;
                case 0x0144:
                    strReplyText = u8"解除导航指令接收失败";
                    break;
                case 0x0145:
                    strReplyText = u8"前发ECU接通指令接收成功";
                    break;
                case 0x0146:
                    strReplyText = u8"前发ECU接通指令接收失败";
                    break;
                case 0x0147:
                    strReplyText = u8"前发ECU断开指令接收成功";
                    break;
                case 0x0148:
                    strReplyText = u8"前发ECU断开指令接收失败";
                    break;
                case 0x0149:
                    strReplyText = u8"后发ECU接通指令接收成功";
                    break;
                case 0x014A:
                    strReplyText = u8"后发ECU接通指令接收失败";
                    break;
                case 0x014B:
                    strReplyText = u8"后发ECU断开指令接收成功";
                    break;
                case 0x014C:
                    strReplyText = u8"后发ECU断开指令接收失败";
                    break;
                case 0x014D:
                    strReplyText = u8"前发电机并网指令接收成功";
                    break;
                case 0x014E:
                    strReplyText = u8"前发电机并网指令接收失败";
                    break;
                case 0x014F:
                    strReplyText = u8"前发电机退网指令接收成功";
                    break;
                case 0x0150:
                    strReplyText = u8"前发电机退网指令接收失败";
                    break;
                case 0x0151:
                    strReplyText = u8"离合器接合指令接收成功";
                    break;
                case 0x0152:
                    strReplyText = u8"离合器接合指令接收失败";
                    break;
                case 0x0153:
                    strReplyText = u8"离合器分离指令接收成功";
                    break;
                case 0x0154:
                    strReplyText = u8"离合器分离指令接收失败";
                    break;
                case 0x016F:
                    strReplyText = u8"前发油泵接通指令接收成功";
                    break;
                case 0x0170:
                    strReplyText = u8"前发油泵接通指令接收失败";
                    break;
                case 0x0171:
                    strReplyText = u8"前发油泵断开指令接收成功";
                    break;
                case 0x0172:
                    strReplyText = u8"前发油泵断开指令接收失败";
                    break;
                case 0x0155:
                    strReplyText = u8"后发油泵接通指令接收成功";
                    break;
                case 0x0156:
                    strReplyText = u8"后发油泵接通指令接收失败";
                    break;
                case 0x0157:
                    strReplyText = u8"后发油泵断开指令接收成功";
                    break;
                case 0x0158:
                    strReplyText = u8"后发油泵断开指令接收失败";
                    break;
                case 0x0159:
                    strReplyText = u8"后发电机接通指令接收成功";
                    break;
                case 0x015A:
                    strReplyText = u8"后发电机接通指令接收失败";
                    break;
                case 0x015B:
                    strReplyText = u8"后发电机断开指令接收成功";
                    break;
                case 0x015C:
                    strReplyText = u8"后发电机断开指令接收失败";
                    break;
                case 0x0173:
                    strReplyText = u8"水泵接通指令接收成功";
                    break;
                case 0x0174:
                    strReplyText = u8"水泵接通指令接收失败";
                    break;
                case 0x0175:
                    strReplyText = u8"水泵关闭指令接收成功";
                    break;
                case 0x0176:
                    strReplyText = u8"水泵关闭指令接收失败";
                    break;
                case 0x0177:
                    strReplyText = u8"投放开指令接收成功";
                    break;
                case 0x0178:
                    strReplyText = u8"投放开指令接收失败";
                    break;
                case 0x0179:
                    strReplyText = u8"投放关指令接收成功";
                    break;
                case 0x017A:
                    strReplyText = u8"投放关指令接收失败";
                    break;
                case 0x017B:
                    strReplyText = u8"吊舱接通指令接收成功";
                    break;
                case 0x017C:
                    strReplyText = u8"吊舱接通指令接收失败";
                    break;
                case 0x017D:
                    strReplyText = u8"吊舱断开指令接收成功";
                    break;
                case 0x017E:
                    strReplyText = u8"吊舱断开指令接收失败";
                    break;
                case 0x015D:
                    strReplyText = u8"掉转执行接收成功";
                    break;
                case 0x015E:
                    strReplyText = u8"掉转执行接收失败";
                    break;
                case 0x015F:
                    strReplyText = u8"取消指令执行接收成功";
                    break;
                case 0x0160:
                    strReplyText = u8"取消指令执行接收成功";
                    break;
                case 0x0161:
                    strReplyText = u8"纵向位置遥调接收成功";
                    break;
                case 0x0162:
                    strReplyText = u8"纵向位置遥调接收失败";
                    break;
                case 0x0163:
                    strReplyText = u8"横向位置遥调接收成功";
                    break;
                case 0x0164:
                    strReplyText = u8"横向位置遥调接收失败";
                    break;
                case 0x0165:
                    strReplyText = u8"高度遥调接收成功";
                    break;
                case 0x0166:
                    strReplyText = u8"高度遥调接收失败";
                    break;
                case 0x0167:
                    strReplyText = u8"航向遥调接收成功";
                    break;
                case 0x0168:
                    strReplyText = u8"航向遥调接收失败";
                    break;
                case 0x0169:
                    strReplyText = u8"纵向速度遥调接收成功";
                    break;
                case 0x016A:
                    strReplyText = u8"纵向速度遥调接收失败";
                    break;
                case 0x016B:
                    strReplyText = u8"垂速遥调接收成功";
                    break;
                case 0x016C:
                    strReplyText = u8"垂速遥调接收成功";
                    break;
                case 0x0185:
                    strReplyText = u8"发动机散热接通接收成功";
                    break;
                case 0x0186:
                    strReplyText = u8"发动机散热接通接收失败";
                    break;
                case 0x0187:
                    strReplyText = u8"发动机散热断开接收成功";
                    break;
                case 0x0188:
                    strReplyText = u8"发动机散热断开接收失败";
                    break;
                case 0x0189:
                    strReplyText = u8"侧向速度遥调接收成功";
                    break;
                case 0x018A:
                    strReplyText = u8"侧向速度遥调接收成功";
                    break;
                case 0x018B:
                    strReplyText = u8"A点坐标装订接收成功";
                    break;
                case 0x018C:
                    strReplyText = u8"A点坐标装订接收失败";
                    break;
                case 0x018D:
                    strReplyText = u8"B点坐标装订接收成功";
                    break;
                case 0x018E:
                    strReplyText = u8"B点坐标装订接收失败";
                    break;
                case 0x018F:
                    strReplyText = u8"C点坐标装订接收成功";
                    break;
                case 0x0190:
                    strReplyText = u8"C点坐标装订接收失败";
                    break;
                case 0x0191:
                    strReplyText = u8"点号遥调接收成功";
                    break;
                case 0x0192:
                    strReplyText = u8"点号遥调接收失败";
                    break;
                case 0x0193:
                    strReplyText = u8"开启空速保护接收成功";
                    break;
                case 0x0194:
                    strReplyText = u8"开启空速保护接收失败";
                    break;
                case 0x0195:
                    strReplyText = u8"开启空速禁止接收成功";
                    break;
                case 0x0196:
                    strReplyText = u8"开启空速禁止接收失败";
                    break;
                case 0x0203:
                    strReplyText = u8"非失控迫降执行接收成功";
                    break;
                case 0x0204:
                    strReplyText = u8"非失控迫降执行接收失败";
                    break;
                case 0x0205:
                    strReplyText = u8"非失控迫降不执行接收成功";
                    break;
                case 0x0206:
                    strReplyText = u8"非失控迫降不执行接收失败";
                    break;
                case 0x0207:
                    strReplyText = u8"迫降点1装订接收成功";
                    break;
                case 0x0208:
                    strReplyText = u8"迫降点1装订接收失败";
                    break;
                case 0x0209:
                    strReplyText = u8"迫降点2装订接收成功";
                    break;
                case 0x020A:
                    strReplyText = u8"迫降点2装订接收失败";
                    break;
                case 0x020B:
                    strReplyText = u8"迫降点3装订接收成功";

                    break;
                case 0x020C:
                    strReplyText = u8"迫降点3装订接收失败";
                    break;
                case 0x020D:
                    strReplyText = u8"迫降点4装订接收成功";
                    break;
                case 0x020E:
                    strReplyText = u8"迫降点4装订接收失败";
                    break;
                case 0x0246:
                    strReplyText = u8"导航切换指令接收成功";
                    break;
                case 0x0247:
                    strReplyText = u8"导航切换指令接收失败";
                    break;
                case 0x0307:
                    strReplyText = u8"后发开车指令接收成功";
                    break;
                case 0x0308:
                    strReplyText = u8"后发开车指令接收失败";
                    break;
                case 0x0309:
                    strReplyText = u8"后发怠速指令接收成功";
                    break;
                case 0x030A:
                    strReplyText = u8"后发怠速指令接收失败";
                    break;
                case 0x030B:
                    strReplyText = u8"后发暖车指令接收成功";
                    break;
                case 0x030C:
                    strReplyText = u8"后发暖车指令接收失败";
                    break;
                case 0x030D:
                    strReplyText = u8"后发额定指令接收成功";
                    break;
                case 0x030E:
                    strReplyText = u8"后发额定指令接收失败";
                    break;
                case 0x030F:
                    strReplyText = u8"后发停车指令接收成功";
                    break;
                case 0x0310:
                    strReplyText = u8"后发停车指令接收失败";
                    break;
                default:
                    break;
                }

                return strReplyText;
            }

          private:
            ros::ServiceClient _command_switch_client;
            ros::Timer _timer_call_1s;

            std::mutex _data_mutex;

            ros::Publisher _command_pub;

            ros::Subscriber _mqtt2ros_command_sub;
            ros::Subscriber third_four_sub;

            int _last_command_reply;
        };

    } // namespace plugin
} // namespace uat


PLUGINLIB_EXPORT_CLASS(uat::plugin::CommandPlugin,
                       uat::plugin::cloud_vehicle_bridge::PluginBase)
