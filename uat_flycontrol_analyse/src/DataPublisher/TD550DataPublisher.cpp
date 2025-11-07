#include "TD550DataPublisher.h"
#include "uat_msg/TD550FirstsubOneframe.h"
#include "uat_msg/TD550FirstsubTwoframe.h"
#include "uat_msg/TD550FirstsubThreeframe.h"
#include "uat_msg/TD550FirstsubFourframe.h"
#include "uat_msg/TD550SecondsubOneframe.h"
#include "uat_msg/TD550SecondsubTwoframe.h"
#include "uat_msg/TD550SecondsubThreeframe.h"
#include "uat_msg/TD550SecondsubFourframe.h"
#include "uat_msg/TD550ThirdsubOneframe.h"
#include "uat_msg/TD550ThirdsubTwoframe.h"
#include "uat_msg/TD550ThirdsubThreeframe.h"
#include "uat_msg/TD550ThirdsubFourframe.h"

TD550DataPublisher::TD550DataPublisher(ros::NodeHandle &nh)
{
    first_one_pub=nh.advertise<uat_msg::TD550FirstsubOneframe>("/telemeter_data_firstsuboneframe", 100);
	first_two_pub=nh.advertise<uat_msg::TD550FirstsubTwoframe>("/telemeter_data_firstsubtwoframe", 100);
	first_three_pub=nh.advertise<uat_msg::TD550FirstsubThreeframe>("/telemeter_data_firstsubthreeframe", 100);
	first_four_pub=nh.advertise<uat_msg::TD550FirstsubFourframe>("/telemeter_data_firstsubfourframe", 100);
	second_one_pub=nh.advertise<uat_msg::TD550SecondsubOneframe>("/telemeter_data_secondsuboneframe", 100);
	second_two_pub=nh.advertise<uat_msg::TD550SecondsubTwoframe>("/telemeter_data_secondsubtwoframe", 100);
	second_three_pub=nh.advertise<uat_msg::TD550SecondsubThreeframe>("/telemeter_data_secondsubthreeframe", 100);
	second_four_pub=nh.advertise<uat_msg::TD550SecondsubFourframe>("/telemeter_data_secondsubfourframe", 100);
	third_one_pub=nh.advertise<uat_msg::TD550ThirdsubOneframe>("/telemeter_data_thirdsuboneframe", 100);
	third_two_pub=nh.advertise<uat_msg::TD550ThirdsubTwoframe>("/telemeter_data_thirdsubtwoframe", 100);
	third_three_pub=nh.advertise<uat_msg::TD550ThirdsubThreeframe>("/telemeter_data_thirdsubthreeframe", 100);
	third_four_pub=nh.advertise<uat_msg::TD550ThirdsubFourframe>("/telemeter_data_thirdsubfourframe", 100);

}

void TD550DataPublisher::publish(const std::shared_ptr<DataDefineInterface> &data)
{
    // 动态转换到 TD550TelemetryData
    auto td550Data = std::dynamic_pointer_cast<TD550TelemetryData>(data);
    if (!td550Data) {
        ROS_ERROR("TD550DataPublisher: Failed to cast DataDefineInterface to TD550TelemetryData");
        return;
    }

    //第一副帧第一子帧
	uat_msg::TD550FirstsubOneframe first_one_msg;
	first_one_msg.SubCounts=td550Data->m_FirstSubOneFrame.SubCounts;
	first_one_msg.FCVersion=td550Data->m_FirstSubOneFrame.FCVersion;
	first_one_msg.CalibAirSpeed=td550Data->m_FirstSubOneFrame.CalibAirSpeed;
	first_one_msg.TureAirSpeed=td550Data->m_FirstSubOneFrame.TureAirSpeed;
	first_one_msg.AirPressHeight=td550Data->m_FirstSubOneFrame.AirPressHeight;
	first_one_msg.StaticTemp=td550Data->m_FirstSubOneFrame.StaticTemp;
	first_one_msg.TotalTemp=td550Data->m_FirstSubOneFrame.TotalTemp;
	first_one_msg.SmoothAirSpeed=td550Data->m_FirstSubOneFrame.SmoothAirSpeed;
	first_one_msg.AvoidNorthPos1=td550Data->m_FirstSubOneFrame.AvoidNorthPos1;
	first_one_msg.AvoidEastPos1=td550Data->m_FirstSubOneFrame.AvoidEastPos1;
	first_one_msg.AvoidNorthPos2=td550Data->m_FirstSubOneFrame.AvoidNorthPos2;
	first_one_msg.AvoidEastPos2=td550Data->m_FirstSubOneFrame.AvoidEastPos2;
	first_one_msg.AvoidNorthPos3=td550Data->m_FirstSubOneFrame.AvoidNorthPos3;
	first_one_msg.NaviState=td550Data->m_FirstSubOneFrame.NaviState;
	first_one_pub.publish(first_one_msg);

	//第一副帧第二子帧
	uat_msg::TD550FirstsubTwoframe first_two_msg;
	first_two_msg.D1ServoControl=td550Data->m_FirstSubTwoFrame.D1ServoControl;
	first_two_msg.D2ServoControl=td550Data->m_FirstSubTwoFrame.D2ServoControl;
	first_two_msg.D3ServoControl=td550Data->m_FirstSubTwoFrame.D3ServoControl;
	first_two_msg.U1ServoControl=td550Data->m_FirstSubTwoFrame.U1ServoControl;
	first_two_msg.U2ServoControl=td550Data->m_FirstSubTwoFrame.U2ServoControl;
	first_two_msg.U3ServoControl=td550Data->m_FirstSubTwoFrame.U3ServoControl;
	first_two_msg.DamperServoControl=td550Data->m_FirstSubTwoFrame.DamperServoControl;
	first_two_msg.DirServoControl=td550Data->m_FirstSubTwoFrame.DirServoControl;
	first_two_msg.D1ServoPostion=td550Data->m_FirstSubTwoFrame.D1ServoPostion;
	first_two_msg.D2ServoPostion=td550Data->m_FirstSubTwoFrame.D2ServoPostion;
	first_two_msg.D3ServoPostion=td550Data->m_FirstSubTwoFrame.D3ServoPostion;
	first_two_msg.U1ServoPostion=td550Data->m_FirstSubTwoFrame.U1ServoPostion;
	first_two_msg.U2ServoPostion=td550Data->m_FirstSubTwoFrame.U2ServoPostion;
	first_two_msg.U3ServoPostion=td550Data->m_FirstSubTwoFrame.U3ServoPostion;
	first_two_msg.DamperServoPostion=td550Data->m_FirstSubTwoFrame.DamperServoPostion;
	first_two_msg.DirServoPostion=td550Data->m_FirstSubTwoFrame.DirServoPostion;
	first_two_msg.lat=td550Data->m_FirstSubTwoFrame.lat;
	first_two_msg.lon=td550Data->m_FirstSubTwoFrame.lon;
	first_two_pub.publish(first_two_msg);

	//第一副帧第三子帧
	uat_msg::TD550FirstsubThreeframe first_three_msg;
	first_three_msg.AbsolutelyHeight=td550Data->m_FirstSubThreeFrame.AbsolutelyHeight;
	first_three_msg.XSpeed=td550Data->m_FirstSubThreeFrame.XSpeed;
	first_three_msg.YSpeed=td550Data->m_FirstSubThreeFrame.YSpeed;
	first_three_msg.ZSpeed=td550Data->m_FirstSubThreeFrame.ZSpeed;
	first_three_msg.XaSpeed=td550Data->m_FirstSubThreeFrame.XaSpeed;
	first_three_msg.YaSpeed=td550Data->m_FirstSubThreeFrame.YaSpeed;
	first_three_msg.ZaSpeed=td550Data->m_FirstSubThreeFrame.ZaSpeed;
	first_three_msg.RollAngVelocity=td550Data->m_FirstSubThreeFrame.RollAngVelocity;
	first_three_msg.PitchAngVelocity=td550Data->m_FirstSubThreeFrame.PitchAngVelocity;
	first_three_msg.YawAngVelocity=td550Data->m_FirstSubThreeFrame.YawAngVelocity;
	first_three_msg.Yaw=td550Data->m_FirstSubThreeFrame.Yaw;
	first_three_msg.Pitch=td550Data->m_FirstSubThreeFrame.Pitch;
	first_three_msg.Roll=td550Data->m_FirstSubThreeFrame.Roll;
	first_three_msg.EastSpeed=td550Data->m_FirstSubThreeFrame.EastSpeed;
	first_three_msg.NothSpeed=td550Data->m_FirstSubThreeFrame.NothSpeed;
	first_three_msg.RelHeight=td550Data->m_FirstSubThreeFrame.RelHeight;
	first_three_msg.AYB=td550Data->m_FirstSubThreeFrame.AYB;
	first_three_msg.EngineR=td550Data->m_FirstSubThreeFrame.EngineR;
	first_three_msg.MainRotor=td550Data->m_FirstSubThreeFrame.MainRotor;
	first_three_pub.publish(first_three_msg);

	//第一副帧第四子帧
	uat_msg::TD550FirstsubFourframe first_four_msg;
	first_four_msg.SignalSource1=td550Data->m_FirstSubFourFrame.SignalSource1;
	first_four_msg.SignalSource2=td550Data->m_FirstSubFourFrame.SignalSource2;
	first_four_msg.SignalSource3=td550Data->m_FirstSubFourFrame.SignalSource3;
	first_four_msg.SignalSource4=td550Data->m_FirstSubFourFrame.SignalSource4;
	first_four_msg.SignalSource5=td550Data->m_FirstSubFourFrame.SignalSource5;
	first_four_msg.FaultCode=td550Data->m_FirstSubFourFrame.FaultCode;
	first_four_msg.D1ServoMFault=td550Data->m_FirstSubFourFrame.D1ServoMFault;
	first_four_msg.D1ServoBFault=td550Data->m_FirstSubFourFrame.D1ServoBFault;
	first_four_msg.D2ServoMFault=td550Data->m_FirstSubFourFrame.D2ServoMFault;
	first_four_msg.D2ServoBFault=td550Data->m_FirstSubFourFrame.D2ServoBFault;
	first_four_msg.D3ServoMFault=td550Data->m_FirstSubFourFrame.D3ServoMFault;
	first_four_msg.D3ServoBFault=td550Data->m_FirstSubFourFrame.D3ServoBFault;
	first_four_msg.U1ServoMFault=td550Data->m_FirstSubFourFrame.U1ServoMFault;
	first_four_msg.U1ServoBFault=td550Data->m_FirstSubFourFrame.U1ServoBFault;
	first_four_msg.U2ServoMFault=td550Data->m_FirstSubFourFrame.U2ServoMFault;
	first_four_msg.U2ServoBFault=td550Data->m_FirstSubFourFrame.U2ServoBFault;
	first_four_msg.U3ServoMFault=td550Data->m_FirstSubFourFrame.U3ServoMFault;
	first_four_msg.U3ServoBFault=td550Data->m_FirstSubFourFrame.U3ServoBFault;
	first_four_msg.DamperServoMFault=td550Data->m_FirstSubFourFrame.DamperServoMFault;
	first_four_msg.DamperServoBFault=td550Data->m_FirstSubFourFrame.DamperServoBFault;
	first_four_msg.DirServoMFault=td550Data->m_FirstSubFourFrame.DirServoMFault;
	first_four_msg.DirServoBFault=td550Data->m_FirstSubFourFrame.DirServoBFault;
	first_four_msg.D1ServoCurrent=td550Data->m_FirstSubFourFrame.D1ServoCurrent;
	first_four_msg.D2ServoCurrent=td550Data->m_FirstSubFourFrame.D2ServoCurrent;
	first_four_msg.D3ServoCurrent=td550Data->m_FirstSubFourFrame.D3ServoCurrent;
	first_four_msg.U1ServoCurrent=td550Data->m_FirstSubFourFrame.U1ServoCurrent;
	first_four_msg.U2ServoCurrent=td550Data->m_FirstSubFourFrame.U2ServoCurrent;
	first_four_msg.U3ServoCurrent=td550Data->m_FirstSubFourFrame.U3ServoCurrent;
	first_four_msg.DamperServoCurrent=td550Data->m_FirstSubFourFrame.DamperServoCurrent;
	first_four_msg.DirServoCurrent=td550Data->m_FirstSubFourFrame.DirServoCurrent;
	first_four_msg.AvoidEastPos3=td550Data->m_FirstSubFourFrame.AvoidEastPos3;
	first_four_pub.publish(first_four_msg);

	//第二副帧第一子帧
	uat_msg::TD550SecondsubOneframe second_one_msg;
	second_one_msg.InertialState=td550Data->m_SecondSubOneFrame.InertialState;
	second_one_msg.BDPdop=td550Data->m_SecondSubOneFrame.BDPdop;
	second_one_msg.MEMSNum=td550Data->m_SecondSubOneFrame.MEMSNum;
	second_one_msg.UTCTime=td550Data->m_SecondSubOneFrame.UTCTime;
	second_one_msg.FlightState=td550Data->m_SecondSubOneFrame.FlightState;
	second_one_msg.RouteNum=td550Data->m_SecondSubOneFrame.RouteNum;
	second_one_msg.WayPointNum=td550Data->m_SecondSubOneFrame.WayPointNum;
	second_one_msg.WayPointState=td550Data->m_SecondSubOneFrame.WayPointState;
	second_one_msg.NextPointNum=td550Data->m_SecondSubOneFrame.NextPointNum;
	second_one_msg.NextPointState=td550Data->m_SecondSubOneFrame.NextPointState;
	second_one_msg.HeadingControl=td550Data->m_SecondSubOneFrame.HeadingControl;
	second_one_msg.TrackError=td550Data->m_SecondSubOneFrame.TrackError;
	second_one_msg.SetoverDis=td550Data->m_SecondSubOneFrame.SetoverDis;
	second_one_msg.MainNavNum=td550Data->m_SecondSubOneFrame.MainNavNum;
	second_one_msg.FlushingDis=td550Data->m_SecondSubOneFrame.FlushingDis;
	second_one_msg.FlushingTime=td550Data->m_SecondSubOneFrame.FlushingTime;
	second_one_msg.MemsWarningState=td550Data->m_SecondSubOneFrame.MemsWarningState;
	second_one_pub.publish(second_one_msg);

	//第二副帧第二子帧
	uat_msg::TD550SecondsubTwoframe second_two_msg;
	second_two_msg.GSB=td550Data->m_SecondSubTwoFrame.GSB;
	second_two_msg.AvoidTimeIndex=td550Data->m_SecondSubTwoFrame.AvoidTimeIndex;
	second_two_msg.AvoidTaskState=td550Data->m_SecondSubTwoFrame.AvoidTaskState;
	second_two_msg.AvoidNorthSpeed=td550Data->m_SecondSubTwoFrame.AvoidNorthSpeed;
	second_two_msg.AvoidEastSpeed=td550Data->m_SecondSubTwoFrame.AvoidEastSpeed;
	second_two_msg.AcoidFCState=td550Data->m_SecondSubTwoFrame.AcoidFCState;
	second_two_msg.AvoidFCOrder=td550Data->m_SecondSubTwoFrame.AvoidFCOrder;
	second_two_msg.ctc=td550Data->m_SecondSubTwoFrame.ctc;
	second_two_msg.b1c=td550Data->m_SecondSubTwoFrame.b1c;
	second_two_msg.a1c=td550Data->m_SecondSubTwoFrame.a1c;
	second_two_msg.dtc=td550Data->m_SecondSubTwoFrame.dtc;
	second_two_msg.chk=td550Data->m_SecondSubTwoFrame.chk;
	second_two_msg.RouteSpeed=td550Data->m_SecondSubTwoFrame.RouteSpeed;
	second_two_msg.RouteVPostion=td550Data->m_SecondSubTwoFrame.RouteVPostion;
	second_two_msg.RouteHPostion=td550Data->m_SecondSubTwoFrame.RouteHPostion;
	second_two_msg.RouteYPostion=td550Data->m_SecondSubTwoFrame.RouteYPostion;
	second_two_msg.RouteHeight=td550Data->m_SecondSubTwoFrame.RouteHeight;
	second_two_msg.RouteVSpeed=td550Data->m_SecondSubTwoFrame.RouteVSpeed;
	second_two_msg.RouteSideSpeed=td550Data->m_SecondSubTwoFrame.RouteSideSpeed;
	second_two_msg.NaviState=td550Data->m_SecondSubTwoFrame.NaviState;
	second_two_pub.publish(second_two_msg);

	//第二副帧第三子帧
	uat_msg::TD550SecondsubThreeframe second_three_msg;
	second_three_msg.FCCPower=td550Data->m_SecondSubThreeFrame.FCCPower;
	second_three_msg.FCCState=td550Data->m_SecondSubThreeFrame.FCCState;
	second_three_msg.FCCTemp=td550Data->m_SecondSubThreeFrame.FCCTemp;
	second_three_msg.ServoTemp=td550Data->m_SecondSubThreeFrame.ServoTemp;
	second_three_msg.FCCError=td550Data->m_SecondSubThreeFrame.FCCError;
	second_three_msg.FCCLevel=td550Data->m_SecondSubThreeFrame.FCCLevel;
	second_three_msg.EngineError=td550Data->m_SecondSubThreeFrame.EngineError;
	second_three_msg.CollectionBoxState=td550Data->m_SecondSubThreeFrame.CollectionBoxState;
	second_three_msg.ForceLandPointIndex=td550Data->m_SecondSubThreeFrame.ForceLandPointIndex;
	second_three_msg.ControlMode=td550Data->m_SecondSubThreeFrame.ControlMode;
	second_three_msg.ModeUsed=td550Data->m_SecondSubThreeFrame.ModeUsed;
	second_three_msg.ManeuMode=td550Data->m_SecondSubThreeFrame.ManeuMode;
	second_three_msg.EngineMode=td550Data->m_SecondSubThreeFrame.EngineMode;
	second_three_msg.Startup=td550Data->m_SecondSubThreeFrame.Startup;
	second_three_msg.Closedown=td550Data->m_SecondSubThreeFrame.Closedown;
	second_three_msg.PSI_DELTA=td550Data->m_SecondSubThreeFrame.PSI_DELTA;
	second_three_msg.DIS_XY=td550Data->m_SecondSubThreeFrame.DIS_XY;
	second_three_msg.DIS_X=td550Data->m_SecondSubThreeFrame.DIS_X;
	second_three_msg.BackupNaviError1=td550Data->m_SecondSubThreeFrame.BackupNaviError1;
	second_three_pub.publish(second_three_msg);

	//第二副帧第四子帧
	uat_msg::TD550SecondsubFourframe second_four_msg;
	second_four_msg.DIS_Y=td550Data->m_SecondSubFourFrame.DIS_Y;
	second_four_msg.takeoffHeight=td550Data->m_SecondSubFourFrame.takeoffHeight;
	second_four_msg.takeoffWeight=td550Data->m_SecondSubFourFrame.takeoffWeight;
	second_four_msg.NowWeight=td550Data->m_SecondSubFourFrame.NowWeight;
	second_four_msg.AvoidPosX=td550Data->m_SecondSubFourFrame.AvoidPosX;
	second_four_msg.AvoidPosY=td550Data->m_SecondSubFourFrame.AvoidPosY;
	second_four_msg.EquipmentState=td550Data->m_SecondSubFourFrame.EquipmentState;
	second_four_msg.EquipmentAlarm1=td550Data->m_SecondSubFourFrame.EquipmentAlarm1;
	second_four_msg.EquipmentAlarm2=td550Data->m_SecondSubFourFrame.EquipmentAlarm2;
	second_four_msg.EquipmentAlarm3=td550Data->m_SecondSubFourFrame.EquipmentAlarm3;
	second_four_msg.EquipmentAlarm4=td550Data->m_SecondSubFourFrame.EquipmentAlarm4;
	second_four_msg.FlightTime=td550Data->m_SecondSubFourFrame.FlightTime;
	second_four_msg.FlightSurTime=td550Data->m_SecondSubFourFrame.FlightSurTime;
	second_four_msg.WaterFan=td550Data->m_SecondSubFourFrame.WaterFan;
	second_four_msg.MiddleFan=td550Data->m_SecondSubFourFrame.MiddleFan;
	second_four_msg.BackupNaviError2=td550Data->m_SecondSubFourFrame.BackupNaviError2;
	second_four_msg.MidFanCurrent=td550Data->m_SecondSubFourFrame.MidFanCurrent;
	second_four_msg.WaterFacCurrent=td550Data->m_SecondSubFourFrame.WaterFacCurrent;
	second_four_msg.PowerBox7V=td550Data->m_SecondSubFourFrame.PowerBox7V;
	second_four_msg.GeneratorCurrent=td550Data->m_SecondSubFourFrame.GeneratorCurrent;
	second_four_msg.RadioHeight=td550Data->m_SecondSubFourFrame.RadioHeight;
	second_four_msg.RadioSmoothAlt=td550Data->m_SecondSubFourFrame.RadioSmoothAlt;
	second_four_msg.BackupNaviError3=td550Data->m_SecondSubFourFrame.BackupNaviError3;
	second_four_msg.Power12V=td550Data->m_SecondSubFourFrame.Power12V;
	second_four_msg.Power24V=td550Data->m_SecondSubFourFrame.Power24V;
	second_four_pub.publish(second_four_msg);

	//第三副帧第一子帧
	uat_msg::TD550ThirdsubOneframe third_one_msg;
	third_one_msg.DisperseIn1=td550Data->m_ThirdSubOneFrame.DisperseIn1;
	third_one_msg.DisperseOut2=td550Data->m_ThirdSubOneFrame.DisperseOut2;
	third_one_msg.DisperseOutRe2=td550Data->m_ThirdSubOneFrame.DisperseOutRe2;
	third_one_msg.Battery=td550Data->m_ThirdSubOneFrame.Battery;
	third_one_msg.MemsTemp=td550Data->m_ThirdSubOneFrame.MemsTemp;
	third_one_msg.OpenK2=td550Data->m_ThirdSubOneFrame.OpenK2;
	third_one_msg.OilPressure=td550Data->m_ThirdSubOneFrame.OilPressure;
	third_one_msg.OilVolume=td550Data->m_ThirdSubOneFrame.OilVolume;
	third_one_msg.OpenK4=td550Data->m_ThirdSubOneFrame.OpenK4;
	third_one_msg.RetarderPressure=td550Data->m_ThirdSubOneFrame.RetarderPressure;
	third_one_msg.RetarderTemp=td550Data->m_ThirdSubOneFrame.RetarderTemp;
	third_one_msg.OpenK1=td550Data->m_ThirdSubOneFrame.OpenK1;
	third_one_msg.Tension=td550Data->m_ThirdSubOneFrame.Tension;
	third_one_pub.publish(third_one_msg);

	//第三副帧第二子帧
	uat_msg::TD550ThirdsubTwoframe third_two_msg;
	third_two_msg.OpenK3=td550Data->m_ThirdSubTwoFrame.OpenK3;
	third_two_msg.Motortemp2=td550Data->m_ThirdSubTwoFrame.Motortemp2;
	third_two_msg.ROiltemp=td550Data->m_ThirdSubTwoFrame.ROiltemp;
	third_two_msg.ROilPress=td550Data->m_ThirdSubTwoFrame.ROilPress;
	third_two_msg.OilPress=td550Data->m_ThirdSubTwoFrame.OilPress;
	third_two_msg.Oil=td550Data->m_ThirdSubTwoFrame.Oil;
	third_two_msg.OilRight=td550Data->m_ThirdSubTwoFrame.OilRight;
	third_two_msg.EngineRPM145=td550Data->m_ThirdSubTwoFrame.EngineRPM145;
	third_two_msg.ThrottlePos145=td550Data->m_ThirdSubTwoFrame.ThrottlePos145;
	third_two_msg.EngineOilPre145=td550Data->m_ThirdSubTwoFrame.EngineOilPre145;
	third_two_msg.EngineOilTemp145=td550Data->m_ThirdSubTwoFrame.EngineOilTemp145;
	third_two_msg.ExhaustTemp1=td550Data->m_ThirdSubTwoFrame.ExhaustTemp1;
	third_two_msg.ExhaustTemp2=td550Data->m_ThirdSubTwoFrame.ExhaustTemp2;
	third_two_msg.ExhaustTemp3=td550Data->m_ThirdSubTwoFrame.ExhaustTemp3;
	third_two_msg.ExhaustTemp4=td550Data->m_ThirdSubTwoFrame.ExhaustTemp4;
	third_two_msg.IntakePressure=td550Data->m_ThirdSubTwoFrame.IntakePressure;
	third_two_msg.TurboPressure=td550Data->m_ThirdSubTwoFrame.TurboPressure;
	third_two_msg.CoolantTemp=td550Data->m_ThirdSubTwoFrame.CoolantTemp;
	third_two_msg.EngineOilPre=td550Data->m_ThirdSubTwoFrame.EngineOilPre;
	third_two_pub.publish(third_two_msg);

	//第三副帧第三子帧
	uat_msg::TD550ThirdsubThreeframe third_three_msg;
	third_three_msg.EngineOilTemp=td550Data->m_ThirdSubThreeFrame.EngineOilTemp;
	third_three_msg.DamperPostion=td550Data->m_ThirdSubThreeFrame.DamperPostion;
	third_three_msg.InairTemp=td550Data->m_ThirdSubThreeFrame.InairTemp;
	third_three_msg.InairPre=td550Data->m_ThirdSubThreeFrame.InairPre;
	third_three_msg.OilUsed=td550Data->m_ThirdSubThreeFrame.OilUsed;
	third_three_msg.OutAirPostion=td550Data->m_ThirdSubThreeFrame.OutAirPostion;
	third_three_msg.EngineRAM=td550Data->m_ThirdSubThreeFrame.EngineRAM;
	third_three_msg.EnineInAirTemp=td550Data->m_ThirdSubThreeFrame.EnineInAirTemp;
	third_three_msg.EcuAbus=td550Data->m_ThirdSubThreeFrame.EcuAbus;
	third_three_msg.EcuBbus=td550Data->m_ThirdSubThreeFrame.EcuBbus;
	third_three_msg.EngineTime=td550Data->m_ThirdSubThreeFrame.EngineTime;
	third_three_msg.OilPreDiff=td550Data->m_ThirdSubThreeFrame.OilPreDiff;
	third_three_msg.StableBoxPre=td550Data->m_ThirdSubThreeFrame.StableBoxPre;
	third_three_msg.StableBoxGoalPre=td550Data->m_ThirdSubThreeFrame.StableBoxGoalPre;
	third_three_msg.StableBoxTemp=td550Data->m_ThirdSubThreeFrame.StableBoxTemp;
	third_three_msg.CoolingTempMax=td550Data->m_ThirdSubThreeFrame.CoolingTempMax;
	third_three_msg.CoolingTempMin=td550Data->m_ThirdSubThreeFrame.CoolingTempMin;
	third_three_msg.ExhaustTempmax=td550Data->m_ThirdSubThreeFrame.ExhaustTempmax;
	third_three_msg.ExhaustTempmin=td550Data->m_ThirdSubThreeFrame.ExhaustTempmin;
	third_three_pub.publish(third_three_msg);

	//第三副帧第四子帧
	uat_msg::TD550ThirdsubFourframe third_four_msg;
	third_four_msg.KgReply=td550Data->m_ThirdSubFourFrame.KgReply;
	third_four_msg.YtReply=td550Data->m_ThirdSubFourFrame.YtReply;
	third_four_msg.YtHdReply=td550Data->m_ThirdSubFourFrame.YtHdReply;
	third_four_msg.B2YReply=td550Data->m_ThirdSubFourFrame.B2YReply;
	third_four_msg.B2XReply=td550Data->m_ThirdSubFourFrame.B2XReply;
	third_four_msg.B1YReply=td550Data->m_ThirdSubFourFrame.B1YReply;
	third_four_msg.B1XReply=td550Data->m_ThirdSubFourFrame.B1XReply;
	third_four_msg.DamperReply=td550Data->m_ThirdSubFourFrame.DamperReply;
	third_four_msg.byte11Reply=td550Data->m_ThirdSubFourFrame.byte11Reply;
	third_four_msg.byte12Reply=td550Data->m_ThirdSubFourFrame.byte12Reply;
	third_four_msg.byte13Reply=td550Data->m_ThirdSubFourFrame.byte13Reply;
	third_four_msg.byte14Reply=td550Data->m_ThirdSubFourFrame.byte14Reply;
	third_four_msg.byte15Reply=td550Data->m_ThirdSubFourFrame.byte15Reply;
	third_four_msg.byte16Reply=td550Data->m_ThirdSubFourFrame.byte16Reply;
	third_four_msg.byte17Reply=td550Data->m_ThirdSubFourFrame.byte17Reply;
	third_four_msg.byte18Reply=td550Data->m_ThirdSubFourFrame.byte18Reply;
	third_four_msg.byte19Reply=td550Data->m_ThirdSubFourFrame.byte19Reply;
	third_four_msg.byte20Reply=td550Data->m_ThirdSubFourFrame.byte20Reply;
	third_four_msg.byte21Reply=td550Data->m_ThirdSubFourFrame.byte21Reply;
	third_four_msg.byte22Reply=td550Data->m_ThirdSubFourFrame.byte22Reply;
	third_four_msg.byte23Reply=td550Data->m_ThirdSubFourFrame.byte23Reply;
	third_four_msg.byte24Reply=td550Data->m_ThirdSubFourFrame.byte24Reply;
	third_four_msg.byte25Reply=td550Data->m_ThirdSubFourFrame.byte25Reply;
	third_four_msg.byte26Reply=td550Data->m_ThirdSubFourFrame.byte26Reply;
	third_four_msg.byte27Reply=td550Data->m_ThirdSubFourFrame.byte27Reply;
	third_four_msg.LinkTest=td550Data->m_ThirdSubFourFrame.LinkTest;
	third_four_msg.EngineAirPre=td550Data->m_ThirdSubFourFrame.EngineAirPre;
	third_four_msg.TCUPower=td550Data->m_ThirdSubFourFrame.TCUPower;
	third_four_msg.ServoPos=td550Data->m_ThirdSubFourFrame.ServoPos;
	third_four_msg.ServoGoalPos=td550Data->m_ThirdSubFourFrame.ServoGoalPos;
	third_four_msg.NaviPDOP=td550Data->m_ThirdSubFourFrame.NaviPDOP;
	third_four_msg.B2YControl=td550Data->m_ThirdSubFourFrame.B2YControl;
	third_four_msg.B2XControl=td550Data->m_ThirdSubFourFrame.B2XControl;
	third_four_msg.B1YControl=td550Data->m_ThirdSubFourFrame.B1YControl;
	third_four_msg.B1XControl=td550Data->m_ThirdSubFourFrame.B1XControl;
	third_four_msg.FUTABAState=td550Data->m_ThirdSubFourFrame.FUTABAState;
	third_four_pub.publish(third_four_msg);
}
