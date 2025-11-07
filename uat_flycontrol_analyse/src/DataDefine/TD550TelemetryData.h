#pragma once
/*
	TD550数据定义
*/

#include "DataDefineInterface.h"

typedef struct FirstsubOneframe
{
	FirstsubOneframe()
	{
		SubCounts = 0;
		FCVersion = 0;
		CalibAirSpeed = 0;
		TureAirSpeed = 0;
		AirPressHeight = 0;
		StaticTemp = 0;
		TotalTemp = 0;
		SmoothAirSpeed = 0;
		AvoidNorthPos1 = 0;
		AvoidEastPos1 = 0;
		AvoidEastPos1 = 0;
	}
	double				SubCounts;//С֡������
	unsigned short      FCVersion;//�ɿذ汾��
	double              CalibAirSpeed;//У׼����
	double              TureAirSpeed;//�����
	double              AirPressHeight;//��ѹ��
	double              StaticTemp;//��������
	double              TotalTemp;//��������
	double              SmoothAirSpeed;//У׼���٣�ƽ��������
	double				AvoidNorthPos1;//�ϰ���---1�������λ��x
	double				AvoidEastPos1;//�ϰ���---1�������λ��y
	double				AvoidNorthPos2;//�ϰ���---2�������λ��x
	unsigned char		NaviState;		//�ߵ�״̬��
	double				AvoidEastPos2;//�ϰ���---2�������λ��y
	double				AvoidNorthPos3;//�ϰ���---3�������λ��x
}*pFirstsubOneframe;

typedef struct FirstsubTwoframe
{
	FirstsubTwoframe()
	{
		D1ServoControl = 0;
		D2ServoControl = 0;
		D3ServoControl = 0;
		U1ServoControl = 0;
		U2ServoControl = 0;
		U3ServoControl = 0;
		DamperServoControl = 0;
		DirServoControl = 0;
		D1ServoPostion = 0;
		D2ServoPostion = 0;
		D3ServoPostion = 0;
		U1ServoPostion = 0;
		U2ServoPostion = 0;
		U3ServoPostion = 0;
		DamperServoPostion = 0;
		DirServoPostion = 0;
		lat = 0;
		lon = 0;
	}
	double              D1ServoControl;//D1���λ�ÿ�����ָ��
	double              D2ServoControl;//D2���λ�ÿ�����ָ��
	double              D3ServoControl;//D3���λ�ÿ�����ָ��
	double              U1ServoControl;//U1���λ�ÿ�����ָ��
	double              U2ServoControl;//U2���λ�ÿ�����ָ��
	double              U3ServoControl;//U3���λ�ÿ�����ָ��
	double              DamperServoControl;//���Ŷ��λ�ÿ�����ָ��
	double              DirServoControl;//������λ�ÿ�����ָ��
	double              D1ServoPostion;//D1���λ�÷���
	double              D2ServoPostion;//D2���λ�÷���
	double              D3ServoPostion;//D3���λ�÷���
	double              U1ServoPostion;//u1���λ�÷���
	double              U2ServoPostion;//u2���λ�÷���
	double              U3ServoPostion;//u3���λ�÷���
	double              DamperServoPostion;//���Ŷ��λ�÷���
	double              DirServoPostion;//������λ�÷���
	double              lat;//γ��
	double              lon;//����
}*pFirstsubTwoframe;

typedef struct FirstsubThreeframe
{
	FirstsubThreeframe()
	{
		AbsolutelyHeight = 0;
		XSpeed = 0;
		YSpeed = 0;
		ZSpeed = 0;
		XaSpeed = 0;
		YaSpeed = 0;
		ZaSpeed = 0;
		RollAngVelocity = 0;
		PitchAngVelocity = 0;
		YawAngVelocity = 0;
		Yaw = 0;
		Pitch = 0;
		Roll = 0;
		EastSpeed = 0;
		NothSpeed = 0;
		RelHeight = 0;
		AYB = 0;
		EngineR = 0;
		MainRotor = 0;
	}
	double              AbsolutelyHeight;//���Ը߶�
	double              XSpeed;//�������;
	double              YSpeed;//�������;
	double              ZSpeed;//�������;
	double              XaSpeed;//������ٶ�;
	double              YaSpeed;//������ٶ�;
	double              ZaSpeed;//������ٶ�;
	double              RollAngVelocity;//��ת������
	double              PitchAngVelocity;//����������
	double              YawAngVelocity;//ƫ��������
	double              Yaw;//ƫ��
	double              Pitch;//����
	double              Roll;//��ת
	double              EastSpeed;//�����ٶ�
	double              NothSpeed;//�����ٶ�
	double              RelHeight;//��Ը߶�
	double              AYB;//�����ۺϼ��ٶ�
	double              EngineR;//ECUA������ת��
	double              MainRotor;//����ת��
}*pFirstsubThreeframe;

typedef struct FirstsubFourframe
{
	FirstsubFourframe()
	{
		SignalSource1 = 0;
		SignalSource2 = 0;
		SignalSource3 = 0;
		SignalSource4 = 0;
		SignalSource5 = 0;
		FaultCode = 0;
		D1ServoMFault = 0;
		D1ServoBFault = 0;
		D2ServoMFault = 0;
		D2ServoBFault = 0;
		D3ServoMFault = 0;
		D3ServoBFault = 0;
		U1ServoMFault = 0;
		U1ServoBFault = 0;
		U2ServoMFault = 0;
		U2ServoBFault = 0;
		U3ServoMFault = 0;
		U3ServoBFault = 0;
		DamperServoMFault = 0;
		DamperServoBFault = 0;
		DirServoMFault = 0;
		DirServoBFault = 0;
		D1ServoCurrent = 0;
		D2ServoCurrent = 0;
		D3ServoCurrent = 0;
		U1ServoCurrent = 0;
		U2ServoCurrent = 0;
		U3ServoCurrent = 0;
		DamperServoCurrent = 0;
		DirServoCurrent = 0;
		AvoidEastPos3 = 0;
	}
	unsigned char       SignalSource1;//�ź�Դѡ����
	unsigned char       SignalSource2;//�ź�Դѡ����
	unsigned char       SignalSource3;//�ź�Դѡ����
	unsigned char       SignalSource4;//�ź�Դѡ����
	unsigned char       SignalSource5;//�ź�Դѡ����
	unsigned int        FaultCode;//�����ۺ���
	unsigned char       D1ServoMFault;//D1��������״̬
	unsigned char       D1ServoBFault;//D1��������״̬
	unsigned char       D2ServoMFault;//D2��������״̬
	unsigned char       D2ServoBFault;//D2��������״̬
	unsigned char       D3ServoMFault;//D3��������״̬
	unsigned char       D3ServoBFault;//D3��������״̬
	unsigned char       U1ServoMFault;//U1��������״̬
	unsigned char       U1ServoBFault;//U1��������״̬
	unsigned char       U2ServoMFault;//U2��������״̬
	unsigned char       U2ServoBFault;//U2��������״̬
	unsigned char       U3ServoMFault;//U3��������״̬
	unsigned char       U3ServoBFault;//U3��������״̬
	unsigned char       DamperServoMFault;//���Ŷ�������״̬
	unsigned char       DamperServoBFault;//���Ŷ�������״̬
	unsigned char       DirServoMFault;//�����������״̬
	unsigned char       DirServoBFault;//�����������״̬
	double              D1ServoCurrent;//D1�������
	double              D2ServoCurrent;//D2�������
	double              D3ServoCurrent;//D3�������
	double              U1ServoCurrent;//u1�������
	double              U2ServoCurrent;//u2�������
	double              U3ServoCurrent;//u3�������
	double              DamperServoCurrent;//���Ŷ������
	double              DirServoCurrent;//����������
	double				AvoidEastPos3;//�ϰ���---3�������λ��y
}*pFirstsubFourframe;

typedef struct SecondsubOneframe
{
	SecondsubOneframe()
	{
		InertialState = 0;
		BDPdop = 0;
		MEMSNum = 0;
		UTCTime = 0;
		FlightState = 0;
		RouteNum = 0;
		WayPointNum = 0;
		WayPointState = 0;
		NextPointNum = 0;
		NextPointState = 0;
		HeadingControl = 0;
		TrackError = 0;
		SetoverDis = 0;
		MainNavNum = 0;
		FlushingDis = 0;
		FlushingTime = 0;
	}
	unsigned char       InertialState;//�ջ����˹ߵ�״̬��
	double              BDPdop;//����PDOPֵ
	int                 MEMSNum;//�������ǿ�����MEMS��
	double              UTCTime;//UTCʱ��
	unsigned char       FlightState;//���н׶�״̬��
	int                 RouteNum;//��ǰ���߱��
	int                 WayPointNum;//��ǰ������
	unsigned char       WayPointState;//��ǰ����������
	int                 NextPointNum;//��һ��������
	unsigned char       NextPointState;//��һ������������
	double              HeadingControl;//�������ź�
	double              TrackError;//��������
	int                 SetoverDis;//��ƫ���ź�
	int                 MainNavNum;//���ߵ����ǿ���
	int                 FlushingDis;//���ɾ��ź�
	int                 FlushingTime;//����ʱ��
	int					MemsWarningState;	//MEMS���ߵ�״̬��
}*pSecondsubOneframe;

typedef struct SecondsubTwoframe
{
	SecondsubTwoframe()
	{
		GSB = 0;
		AvoidTimeIndex = 0;
		AvoidTaskState = 0;
		AvoidNorthSpeed = 0;
		AvoidEastSpeed = 0;
		AcoidFCState = 0;
		AvoidFCOrder = 0;
		ctc = 0;
		b1c = 0;
		a1c = 0;
		dtc = 0;
		chk = 0;
		RouteSpeed = 0;
		RouteVPostion = 0;
		RouteHPostion = 0;
		RouteYPostion = 0;
		RouteHeight = 0;
		RouteVSpeed = 0;
		RouteSideSpeed = 0;
		NaviState = 0;
	}
	double              GSB;//Ԥ����б��GSB
	unsigned short		AvoidTimeIndex;//�ϰ���ʱ���
	unsigned char		AvoidTaskState;//�ϰ�������ִ��״̬
	double				AvoidNorthSpeed;//�ϰ��������ٶȲο�
	double				AvoidEastSpeed;//�ϰ��������ٶȲο�
	unsigned char		AcoidFCState;//�ɿر���״̬
	unsigned char		AvoidFCOrder;//�ɿر�������
	double              ctc;//�ܾཬ���
	double              b1c;//���������
	double              a1c;//��������
	double              dtc;//���򽬾��
	double              chk;//���Ŷ������
	double              RouteSpeed;//�����ٶȸ�����׼ֵ
	double              RouteVPostion;//����λ�ø�����׼ֵ
	double              RouteHPostion;//����λ�ø�����׼ֵ
	double              RouteYPostion;//����Ǹ�����׼ֵ
	double              RouteHeight;//�߶ȸ�����׼ֵ
	double              RouteVSpeed;//��ֱ�ٶȸ�����׼ֵ
	double				RouteSideSpeed;//�����ٶȸ�����׼ֵ
	unsigned char		NaviState;//�������ջ�״̬��
}*pSecondsubTwoframe;

typedef struct SecondsubThreeframe
{
	SecondsubThreeframe()
	{
		FCCPower = 0;
		FCCState = 0;
		FCCTemp = 0;
		ServoTemp = 0;
		FCCError = 0;
		FCCLevel = 0;
		EngineError = 0;
		CollectionBoxState = 0;
		ControlMode = 0;
		ModeUsed = 0;
		ManeuMode = 0;
		EngineMode = 0;
		Startup = 0;
		Closedown = 0;
		PSI_DELTA = 0;
		DIS_XY = 0;
		DIS_X = 0;
	}
	double              FCCPower;//�ɿؼ����28Vͨ����Դ
	unsigned short      FCCState;//�ɿؼ��������״̬��
	double              FCCTemp;//�ɿؼ�����¶�
	int                 ServoTemp;//�ŷ��������¶�
	unsigned short      FCCError;//�ɿؼ�������ϱ���
	unsigned char       FCCLevel;//�ɿؼ����״̬�ȼ�
	unsigned int        EngineError;//�����������루145ר�ã�
	unsigned short      CollectionBoxState;//�ɼ��й���״̬��145ר�ã�
	unsigned char       ForceLandPointIndex;//�Ƚ������
	unsigned char       ControlMode;//����ģʽ״̬��
	unsigned short      ModeUsed;//ģ̬Ͷ��״̬��
	unsigned short      ManeuMode;//��������ģ̬״̬��
	unsigned char       EngineMode;//������״̬��
	unsigned char       Startup;//һ������״̬��
	unsigned char       Closedown;//һ���ر�״̬��
	double              PSI_DELTA;//����ƫ��
	double              DIS_XY;//�ɻ���ǰλ������ɵ�֮�����
	double              DIS_X;//��ǰλ������ɵ�λ�õ�X��ƫ��
	unsigned short		BackupNaviError1;//���ߵ�������0-15
}*pSecondsubThreeframe;

typedef struct SecondsubFourframe
{
	SecondsubFourframe()
	{
		DIS_Y = 0;
		takeoffHeight = 0;
		takeoffWeight = 0;
		NowWeight = 0;
		AvoidPosX = 0;
		AvoidPosY = 0;
		EquipmentState = 0;
		EquipmentAlarm1 = 0;
		EquipmentAlarm2 = 0;
		EquipmentAlarm3 = 0;
		EquipmentAlarm4 = 0;
		FlightTime = 0;
		FlightSurTime = 0;
		WaterFan = 0;
		MiddleFan = 0;
		MidFanCurrent = 0;
		WaterFacCurrent = 0;
		PowerBox7V = 0;
		GeneratorCurrent = 0;
		RadioHeight = 0;
		RadioSmoothAlt = 0;
		Power12V = 0;
		Power24V = 0;
	}
	double              DIS_Y;//��ǰλ������ɵ�λ�õ�Y��ƫ��
	double              takeoffHeight;//��ɻ�������
	double              takeoffWeight;//�ɻ��������
	double              NowWeight;//�ɻ�ʵʱ����
	short				AvoidPosX;//��ǰλ����������ƫ��X
	short				AvoidPosY;//��ǰλ����������ƫ��Y
	unsigned short      EquipmentState;//�����豸״̬��
	unsigned char       EquipmentAlarm1;//�����豸�澯��1
	unsigned char       EquipmentAlarm2;//�����豸�澯��2
	unsigned char       EquipmentAlarm3;//�����豸�澯��3
	unsigned char       EquipmentAlarm4;//�����豸�澯��4
	short               FlightTime;//���й���ʱ��
	short               FlightSurTime;//ʣ�ຽ��ʱ��
	unsigned char       WaterFan;//������ˮɢ�ȷ���״̬
	unsigned char       MiddleFan;//�������������״̬
	unsigned char       BackupNaviError2;//���ߵ�������16-23
	//double              RadarHeight;//���ײ��״�߶�
	//double              LandHeight;//���ײ��״�����ٶ�
	double				MidFanCurrent;//������ȹ������
	double				WaterFacCurrent;//ˮ����ȹ������
	double				PowerBox7V;//��Դ������7.4V��ѹ
	double				GeneratorCurrent;//������������
	double              RadioHeight;//���ߵ�߶�
	double              RadioSmoothAlt;//���ߵ�߶ȣ�ƽ���˲���
	unsigned char       BackupNaviError3;//���ߵ�������24-31
	double              Power12V;//��Դ������12V��ѹ
	double              Power24V;//��Դ������24V��ѹ
}*pSecondsubFourframe;

typedef struct ThirdsubOneframe
{
	ThirdsubOneframe()
	{
		DisperseIn1 = 0;
		DisperseOut2 = 0;
		DisperseOutRe2 = 0;
		Battery = 0;
		OpenK2 = 0;
		OilPressure = 0;
		OilVolume = 0;
		OpenK4 = 0;
		RetarderPressure = 0;
		RetarderTemp = 0;
		OpenK1 = 0;
		Tension = 0;
	}
	unsigned char       DisperseIn1;//�ⲿ��ɢ��������1
	//unsigned char       DisperseOut1;//��ɢ���������1
	unsigned short      DisperseOut2;//��ɢ���ָ��2
	unsigned short      DisperseOutRe2;//��ɢ���������2
	unsigned char       Battery;//����״̬
	double				MemsTemp;//��mems�¶�
	double              OpenK2;//����λ����K2��ѹֵ
	double              OilPressure;//ȼ��ѹ����ѹֵ
	double              OilVolume;//ȼ��������ѹֵ
	double              OpenK4;//�ҵ�λ����K4��ѹֵ
	double              RetarderPressure;//����������ѹ����ѹֵ
	double              RetarderTemp;//�����������¶ȵ���ֵ
	double              OpenK1;//����λ����K1��ѹֵ
	double              Tension;//�Ž�״ָ̬ʾ��ѹֵ
}*pThirdsubOneframe;

typedef struct ThirdsubTwoframe
{
	ThirdsubTwoframe()
	{
		OpenK3 = 0;
		Motortemp2 = 0;
		ROiltemp = 0;
		ROilPress = 0;
		Oil = 0;
		EngineRPM145 = 0;
		ThrottlePos145 = 0;
		EngineOilPre145 = 0;
		EngineOilTemp145 = 0;
		ExhaustTemp1 = 0;
		ExhaustTemp2 = 0;
		ExhaustTemp3 = 0;
		ExhaustTemp4 = 0;
		IntakePressure = 0;
		TurboPressure = 0;
		CoolantTemp = 0;
		EngineOilPre = 0;
	}
	double              OpenK3;//����λ����K3��ѹֵ
	//double              Motortemp1;//����1�¶�
	double              Motortemp2;//����2�¶�
	double              ROiltemp;//�����������¶�
	double              ROilPress;//����������ѹ��
	double              OilPress;//ȼ��ѹ��
	double              Oil;//ȼ������
	double				OilRight;//����������
	double				EngineRPM145;//������ת�٣�145ר�ã�
	double				ThrottlePos145;//������λ�ã�145ר�ã�
	double				EngineOilPre145;//����������ѹ����145ר�ã�
	double				EngineOilTemp145;//�����������¶ȣ�145ר�ã�
	double              ExhaustTemp1;//�����¶�1
	double              ExhaustTemp2;//�����¶�2
	double              ExhaustTemp3;//�����¶�3
	double              ExhaustTemp4;//�����¶�4
	double              IntakePressure;//����������ѹ��
	double              TurboPressure;//������ѹѹ��
	double              CoolantTemp;//��ȴҺ�¶�
	double              EngineOilPre;//����������ѹ��
}*pThirdsubTwoframe;

typedef struct ThirdsubThreeframe
{
	ThirdsubThreeframe()
	{
		EngineOilTemp = 0;
		DamperPostion = 0;
		InairTemp = 0;
		InairPre = 0;
		OilUsed = 0;
		OutAirPostion = 0;
		EngineRAM = 0;
		EnineInAirTemp = 0;
		EcuAbus = 0;
		EcuBbus = 0;
		EngineTime = 0;
		OilPreDiff = 0;
		StableBoxPre = 0;
		StableBoxGoalPre = 0;
		StableBoxTemp = 0;
		CoolingTempMax = 0;
		CoolingTempMin = 0;
		ExhaustTempmax = 0;
		ExhaustTempmin = 0;
	}
	double              EngineOilTemp;//�����������¶�
	double              DamperPostion;//����λ��
	double              InairTemp;//��������¶�
	double              InairPre;//�������ѹ��
	double              OilUsed;//ȼ������
	double              OutAirPostion;//������λ��
	double              EngineRAM;//ECUB������ת��
	double              EnineInAirTemp;//��������������
	double              EcuAbus;//ECUA-BUS��ѹ
	double              EcuBbus;//ECUB-BUS��ѹ
	int                 EngineTime;//����������ʱ��
	double				OilPreDiff;//ȼ��ѹ���145ר�ã�
	double				StableBoxPre;//��ѹ��ѹ����145ר�ã�
	double				StableBoxGoalPre;//��ѹ��Ŀ��ѹ����145ר�ã�
	double				StableBoxTemp;//��ѹ���¶ȣ�145ר�ã�
	double				CoolingTempMax;//��ȴҺ�¶����ֵ��145ר�ã�
	double				CoolingTempMin;//��ȴҺ�¶����ֵ��145ר�ã�
	double				ExhaustTempmax;//�������ֵ��145ר�ã�
	double				ExhaustTempmin;//�������ֵ��145ר�ã�
}*pThirdsubThreeframe;

typedef struct ThirdsubFourframe
{
	ThirdsubFourframe()
	{
		KgReply = 0;
		YtReply = 0;
		YtHdReply = 0;
		B2YReply = 0;
		B2XReply = 0;
		B1YReply = 0;
		B1XReply = 0;
		DamperReply = 0;
		byte11Reply = 0;
		byte12Reply = 0;
		byte13Reply = 0;
		byte14Reply = 0;
		byte15Reply = 0;
		byte16Reply = 0;
		byte17Reply = 0;
		byte18Reply = 0;
		byte19Reply = 0;
		byte20Reply = 0;
		byte21Reply = 0;
		byte22Reply = 0;
		byte23Reply = 0;
		byte24Reply = 0;
		byte25Reply = 0;
		byte26Reply = 0;
		byte27Reply = 0;
		LinkTest = 0;
		EngineAirPre = 0;
		TCUPower = 0;
		ServoPos = 0;
		ServoGoalPos = 0;
		B2YControl = 0;
		B2XControl = 0;
		B1YControl = 0;
		B1XControl = 0;
		FUTABAState = 0;
	}
	unsigned short      KgReply;//���п���ָ��Ӧ��
	unsigned char       YtReply;//�������ָ�����ر�
	unsigned char       YtHdReply;//�������ָ�����Ӧ��
	double              B2YReply;//�������ڱ��ң��ָ��ر������غУ�
	double              B2XReply;//�������ڱ��ң��ָ��ر������غУ�
	double              B1YReply;//�ܾ�ң��ָ��ر������غУ�
	double              B1XReply;//����ң��ָ��ر������غУ�
	double              DamperReply;//����ң��ָ��ر������غУ�
	unsigned char       byte11Reply;//���ָ�����ݻر�-ԭ֡11�ֽ�
	unsigned char       byte12Reply;//���ָ�����ݻر�-ԭ֡12�ֽ�
	unsigned char       byte13Reply;//���ָ�����ݻر�-ԭ֡13�ֽ�
	unsigned char       byte14Reply;//���ָ�����ݻر�-ԭ֡14�ֽ�
	unsigned char       byte15Reply;//���ָ�����ݻر�-ԭ֡15�ֽ�
	unsigned char       byte16Reply;//���ָ�����ݻر�-ԭ֡16�ֽ�
	unsigned char       byte17Reply;//���ָ�����ݻر�-ԭ֡17�ֽ�
	unsigned char       byte18Reply;//���ָ�����ݻر�-ԭ֡18�ֽ�
	unsigned char       byte19Reply;//���ָ�����ݻر�-ԭ֡19�ֽ�
	unsigned char       byte20Reply;//���ָ�����ݻر�-ԭ֡20�ֽ�
	unsigned char       byte21Reply;//���ָ�����ݻر�-ԭ֡21�ֽ�
	unsigned char       byte22Reply;//���ָ�����ݻر�-ԭ֡22�ֽ�
	unsigned char       byte23Reply;//���ָ�����ݻر�-ԭ֡23�ֽ�
	unsigned char       byte24Reply;//���ָ�����ݻر�-ԭ֡24�ֽ�
	unsigned char       byte25Reply;//���ָ�����ݻر�-ԭ֡25�ֽ�
	unsigned char       byte26Reply;//���ָ�����ݻر�-ԭ֡26�ֽ�
	unsigned char       byte27Reply;//���ָ�����ݻر�-ԭ֡27�ֽ�
	unsigned char       LinkTest;//ң��ң����·���Լ���
	double				EngineAirPre;//����������ѹ����145ר�ã�
	double				TCUPower;//TCU�����ѹ��145ר�ã�
	double				ServoPos;//�ŷ����λ�ã�145ר�ã�
	double				ServoGoalPos;//�ŷ����Ŀ��λ�ã�145ר�ã�
	double				NaviPDOP;	//�ߵ�PDOPֵ
	double              B2YControl;//����ң��ָ��ر���FUTABA)
	double              B2XControl;//����ң��ָ��ر���FUTABA)
	double              B1YControl;//�ܾ�ң��ָ��ر���FUTABA)
	double              B1XControl;//����ң��ָ��ر���FUTABA)
	unsigned char       FUTABAState;//FUTABA״̬��
}*pThirdsubFourframe;

struct TD550TelemetryData :public DataDefineInterface
{
	~TD550TelemetryData() override=default;
	FirstsubOneframe m_FirstSubOneFrame;				//��һ��֡��һС֡
	FirstsubTwoframe m_FirstSubTwoFrame;				//��һ��֡�ڶ�С֡
	FirstsubThreeframe m_FirstSubThreeFrame;			//��һ��֡����С֡
	FirstsubFourframe m_FirstSubFourFrame;				//��һ��֡����С֡
	SecondsubOneframe m_SecondSubOneFrame;				//�ڶ���֡��һС֡
	SecondsubTwoframe m_SecondSubTwoFrame;				//�ڶ���֡�ڶ�С֡
	SecondsubThreeframe m_SecondSubThreeFrame;			//�ڶ���֡����С֡
	SecondsubFourframe m_SecondSubFourFrame;			//�ڶ���֡����С֡
	ThirdsubOneframe m_ThirdSubOneFrame;				//������֡��һС֡
	ThirdsubTwoframe m_ThirdSubTwoFrame;				//������֡�ڶ�С֡
	ThirdsubThreeframe m_ThirdSubThreeFrame;			//������֡����С֡
	ThirdsubFourframe m_ThirdSubFourFrame;				//������֡����С֡
};