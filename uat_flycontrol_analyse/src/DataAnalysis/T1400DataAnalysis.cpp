#include "T1400DataAnalysis.h"
#include "../DataDefine/T1400TelemetryData.h"
#include <cstring>

T1400DataAnalysis::T1400DataAnalysis()
	: DataAnalysisInterface()
{
	m_nDataSize = 0;
	m_nBufSize = 4096 * 5;
	m_pBuf = new unsigned char[m_nBufSize];
	m_pBufSwap = new unsigned char[m_nBufSize];
	m_nFlag = 0;
	memset(m_arrDecodeData, 0, 256);

	m_T1400TelemetryData = std::make_shared<T1400TelemetryData>();
}

T1400DataAnalysis::~T1400DataAnalysis()
{
	
}

std::shared_ptr<DataDefineInterface> T1400DataAnalysis::AnalyseData(unsigned char* buf, int nLength)
{
	if ((nLength > m_nBufSize) || (nLength < 0))
		return nullptr;

	static Decode450State decodeState = Decode_Head81;
	static unsigned char decodeData[256] = { 0 };
	static unsigned int decodeDataLength = 0;
	memcpy(m_pBuf, buf, nLength);
	m_nDataSize = nLength;

	int nIndex = 0;
	while (nIndex < m_nDataSize)
	{
		//若decodeData数组已满，仍未找到一帧完整的数据，应将所有数据和状态清空
		if (decodeDataLength >= 256)
		{
			decodeDataLength = 0;
			decodeState = Decode_Head81;
			memset(decodeData, 0, 256);
		}

		if (Decode_Head81 == decodeState)
		{
			//寻找第一副帧帧头
			if (HEAD81 == m_pBuf[nIndex])
			{
				decodeData[decodeDataLength++] = m_pBuf[nIndex];
				decodeState = Decode_Head81_NUM;
			}
		}
		else if (Decode_Head81_NUM == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第一副帧编号
			if (2 == decodeDataLength)
			{
				if ((HEAD81_1 == decodeData[decodeDataLength - 1]) || (HEAD81_2 == decodeData[decodeDataLength - 1]) ||
					(HEAD81_3 == decodeData[decodeDataLength - 1]))
				{
					decodeState = Decode_Frame81_EB;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Frame81_EB == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第一副帧同步字1
			if (63 == decodeDataLength)
			{
				if (TAILEB == decodeData[decodeDataLength - 1])
				{
					decodeState = Decode_Frame81_90;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Frame81_90 == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第一副帧同步字2
			if (64 == decodeDataLength)
			{
				if (TAIL90 == decodeData[decodeDataLength - 1])
				{
					decodeState = Decode_Head82;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Head82 == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第二副帧帧头
			if (65 == decodeDataLength)
			{
				if (HEAD82 == decodeData[decodeDataLength - 1])
				{
					decodeState = Decode_Head82_NUM;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Head82_NUM == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第二副帧编号
			if (66 == decodeDataLength)
			{
				if ((HEAD82_1 == decodeData[decodeDataLength - 1]) || (HEAD82_2 == decodeData[decodeDataLength - 1]) ||
					(HEAD82_3 == decodeData[decodeDataLength - 1]))
				{
					decodeState = Decode_Frame82_EB;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Frame82_EB == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第二副帧同步字1
			if (127 == decodeDataLength)
			{
				if (TAILEB == decodeData[decodeDataLength - 1])
				{
					decodeState = Decode_Frame82_90;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Frame82_90 == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第二副帧同步字2
			if (128 == decodeDataLength)
			{
				if (TAIL90 == decodeData[decodeDataLength - 1])
				{
					decodeState = Decode_Head83;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Head83 == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第三副帧帧头
			if (129 == decodeDataLength)
			{
				if (HEAD83 == decodeData[decodeDataLength - 1])
				{
					decodeState = Decode_Head83_NUM;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Head83_NUM == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第三副帧编号
			if (130 == decodeDataLength)
			{
				if ((HEAD83_1 == decodeData[decodeDataLength - 1]) || (HEAD83_2 == decodeData[decodeDataLength - 1]) ||
					(HEAD83_3 == decodeData[decodeDataLength - 1]))
				{
					decodeState = Decode_Frame83_EB;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Frame83_EB == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第三副帧同步字1
			if (191 == decodeDataLength)
			{
				if (TAILEB == decodeData[decodeDataLength - 1])
				{
					decodeState = Decode_Frame83_90;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Frame83_90 == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第三副帧同步字2
			if (192 == decodeDataLength)
			{
				if (TAIL90 == decodeData[decodeDataLength - 1])
				{
					decodeState = Decode_Head84;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Head84 == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第四副帧帧头
			if (193 == decodeDataLength)
			{
				if (HEAD84 == decodeData[decodeDataLength - 1])
				{
					decodeState = Decode_Head84_NUM;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Head84_NUM == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第四副帧编号
			if (194 == decodeDataLength)
			{
				if ((HEAD84_1 == decodeData[decodeDataLength - 1]) || (HEAD84_2 == decodeData[decodeDataLength - 1]) ||
					(HEAD84_3 == decodeData[decodeDataLength - 1]))
				{
					decodeState = Decode_Frame84_EB;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Frame84_EB == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第四副帧同步字1
			if (255 == decodeDataLength)
			{
				if (TAILEB == decodeData[decodeDataLength - 1])
				{
					decodeState = Decode_Frame84_90;
				}
				else
				{
					decodeState = Decode_Head81;
					decodeDataLength = 0;
					memset(decodeData, 0, 256);
				}
			}
		}
		else if (Decode_Frame84_90 == decodeState)
		{
			decodeData[decodeDataLength++] = m_pBuf[nIndex];
			//寻找第四副帧同步字
			if (256 == decodeDataLength)
			{
				if (TAIL90 == decodeData[decodeDataLength - 1])
				{
					memcpy(m_arrDecodeData, decodeData, 256);
					handleData();

					return m_T1400TelemetryData;
				}

				decodeState = Decode_Head81;
				decodeDataLength = 0;
				memset(decodeData, 0, 256);
			}
		}

		nIndex++;
	}

	return nullptr;
}

void T1400DataAnalysis::handleData()
{
	unsigned char cCRCData[201];
	int nIndex = 0;
	memset(cCRCData, 0, 201);
	for (int i = 1; i <= 241; i++)
	{
		if ((i > 11 && i < 20) || (i>43 && i < 52) || (i>107 && i < 116) || (i>139 && i < 148) || (i>171 && i < 180))
		{
			continue;
		}
		else
			cCRCData[nIndex++] = m_arrDecodeData[i];
	}

	unsigned short crc;
	crc = CalCRC16_CCITT(cCRCData, nIndex);
	if (crc == (m_arrDecodeData[242] + m_arrDecodeData[243] * 0x100))
	{
		decode1400(m_arrDecodeData, 256);
	}
	memset(m_arrDecodeData, 0, 256);
}

unsigned short T1400DataAnalysis::CalCRC16_CCITT(unsigned char *chkbuf, int len)
{
	unsigned char byte_up = 0;
	unsigned char byte_temp = 0;
	unsigned short CRC_temp = 0;
	unsigned short CRC_code = 0xffff;
	unsigned short tempL = 0;

	for (int i = 0; i < len; i++)
	{
		tempL = chkbuf[i];

		byte_up = (unsigned char)(CRC_code >> 12);

		byte_temp = byte_up ^ (tempL >> 4);

		CRC_temp = CRC16_CCIT_table[byte_temp];

		CRC_code = ((CRC_code << 4) ^ CRC_temp) & 0xffff;

		byte_up = (unsigned char)(CRC_code >> 12);

		byte_temp = byte_up ^ (tempL & 0xf);

		CRC_temp = CRC16_CCIT_table[byte_temp];

		CRC_code = ((CRC_code << 4) ^ CRC_temp);
	}
	return CRC_code;
}

void T1400DataAnalysis::decode1400(unsigned char * m_rebuf, int m_rxlen)
{
	int	len = 0;
	unsigned char ch;
	static int cnt = 0;
	static unsigned char g_rxBuf[512];
	static enum DecoderState450 state = DECODER_STATE_SYNC0;
	for (int i = 0; i < m_rxlen; i++)
	{
		ch = m_rebuf[i];
		if (state == DECODER_STATE_SYNC0)
		{
			if (ch == START1 || ch == START2 || ch == START3 || ch == START4)//帧头是0x81,0x82,0x83,0x84
			{
				cnt = 0;
				g_rxBuf[cnt++] = ch;
				state = DECODER_STATE_SYNC1;//81,82,83,84
			}
		}
		else if (state == DECODER_STATE_SYNC1)
		{
			//判断如果计数超过协议规定同步字位置，则丢弃该帧
			if (cnt > 0x1F)
			{
				state = DECODER_STATE_SYNC0;
			}
			if (cnt == 0x1F)//根据协议，找到同步字所在位置
			{
				g_rxBuf[cnt++] = ch;
				if (g_rxBuf[0x1E] == 0x55 && g_rxBuf[0x1F] == 0xAA)//同步字匹配
				{
					state = DECODER_STATE_SYNC2;//81
				}
				else
				{
					state = DECODER_STATE_SYNC0;
				}
			}
			else
			{
				g_rxBuf[cnt++] = ch;
			}
		}
		else if (state == DECODER_STATE_SYNC2)//帧头状态
		{
			if (ch == g_rxBuf[0])//81,82,83,84
			{
				state = DECODER_STATE_SYNC3;
				g_rxBuf[cnt++] = ch;
			}
			else
			{
				state = DECODER_STATE_SYNC0;
			}
		}
		else if (state == DECODER_STATE_SYNC3)//同步字状态
		{
			//判断如果计数超过协议规定同步字位置，则丢弃该帧
			if (cnt > 0x3F)
			{
				state = DECODER_STATE_SYNC0;
			}
			if (cnt == 0x3F)
			{
				g_rxBuf[cnt++] = ch;
				if (g_rxBuf[0x3E] == SYNC3&&g_rxBuf[0x3F] == SYNC4)
				{
					if (g_rxBuf[0] == START1)//第一副帧原始数据，分为A机、B机
					{
						if (1 == 1)//450与101切换标识，暂时不使用
						{
							if (g_rxBuf[1] == 0x11)
							{
								FirstsubOneframe& firstOneFrame=m_T1400TelemetryData->m_FirstSubOneFrame;
								memset(&firstOneFrame, 0, sizeof(firstOneFrame));
								m_YCMutex.lock();
								firstOneFrame.SubCounts = (g_rxBuf[7] * 0x1000000 + g_rxBuf[6] * 0x10000 + g_rxBuf[5] * 0x100 + g_rxBuf[4])*1.0*0.02;
								firstOneFrame.FCVersion = g_rxBuf[9] * 0x100 + g_rxBuf[8];			//飞控版本
								firstOneFrame.CalibAirSpeed = (*(short*)&g_rxBuf[10])* 1.0 * 0.00153;//校准空速
								firstOneFrame.TureAirSpeed = (*(short*)&g_rxBuf[20]) * 1.0 * 0.00153;//真空速
								firstOneFrame.AirPressHeight = *(short*)&g_rxBuf[22] * 1.0 * 0.3357;//气压高
								firstOneFrame.StaticTemp = *(short*)&g_rxBuf[28] * 1.0 * 0.00214;//大气静温
								firstOneFrame.TotalTemp = *(short*)&g_rxBuf[33] * 1.0 * 0.00214;//大气总温
								firstOneFrame.SmoothAirSpeed = (*(short*)&g_rxBuf[35]) * 1.0 * 0.00153;//校准空速（平滑处理）
								firstOneFrame.AvoidNorthPos1 = (*(short*)&g_rxBuf[37]);
								firstOneFrame.AvoidEastPos1 = (*(short*)&g_rxBuf[39]);
								firstOneFrame.AvoidNorthPos2 = (*(short*)&g_rxBuf[41]);
								firstOneFrame.NaviState = g_rxBuf[43];
								firstOneFrame.AvoidEastPos2 = (*(short*)&g_rxBuf[58]);
								firstOneFrame.AvoidNorthPos3 = (*(short*)&g_rxBuf[60]);
								m_YCMutex.unlock();
							}

							if (g_rxBuf[1] == 0x21)
							{
								SecondsubOneframe& secondOneFrame=m_T1400TelemetryData->m_SecondSubOneFrame;
								memset(&secondOneFrame, 0, sizeof(secondOneFrame));
								m_YCMutex.lock();
								secondOneFrame.InertialState = g_rxBuf[8];
								secondOneFrame.BDPdop = (*(short*)&g_rxBuf[9]) * 1.0 * 0.01;
								secondOneFrame.MEMSNum = g_rxBuf[11];
								secondOneFrame.UTCTime = (g_rxBuf[23] * 0x1000000 + g_rxBuf[22] * 0x10000 + g_rxBuf[21] * 0x100 + g_rxBuf[20])* 1.0 * 0.001;
								secondOneFrame.FlightState = g_rxBuf[28];
								secondOneFrame.RouteNum = g_rxBuf[29];
								secondOneFrame.WayPointNum = g_rxBuf[33];
								secondOneFrame.WayPointState = g_rxBuf[34];
								secondOneFrame.NextPointNum = g_rxBuf[35];
								secondOneFrame.NextPointState = g_rxBuf[36];
								secondOneFrame.HeadingControl = (*(short*)&g_rxBuf[37])* 1.0 * 0.01;
								secondOneFrame.TrackError = (*(short*)&g_rxBuf[39])* 1.0 * 0.01;
								secondOneFrame.SetoverDis = *(short*)&g_rxBuf[41];
								secondOneFrame.MainNavNum = g_rxBuf[43];
								secondOneFrame.FlushingDis = *(short*)&g_rxBuf[58];
								secondOneFrame.FlushingTime = g_rxBuf[61] * 0x100 + g_rxBuf[60];
								m_YCMutex.unlock();
							}

							if (g_rxBuf[1] == 0x31)
							{
								ThirdsubOneframe& thirdOneFrame=m_T1400TelemetryData->m_ThirdSubOneFrame;
								memset(&thirdOneFrame, 0, sizeof(thirdOneFrame));
								m_YCMutex.lock();
								thirdOneFrame.DisperseIn1 = g_rxBuf[8];
								thirdOneFrame.DisperseOut2 = g_rxBuf[11] * 0x100 + g_rxBuf[10];
								thirdOneFrame.DisperseOutRe2 = g_rxBuf[21] * 0x100 + g_rxBuf[20];
								
								thirdOneFrame.Battery = g_rxBuf[22];
								thirdOneFrame.OpenK2 = (*(short*)&g_rxBuf[28])* 1.0 * 0.1;//前旋翼转速;
								double aaa = (*(short*)&g_rxBuf[33]);
								thirdOneFrame.OilPressure = (*(short*)&g_rxBuf[33])* 1.0 * 0.001;//前发燃油压力;
								thirdOneFrame.OilPressure2 = (*(short*)&g_rxBuf[35])* 1.0 * 0.001;//后发燃油压力;
								thirdOneFrame.OilVolume = (*(short*)&g_rxBuf[37])* 1.0 * 0.001;//下油量电压;

								thirdOneFrame.RetarderPressure = (*(short*)&g_rxBuf[39])* 1.0 * 0.1;
								thirdOneFrame.RetarderTemp = (*(short*)&g_rxBuf[41])* 1.0 * 0.1;
								thirdOneFrame.OpenK1 = (*(short*)&g_rxBuf[58])* 1.0;//离合状态;
								thirdOneFrame.Tension = (*(short*)&g_rxBuf[60])* 1.0 * 0.001;//离合反馈电压;
								m_YCMutex.unlock();
							}
						}
					}
					else if (g_rxBuf[0] == START2)//第二副帧原始数据，分为A机、B机
					{
						if (1 == 1)//450与101切换标识，暂时不使用
						{
							if (g_rxBuf[1] == 0x12)
							{
								FirstsubTwoframe& firstTwoFrame=m_T1400TelemetryData->m_FirstSubTwoFrame;
								memset(&firstTwoFrame, 0, sizeof(firstTwoFrame));
								m_YCMutex.lock();
								firstTwoFrame.D1ServoControl = (*(short*)&g_rxBuf[3])* 1.0 * 0.002;//D1舵机位置控制律指令
								firstTwoFrame.D2ServoControl = (*(short*)&g_rxBuf[5])* 1.0 * 0.002;//D2舵机位置控制律指令
								firstTwoFrame.D3ServoControl = (*(short*)&g_rxBuf[7])* 1.0 * 0.002;//D3舵机位置控制律指令
								firstTwoFrame.U1ServoControl = (*(short*)&g_rxBuf[9])* 1.0 * 0.002;//U1舵机位置控制律指令
								firstTwoFrame.U2ServoControl = (*(short*)&g_rxBuf[11])* 1.0 * 0.002;//U2舵机位置控制律指令
								firstTwoFrame.U3ServoControl = (*(short*)&g_rxBuf[13])* 1.0 * 0.002;//U3舵机位置控制律指令
								firstTwoFrame.DamperServoControl = (*(short*)&g_rxBuf[15])* 1.0 * 0.01;//风门舵机位置控制律指令
								firstTwoFrame.DirServoControl = (*(short*)&g_rxBuf[17])* 1.0 * 0.01;//方向舵机位置控制律指令
								firstTwoFrame.D1ServoPostion = (*(short*)&g_rxBuf[19])* 1.0 * 0.001;//D1舵机位置反馈
								firstTwoFrame.D2ServoPostion = (*(short*)&g_rxBuf[21])* 1.0 * 0.001;//D2舵机位置反馈
								firstTwoFrame.D3ServoPostion = (*(short*)&g_rxBuf[23])* 1.0 * 0.001;//D3舵机位置反馈
								firstTwoFrame.U1ServoPostion = (*(short*)&g_rxBuf[25])* 1.0 * 0.001;//u1舵机位置反馈
								firstTwoFrame.U2ServoPostion = (*(short*)&g_rxBuf[36])* 1.0 * 0.001;//u2舵机位置反馈
								firstTwoFrame.U3ServoPostion = (*(short*)&g_rxBuf[38])* 1.0 * 0.001;//u3舵机位置反馈
								firstTwoFrame.QXYZJJJJ = (*(short*)&g_rxBuf[40])* 1.0 * 0.001;//前旋翼总距桨距角
								firstTwoFrame.QXYFYJJJ = (*(short*)&g_rxBuf[42])* 1.0 * 0.001;//前旋翼俯仰桨距角
								firstTwoFrame.lat = (*(int*)&g_rxBuf[52])* 1.0 * 0.0000001;//纬度
								firstTwoFrame.lon = (*(int*)&g_rxBuf[56])* 1.0 * 0.0000001;//经度
								m_YCMutex.unlock();
							}

							if (g_rxBuf[1] == 0x22)
							{
								SecondsubTwoframe& secondTwoFrame=m_T1400TelemetryData->m_SecondSubTwoFrame;
								memset(&secondTwoFrame, 0, sizeof(secondTwoFrame));
								m_YCMutex.lock();
								secondTwoFrame.GSB = (*(short*)&g_rxBuf[3])* 1.0 * 0.01;
								secondTwoFrame.AvoidTimeIndex = g_rxBuf[6] * 0x100 + g_rxBuf[5];
								secondTwoFrame.AvoidTaskState = g_rxBuf[7];
								secondTwoFrame.AvoidNorthSpeed = (*(short*)&g_rxBuf[8])*1.0*0.01;
								secondTwoFrame.AvoidEastSpeed = (*(short*)&g_rxBuf[10])*1.0*0.01;
								secondTwoFrame.AcoidFCState = g_rxBuf[12];
								secondTwoFrame.AvoidFCOrder = g_rxBuf[13];
								secondTwoFrame.ctc = (*(short*)&g_rxBuf[15])* 1.0 * 0.01;
								secondTwoFrame.b1c = (*(short*)&g_rxBuf[17])* 1.0 * 0.01;
								secondTwoFrame.a1c = (*(short*)&g_rxBuf[19])* 1.0 * 0.01;
								secondTwoFrame.dtc = (*(short*)&g_rxBuf[21])* 1.0 * 0.01;
								secondTwoFrame.HZJJJJ = (*(short*)&g_rxBuf[23])* 1.0 * 0.001;
								secondTwoFrame.RouteSpeed = (*(short*)&g_rxBuf[25])* 1.0 * 0.01;
								secondTwoFrame.RouteVPostion = (*(short*)&g_rxBuf[36])* 1.0 * 0.01;
								secondTwoFrame.RouteHPostion = (*(short*)&g_rxBuf[38])* 1.0 * 0.01;
								secondTwoFrame.RouteYPostion = (*(short*)&g_rxBuf[40])* 1.0 * 0.01;
								secondTwoFrame.RouteHeight = (*(short*)&g_rxBuf[42])* 1.0 * 0.2;
								secondTwoFrame.RouteVSpeed = (*(short*)&g_rxBuf[52])* 1.0 * 0.01;
								secondTwoFrame.RouteSideSpeed = (*(short*)&g_rxBuf[54])* 1.0 * 0.01;
								secondTwoFrame.NaviState = g_rxBuf[59];
								m_YCMutex.unlock();


								// std::cout<<"NaviState is "<<secondTwoFrame.NaviState<<std::endl;
							}

							if (g_rxBuf[1] == 0x32)
							{
								ThirdsubTwoframe& thirdTwoFrame=m_T1400TelemetryData->m_ThirdSubTwoFrame;
								memset(&thirdTwoFrame, 0, sizeof(thirdTwoFrame));
								m_YCMutex.lock();
								thirdTwoFrame.OpenK3 = (*(short*)&g_rxBuf[3])* 1.0 * 0.1;
								thirdTwoFrame.Motortemp1 = (*(unsigned short*)&g_rxBuf[5])* 1.0 * 0.01;
								thirdTwoFrame.Motortemp2 = (*(unsigned short*)&g_rxBuf[7])* 1.0 * 0.01;
								thirdTwoFrame.ROiltemp = (*(short*)&g_rxBuf[9])* 1.0 * 0.1;
								thirdTwoFrame.ROilPress = (*(short*)&g_rxBuf[11])* 1.0 * 0.1;
								thirdTwoFrame.OilPress = (*(short*)&g_rxBuf[13])* 1.0 * 0.001;
								thirdTwoFrame.Oil = (*(short*)&g_rxBuf[15])* 1.0 * 0.01;
								thirdTwoFrame.EngineRPM145 = (*(short*)&g_rxBuf[19])* 1.0 * 0.2;
								thirdTwoFrame.ThrottlePos145 = (*(short*)&g_rxBuf[21])* 1.0 * 0.05;
								thirdTwoFrame.EngineOilPre145 = (*(unsigned short*)&g_rxBuf[23])*0.01;
								thirdTwoFrame.EngineOilTemp145 = (*(unsigned short*)&g_rxBuf[25])* 1.0 * 0.01;
								thirdTwoFrame.ExhaustTemp1 = ((short)(g_rxBuf[37] * 0x100 + (g_rxBuf[36])))*1.0*0.1;
								thirdTwoFrame.ExhaustTemp2 = ((short)(g_rxBuf[39] * 0x100 + (g_rxBuf[38])))*1.0*0.1;
								thirdTwoFrame.ExhaustTemp3 = (*(short*)&g_rxBuf[40])* 1.0 * 0.1;
								thirdTwoFrame.ExhaustTemp4 = (*(short*)&g_rxBuf[42])* 1.0 * 0.1;
								thirdTwoFrame.IntakePressure = (*(unsigned short*)&g_rxBuf[52])* 1.0 * 0.1;
								thirdTwoFrame.TurboPressure = (*(unsigned short*)&g_rxBuf[54])* 1.0 * 0.1;
								thirdTwoFrame.CoolantTemp = (*(unsigned short*)&g_rxBuf[56])* 1.0 * 0.1;
								thirdTwoFrame.EngineOilPre = ((*(short*)&g_rxBuf[58]) * 1.0)*0.01;
								m_YCMutex.unlock();
							}
						}
						else
						{

						}
					}
					else if (g_rxBuf[0] == START3)//第三副帧原始数据，分为A机、B机
					{
						if (1 == 1)//450与101切换标识，暂不使用
						{

							if (g_rxBuf[1] == 0x13)
							{
								FirstsubThreeframe& firstThreeFrame=m_T1400TelemetryData->m_FirstSubThreeFrame;
								memset(&firstThreeFrame, 0, sizeof(firstThreeFrame));
								m_YCMutex.lock();
								firstThreeFrame.AbsolutelyHeight = (*(short*)&g_rxBuf[3])* 1.0 * 0.5;
								firstThreeFrame.XSpeed = (*(short*)&g_rxBuf[5])* 1.0 * 0.05;
								firstThreeFrame.YSpeed = (*(short*)&g_rxBuf[7])* 1.0 * 0.05;
								firstThreeFrame.ZSpeed = (*(short*)&g_rxBuf[9])* 1.0 * 0.05;
								firstThreeFrame.RemoteAuthorizationStatus = g_rxBuf[15];//遥控器是否授权状态;
								firstThreeFrame.XaSpeed = (*(short*)&g_rxBuf[20])* 1.0 * 0.00245;
								firstThreeFrame.YaSpeed = (*(short*)&g_rxBuf[22])* 1.0 * 0.00245;
								firstThreeFrame.ZaSpeed = (*(short*)&g_rxBuf[24])* 1.0 * 0.00245;
								firstThreeFrame.RollAngVelocity = (*(short*)&g_rxBuf[26]) * 1.0 * 0.02;
								firstThreeFrame.PitchAngVelocity = (*(short*)&g_rxBuf[28])* 1.0 * 0.02;
								firstThreeFrame.YawAngVelocity = (*(short*)&g_rxBuf[33])* 1.0 * 0.02;
								firstThreeFrame.Yaw = (*(short*)&g_rxBuf[35])* 1.0 * 0.01;
								firstThreeFrame.Pitch = (*(short*)&g_rxBuf[37])* 1.0 * 0.01;
								firstThreeFrame.Roll = (*(short*)&g_rxBuf[39])* 1.0 * 0.01;
								firstThreeFrame.EastSpeed = (*(short*)&g_rxBuf[41])* 1.0 * 0.05;
								firstThreeFrame.NothSpeed = (*(short*)&g_rxBuf[52])* 1.0 * 0.05;
								firstThreeFrame.RelHeight = (*(short*)&g_rxBuf[54])* 1.0 * 0.2;
								firstThreeFrame.AYB = (*(short*)&g_rxBuf[56])* 1.0 * 0.00245;
								firstThreeFrame.EngineR = (g_rxBuf[59] * 0x100 + g_rxBuf[58])* 1.0 */* 0.2*/0.5;
								firstThreeFrame.MainRotor = (g_rxBuf[61] * 0x100 + g_rxBuf[60])* 1.0 * 0.1;
								m_YCMutex.unlock();
							}

							if (g_rxBuf[1] == 0x23)
							{
								SecondsubThreeframe& secondThreeFrame=m_T1400TelemetryData->m_SecondSubThreeFrame;
								memset(&secondThreeFrame, 0, sizeof(secondThreeFrame));
								m_YCMutex.lock();
								secondThreeFrame.FCCPower = (*(short*)&g_rxBuf[3])* 1.0 * 100 / 32767;
								secondThreeFrame.FCCState = g_rxBuf[6] * 0x100 + g_rxBuf[5];
								secondThreeFrame.FCCTemp = (*(short*)&g_rxBuf[7])* 1.0 * 0.0625;
								secondThreeFrame.ServoTemp = g_rxBuf[9];
								secondThreeFrame.FCCError = (g_rxBuf[11] * 0x100 + g_rxBuf[10]);
								secondThreeFrame.FCCLevel = g_rxBuf[20];
								secondThreeFrame.ErrorID_qian = g_rxBuf[21];
								secondThreeFrame.ErrorNumber_qian = (*(short*)&g_rxBuf[22]);
								secondThreeFrame.ErrorID_hou = g_rxBuf[24];
								secondThreeFrame.ErrorNumber_hou = (*(short*)&g_rxBuf[25]);
								secondThreeFrame.CollectionBoxState = g_rxBuf[28] * 0x100 + g_rxBuf[27];
								secondThreeFrame.ForceLandPointIndex = g_rxBuf[29];
								secondThreeFrame.ControlMode = g_rxBuf[33];
								secondThreeFrame.ModeUsed = g_rxBuf[35] * 0x100 + g_rxBuf[34];
								secondThreeFrame.ManeuMode = g_rxBuf[37] * 0x100 + g_rxBuf[36];
								secondThreeFrame.EngineMode = g_rxBuf[38];
								secondThreeFrame.Startup = g_rxBuf[39];
								secondThreeFrame.Closedown = g_rxBuf[40];
								secondThreeFrame.Reserved = g_rxBuf[41] * 1.0 * 0.1;;
								secondThreeFrame.PSI_DELTA = (*(short*)&g_rxBuf[42])* 1.0 * 0.01;
								secondThreeFrame.DIS_XY = (g_rxBuf[55] * 0x1000000 + g_rxBuf[54] * 0x10000 + g_rxBuf[53] * 0x100 + g_rxBuf[52])* 1.0 * 0.1;
								secondThreeFrame.DIS_X = (g_rxBuf[59] * 0x1000000 + g_rxBuf[58] * 0x10000 + g_rxBuf[57] * 0x100 + g_rxBuf[56])* 1.0 * 0.1;
								m_YCMutex.unlock();
							}

							if (g_rxBuf[1] == 0x33)
							{
								ThirdsubThreeframe& thirdThreeFrame=m_T1400TelemetryData->m_ThirdSubThreeFrame;
								memset(&thirdThreeFrame, 0, sizeof(thirdThreeFrame));
								m_YCMutex.lock();
								thirdThreeFrame.EngineOilTemp = (*(short*)&g_rxBuf[3])* 1.0 * 0.01;
								thirdThreeFrame.DamperPostion = (*(short*)&g_rxBuf[5])* 1.0 * 0.05;
								thirdThreeFrame.InairTemp = (*(short*)&g_rxBuf[7])* 1.0 * 0.1;
								thirdThreeFrame.InairPre = (*(short*)&g_rxBuf[9])* 1.0 * 0.1;
								
								thirdThreeFrame.FrontEngineIdleMarking = g_rxBuf[20] * 1.0;
								thirdThreeFrame.BehindEngineIdleMarking = g_rxBuf[21] * 1.0;
								thirdThreeFrame.OutAirPostion = (*(unsigned short*)&g_rxBuf[22])* 1.0 * 0.01;
								thirdThreeFrame.EngineRAM = (*(short*)&g_rxBuf[24])* 1.0 * 0.5;
								thirdThreeFrame.EnineInAirTemp = (*(unsigned short*)&g_rxBuf[26])* 0.01;
								thirdThreeFrame.EcuAbus = (*(short*)&g_rxBuf[28])* 1.0 * 0.1;
								thirdThreeFrame.EcuBbus = (*(short*)&g_rxBuf[33])* 1.0 * 0.1;
								thirdThreeFrame.EngineTime = (*(short*)&g_rxBuf[35]) * 20;
								thirdThreeFrame.OilPreDiff = (*(short*)&g_rxBuf[38])* 1.0 * 0.0001;//前发风扇pwm
								thirdThreeFrame.StableBoxPre = (*(short*)&g_rxBuf[40])* 1.0 * 0.1;//后发风扇pwm
								thirdThreeFrame.StableBoxGoalPre = (*(short*)&g_rxBuf[42])* 1.0 * 0.1;
								thirdThreeFrame.StableBoxTemp = (*(short*)&g_rxBuf[52])* 1.0 * 0.1;
								thirdThreeFrame.CoolingTempMax = (*(short*)&g_rxBuf[54])* 1.0 * 0.1;
								thirdThreeFrame.CoolingTempMin = (*(short*)&g_rxBuf[56])* 1.0 * 0.1;
								thirdThreeFrame.ExhaustTempmax = (*(short*)&g_rxBuf[58])* 1.0 * 0.1;
								thirdThreeFrame.ExhaustTempmin = (*(short*)&g_rxBuf[60])* 1.0 * 0.1;
								m_YCMutex.unlock();
							}
						}
					}
					else if (g_rxBuf[0] == START4)//第四副帧原始数据，分为A机、B机
					{
						if (1 == 1)//450切换标识
						{
							if (g_rxBuf[1] == 0x14)
							{
								FirstsubFourframe& firstFourFrame=m_T1400TelemetryData->m_FirstSubFourFrame;
								memset(&firstFourFrame, 0, sizeof(firstFourFrame));
								m_YCMutex.lock();
								firstFourFrame.SignalSource1 = g_rxBuf[3];
								firstFourFrame.SignalSource2 = g_rxBuf[4];
								firstFourFrame.SignalSource3 = g_rxBuf[5];
								firstFourFrame.SignalSource4 = g_rxBuf[6];
								firstFourFrame.SignalSource5 = g_rxBuf[7];
								firstFourFrame.FaultCode = g_rxBuf[12] * 0x1000000 + g_rxBuf[11] * 0x10000 + g_rxBuf[10] * 0x100 + g_rxBuf[9];
								firstFourFrame.AvoidEastPos3 = (*(short*)&g_rxBuf[13]);
								firstFourFrame.D1ServoMFault = g_rxBuf[15];
								firstFourFrame.D1ServoBFault = g_rxBuf[16];
								firstFourFrame.D2ServoMFault = g_rxBuf[17];
								firstFourFrame.D2ServoBFault = g_rxBuf[18];
								firstFourFrame.D3ServoMFault = g_rxBuf[19];
								firstFourFrame.D3ServoBFault = g_rxBuf[20];
								firstFourFrame.U1ServoMFault = g_rxBuf[21];
								firstFourFrame.U1ServoBFault = g_rxBuf[22];
								firstFourFrame.U2ServoMFault = g_rxBuf[23];
								firstFourFrame.U2ServoBFault = g_rxBuf[24];
								firstFourFrame.U3ServoMFault = g_rxBuf[25];
								firstFourFrame.U3ServoBFault = g_rxBuf[26];
								firstFourFrame.QXYHGJJJ = (*(short*)&g_rxBuf[27])* 1.0 * 0.001;
								firstFourFrame.DirServoMFault = g_rxBuf[29];
								firstFourFrame.DirServoBFault = g_rxBuf[33];
								firstFourFrame.D1ServoCurrent = (*(short*)&g_rxBuf[34])* 1.0 * 0.01;
								firstFourFrame.D2ServoCurrent = (*(short*)&g_rxBuf[36])* 1.0 * 0.01;
								firstFourFrame.D3ServoCurrent = (*(short*)&g_rxBuf[38])* 1.0 * 0.01;
								firstFourFrame.U1ServoCurrent = (*(short*)&g_rxBuf[40])* 1.0 * 0.01;
								firstFourFrame.U2ServoCurrent = (*(short*)&g_rxBuf[42])* 1.0 * 0.01;
								firstFourFrame.U3ServoCurrent = (*(short*)&g_rxBuf[44])* 1.0 * 0.01;
								firstFourFrame.HXYFYJJJ = (*(short*)&g_rxBuf[46])* 1.0 * 0.001;//后旋翼俯仰桨距角;
								firstFourFrame.HXYHGJJJ = (*(short*)&g_rxBuf[48])* 1.0 * 0.001;//后旋翼横滚桨距角;
								m_YCMutex.unlock();
							}

							if (g_rxBuf[1] == 0x24)
							{
								SecondsubFourframe& secondFourFrame=m_T1400TelemetryData->m_SecondSubFourFrame;
								memset(&secondFourFrame, 0, sizeof(secondFourFrame));
								m_YCMutex.lock();
								secondFourFrame.DIS_Y = (g_rxBuf[6] * 0x1000000 + g_rxBuf[5] * 0x10000 + g_rxBuf[4] * 0x100 + g_rxBuf[3])* 1.0 * 0.1;
								secondFourFrame.takeoffHeight = (*(short*)&g_rxBuf[7])* 1.0 * 0.2;
								secondFourFrame.takeoffWeight = (g_rxBuf[10] * 0x100 + g_rxBuf[9])* 1.0 * 0.1;
								secondFourFrame.NowWeight = (g_rxBuf[12] * 0x100 + g_rxBuf[11])* 1.0 * 0.1;
								secondFourFrame.AvoidPosX = (*(short*)&g_rxBuf[13]);
								secondFourFrame.AvoidPosY = (*(short*)&g_rxBuf[15]);
								secondFourFrame.EquipmentState = g_rxBuf[28] * 0x100 + g_rxBuf[17];
								secondFourFrame.EquipmentAlarm1 = g_rxBuf[18];
								secondFourFrame.EquipmentAlarm2 = g_rxBuf[19];
								secondFourFrame.EquipmentAlarm3 = g_rxBuf[20];
								secondFourFrame.EquipmentAlarm4 = g_rxBuf[21];
								secondFourFrame.FlightTime = g_rxBuf[23] * 0x100 + g_rxBuf[22];
								secondFourFrame.FlightSurTime = g_rxBuf[25] * 0x100 + g_rxBuf[24];
								secondFourFrame.WaterFan = g_rxBuf[26];
								secondFourFrame.MiddleFan = g_rxBuf[27];
								secondFourFrame.MidFanCurrent = (*(short*)&g_rxBuf[33])* 1.0 * 0.01;//前发风扇电流;
								secondFourFrame.WaterFacCurrent = (*(short*)&g_rxBuf[35])* 1.0 * 0.01;//后发风扇电流;
								secondFourFrame.GeneratorCurrent = (*(short*)&g_rxBuf[37])* 1.0 * 0.01;
								secondFourFrame.GeneratorCurrentBack = (*(short*)&g_rxBuf[39])* 1.0 * 0.01;
								secondFourFrame.RadioHeight = (*(short*)&g_rxBuf[41])* 1.0 * 0.1;
								secondFourFrame.RadioSmoothAlt = (*(short*)&g_rxBuf[43])* 1.0 * 0.1;
								secondFourFrame.BackupNaviError=g_rxBuf[45];
								secondFourFrame.Power74V = g_rxBuf[46] * 1.0 * 0.1;
								secondFourFrame.Power12V = g_rxBuf[47] * 1.0 * 0.1;
								secondFourFrame.Power24V = (*(short*)&g_rxBuf[48])* 1.0 * 0.01;
								m_YCMutex.unlock();
							}

							if (g_rxBuf[1] == 0x34)
							{
								ThirdsubFourframe& thirdFourFrame=m_T1400TelemetryData->m_ThirdSubFourFrame;
								memset(&thirdFourFrame, 0, sizeof(thirdFourFrame));
								m_YCMutex.lock();
								thirdFourFrame.KgReply = g_rxBuf[4] * 0x100 + g_rxBuf[3];
								thirdFourFrame.YtReply = g_rxBuf[5];
								thirdFourFrame.YtHdReply = g_rxBuf[6];
								thirdFourFrame.B2YReply = (*(char*)&g_rxBuf[7])* 100.0 / 127;
								thirdFourFrame.B2XReply = (*(char*)&g_rxBuf[8])* 100.0 / 127;
								thirdFourFrame.B1YReply = (*(char*)&g_rxBuf[9])* 100.0 / 127;
								thirdFourFrame.B1XReply = (*(char*)&g_rxBuf[10])* 100.0 / 127;
								thirdFourFrame.DamperReply = (*(char*)&g_rxBuf[11])* 100.0 / 127;
								thirdFourFrame.byte11Reply = *(char*)&g_rxBuf[13];
								thirdFourFrame.byte12Reply = *(char*)&g_rxBuf[14];
								thirdFourFrame.byte13Reply = *(char*)&g_rxBuf[15];
								thirdFourFrame.byte14Reply = *(char*)&g_rxBuf[16];
								thirdFourFrame.byte15Reply = *(char*)&g_rxBuf[17];
								thirdFourFrame.byte16Reply = *(char*)&g_rxBuf[18];
								thirdFourFrame.byte17Reply = *(char*)&g_rxBuf[19];
								thirdFourFrame.byte18Reply = *(char*)&g_rxBuf[20];
								thirdFourFrame.byte19Reply = *(char*)&g_rxBuf[21];
								thirdFourFrame.byte20Reply = *(char*)&g_rxBuf[22];
								thirdFourFrame.byte21Reply = *(char*)&g_rxBuf[23];
								thirdFourFrame.byte22Reply = *(char*)&g_rxBuf[24];
								thirdFourFrame.byte23Reply = *(char*)&g_rxBuf[25];
								thirdFourFrame.byte24Reply = *(char*)&g_rxBuf[26];
								thirdFourFrame.byte25Reply = *(char*)&g_rxBuf[27];
								thirdFourFrame.byte26Reply = *(char*)&g_rxBuf[28];
								thirdFourFrame.byte27Reply = *(char*)&g_rxBuf[29];
								thirdFourFrame.LinkTest = g_rxBuf[33];
								thirdFourFrame.EngineAirPre = (*(unsigned short*)&g_rxBuf[35])* 1.0 * 0.1;
								thirdFourFrame.TCUPower = (*(short*)&g_rxBuf[37])* 1.0 * 0.01;
								thirdFourFrame.ServoPos = (*(short*)&g_rxBuf[39])* 1.0 * 0.01;
								thirdFourFrame.ServoGoalPos = (*(short*)&g_rxBuf[41])* 1.0 * 0.01;
								thirdFourFrame.NaviPDOP = (*(unsigned short*)&g_rxBuf[43])* 1.0 * 0.01;
								thirdFourFrame.B2YControl = (*(char*)&g_rxBuf[45])* 1.0 / 127;
								thirdFourFrame.B2XControl = (*(char*)&g_rxBuf[46])* 1.0 / 127;
								thirdFourFrame.B1YControl = (*(char*)&g_rxBuf[47])* 1.0 / 127;
								thirdFourFrame.B1XControl = (*(char*)&g_rxBuf[48])* 1.0 / 127;
								thirdFourFrame.FUTABAState = g_rxBuf[49];
								m_YCMutex.unlock();
							}
						}
					}
					state = DECODER_STATE_SYNC0;
					cnt = 0;
				}
				else
				{
					state = DECODER_STATE_SYNC0;
				}
			}
			else
			{
				g_rxBuf[cnt++] = ch;
			}
		}
		else
		{
			state = DECODER_STATE_SYNC0;
		}
	}
}