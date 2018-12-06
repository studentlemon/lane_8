//#include "LDW_Warning.h"
#include "Common.h"
#include "LDW.h"
#include "math.h"



Uint16 m_iWarningCounter;               // 报警计数
VEHICLESTATUS m_InputVehicleStatus;		// 当前实车采集到的车身信息
VEHICLESTATUS m_VehicleStatus;			// 经过平滑处理的车体状态
Uint16 m_iHighSpeedKeepingTime;         // ？？？？？？

int m_iSpeedBuff[8];
int m_iSpeedBuffIndex;
int m_iSpeedBuffCnt;

int m_iPositionBuff[4];
int m_iPosBuffIndex;
int m_iPosBuffCnt;

float m_fOrientationBuff[4];
int m_iOriBuffIndex;
int m_iOriBuffCnt;


Uint8 m_iWarningLevel;				    // 报警等级，数值越大，报警级别越高，0表示误无报警；
Uint8	m_iWarningEnabled;				// 0-报警被屏蔽， 1-报警功能启动
Uint8	m_iWorkingEnabled;
VEHICLEPROPERTY m_VehicleProperty;
//VEHICLESTATUS m_VechileStatus;

int m_iWarningCnt;				// 报警帧数累计数量，超过阈值则触发报警
float m_fTLCTime;				// TLC时间单位为秒

Bool m_bLaneWidthOK;
//Uint16 m_iWarningCounter;

int m_iPosition;
float m_fOrientation;

void ClearVehicleStatus(VEHICLESTATUS* pVS)
{
	pVS->Brake = 0;
	pVS->CarAccelerate = 0;
	pVS->CarLfLight = 0;
	pVS->CarRtLight = 0;
	pVS->CarSpeed = 0;
}


//Init Warning
//CWarning()
void CWarning(void)
{
	m_iWarningLevel = 0;	// 数值越大，报警级别越高，0表示误无报警；
	m_iWarningEnabled = 0;	// 0-报警被屏蔽， 1-报警功能启动
	ClearVehicleStatus(&m_VehicleStatus);
	ClearVehicleStatus(&m_InputVehicleStatus);
	m_VehicleProperty.tread = 860;
	m_iWorkingEnabled = 0;
	m_iHighSpeedKeepingTime = 0;
}

//CLDWWarning()
void CLDWWarning(void)
{
	m_iSpeedBuffIndex = 0;
	m_iSpeedBuffCnt = 0;

	m_iPosBuffIndex = 0;
	m_iPosBuffCnt = 0;

	m_iOriBuffIndex = 0;
	m_iOriBuffCnt = 0;

	m_iWarningCnt = 0;
	m_bLaneWidthOK = FALSE;

	m_fTLCTime = 0;
	m_iWarningCounter = 0;
}


//
//UpdateBuff()
//对输入的车速做平均
int UpdateBuff(int* buf, int iCnt, int iValue, int* iCurIndex, int* iCurCnt)
{
	int sum = 0;
	int ret;
	int i;

	if(*iCurCnt == iCnt)
	{
		buf[*iCurIndex] = iValue;
		for(i=0; i<iCnt; i++)
		{
			sum+=buf[i];
		}
		ret = (int)(sum/iCnt);
	}
	else
	{
		buf[*iCurIndex] = iValue;
		ret = iValue;
	}

	(*iCurIndex)++;
	if(*iCurIndex>iCnt-1)
	{
		*iCurIndex = 0;
		*iCurCnt = iCnt;
	}
	return ret;
}


/************************************************************************************************************
函数名: UpdateVehicleStatus()
注释:   更新车身信息
输入:	pVechileStatus	   输入的当前车身信息
输出:	m_VehicleStatus	   输出经过处理的车身信息
*************************************************************************************************************/
Bool UpdateVehicleStatus(VEHICLESTATUS* pVechileStatus)
{
	if (pVechileStatus->Brake == VEHICLE_SIGNAL_LIFE)
	{
		m_VehicleStatus.Brake = pVechileStatus->Brake;
	}
	else
	{
		m_VehicleStatus.Brake = max(m_VehicleStatus.Brake-1, 0);
	}

	if (pVechileStatus->CarLfLight == VEHICLE_SIGNAL_LIFE)
	{
		m_VehicleStatus.CarLfLight = pVechileStatus->CarLfLight;
	}
	else
	{
		m_VehicleStatus.CarLfLight = max(m_VehicleStatus.CarLfLight-1, 0);
	}

	if (pVechileStatus->CarRtLight == VEHICLE_SIGNAL_LIFE)
	{
		m_VehicleStatus.CarRtLight = pVechileStatus->CarRtLight;
	}
	else
	{
		m_VehicleStatus.CarRtLight = max(m_VehicleStatus.CarRtLight-1, 0);
	}

	m_VehicleStatus.CarSpeed = UpdateBuff(m_iSpeedBuff, 8, pVechileStatus->CarSpeed, &m_iSpeedBuffIndex, &m_iSpeedBuffCnt); //对车速进行滤波

	if(m_VehicleStatus.CarSpeed > LDW_WARNING_ENABLE_SPEED)
	{
		m_iHighSpeedKeepingTime ++;
	}
	else
	{
		m_iHighSpeedKeepingTime = 0;
	}

	return TRUE;
}


void SetWorkingEnable(void)
{
	if(m_VehicleStatus.CarSpeed > LDW_WORKING_ENABLE_SPEED)
	{
		m_iWorkingEnabled = min(m_iWorkingEnabled+1, 2);
	}
	else
	{
		m_iWorkingEnabled = max(m_iWorkingEnabled-1, 0);
	}
}


//SetWarningEnable()
void SetWarningEnable(void)
{
	if(m_iWorkingEnabled)
	{
		if(m_iHighSpeedKeepingTime<2 || m_VehicleStatus.CarRtLight|| m_VehicleStatus.CarLfLight || m_VehicleStatus.Brake)
		{
#ifdef ENABLE_VEHICLE_SIGNAL
			m_iWarningEnabled = max(m_iWarningEnabled-1, 0);
#else
			m_iWarningEnabled = 2;
#endif
		}
		else
		{
			m_iWarningEnabled = min(m_iWarningEnabled+1, 2);
		}
	}
	else
	{
		m_iWarningEnabled = 0;
	}

}


//TLC()
//iPos:单位毫米
Uint8 TLC(int iPos, float fOri, float fThresh, int iSpeed, int iDepartThresh)
{
	int iDeparturespeed;
	float fDepartTime;

	if (iSpeed<LDW_WORKING_ENABLE_SPEED)
	{
		m_fTLCTime = 0;
		return 0;
	}

	if(fOri>0.005) //向右偏离
	{
		if(iPos<LANE_WIDTH_WORLD/2)
		{
			return 0;
		}
		iDeparturespeed = (int)(iSpeed*tan(fOri));
		fDepartTime = (float)(iDepartThresh+LANE_WIDTH_WORLD/2-iPos)/iDeparturespeed;
		m_fTLCTime = fDepartTime;
	}

	else if(fOri<-0.005) //向左偏离
	{
		if(iPos>LANE_WIDTH_WORLD/2)
		{
			return 0;
		}
		iDeparturespeed = (int)(-iSpeed*tan(fOri));
		fDepartTime = (float)(iPos-(LANE_WIDTH_WORLD/2-iDepartThresh))/iDeparturespeed;
		m_fTLCTime = fDepartTime;
	}
	else
	{
		return 0;
	}
	if(fDepartTime<fThresh)
	{
		if(fOri<0)   //左偏
		{
			return 1;
		}
		else        //右偏
		{
			return 2;
		}
	}
	else
	{
		return 0;
	}
}

//CPP()
Uint8 CPP(int iPos, float fOri, int iThresh, Bool bCrossMark)
{
	if(iPos>iThresh+LANE_WIDTH_WORLD/2)
	{
		return 2;
	}
	else if(iPos<LANE_WIDTH_WORLD/2-iThresh)
	{
		return 1;
	}
	else
	{
		return 0;
	}

}

//fUpdateBuff()
float fUpdateBuff(float* buf, int iCnt, float fValue, int* iCurIndex, int* iCurCnt)
{
	float sum;
	float ret;
	int i;
	sum = 0;
	if(*iCurCnt == iCnt)
	{
		buf[*iCurIndex] = fValue;
#if 0
		for(i=0; i<iCnt; i++)
		{
			sum+=buf[i];
		}
#else
       i = iCnt;
	   while (i)
	   {
	      i--;
		  sum+=buf[i];
	   }
#endif
		ret = (float)(sum/iCnt);
	}
	else
	{
		buf[*iCurIndex] = fValue;
		ret = fValue;
	}

	(*iCurIndex)++;
	if(*iCurIndex>iCnt-1)
	{
		*iCurIndex = 0;
		*iCurCnt = iCnt;
	}
	return ret;
}


//UpdateVehiclePO()
Bool UpdateVehiclePO(int P, float O, Bool bCrossMark)
{
	m_iPosition = UpdateBuff(m_iPositionBuff, 4, P, &m_iPosBuffIndex, &m_iPosBuffCnt);

	if ((max(abs(m_iPosition),abs(P))) > (min(abs(m_iPosition),abs(P)) * 5))
	{
		m_iPosition = P;
	}
	if(bCrossMark)
	{
		m_iPosition = P;
	}
	m_fOrientation = fUpdateBuff(m_fOrientationBuff, 4, O, &m_iOriBuffIndex, &m_iOriBuffCnt);
	return TRUE;
}


/************************************************************************************************************
函数名: LDW_Warning()
注释: 报警入口函数
输入:	VehicleStatus	车体状态信息
		iPos			车辆中心距离左侧车道线距离
		fOri			车道线角度
		bTrust			本帧车道线测量结果是否可信
		bCrossMark		车体是否正在跨越车道线
输出:	m_iWarningLevel	报警级别，大于0则表示报警
*************************************************************************************************************/
DepartureState LDW_Warning(VEHICLESTATUS* VehicleStatus, int iPos, float fOri, Bool bTrust, Bool bCrossMark)
{
	DepartureState DpState;
	Uint8 TLCRe = 0;
	Uint8 CPPRe = 0;
	m_iWarningCounter = (m_iWarningCounter>0)? m_iWarningCounter-1:0;
	
	UpdateVehicleStatus(VehicleStatus);			    // 更新车体状态
	UpdateVehiclePO(iPos, fOri, bCrossMark);		// 更新位置信息

	SetWorkingEnable();								// 根据车体状态设置报警模块是否工作
	SetWarningEnable();								// 根据车体状态和系统工作状态设置报警使能

	if(m_iWarningEnabled)							// 如报警使能
	{
		if(bTrust)									// 如测量结果可信
		{
			//if(m_bLaneWidthOK)						// 如宽度正确
			{
			    //m_iCurWarningMethod = 1;
				TLCRe = TLC(m_iPosition, m_fOrientation, 1.5, m_VehicleStatus.CarSpeed, LANE_WIDTH_WORLD / 2 - m_VehicleProperty.tread);
				CPPRe = 0; //CPP(m_iPosition, m_fOrientation, LANE_WIDTH_WORLD / 2 - m_VehicleProperty.tread, bCrossMark);
				if(TLCRe || CPPRe)	// 如果符合报警条件
				{
					m_iWarningCnt = min(m_iWarningCnt+1, 3);		// 报警帧数计数器加1.
				}
				else												// 否则清零
				{
					m_iWarningCnt = 0;
				}
			}
			/*
			else									// 如宽度不正确
			{
				if(CPP(m_iPosition, m_fOrientation, LANE_WIDTH_WORLD/2-m_VehicleProperty.tread, bCrossMark))		// 采用CPP算法
				{
					m_iWarningCnt = min(m_iWarningCnt+1, 3);
				}
				else
				{
					m_iWarningCnt = 0;
				}
			}
			// */
		}
		else										    // 当前帧不可信
		{
			m_iWarningCnt = max(m_iWarningCnt-1, 0);	// 报警帧数计数器减1.
		}

		if(m_iWarningCnt>2)							// 报警帧数计数器超过阈值
		{
			m_iWarningLevel = 1;					// 报警
			m_iWarningCounter = (m_iWarningCounter>0)? m_iWarningCounter:LDW_WARNING_LENGTH;
		}
		else
		{
			m_iWarningLevel = 0;					// 不报警
		}
	}
	else											// 报警模块不工作
	{
		m_iWarningCnt = 0;
		m_iWarningLevel = 0;
	}

	DpState.DepartureState = m_iWarningLevel;
	DpState.DepartureDir = TLCRe | CPPRe;

	return DpState;

}








