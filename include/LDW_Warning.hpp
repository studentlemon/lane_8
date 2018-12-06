//#include "LDW_Warning.h"
#include "Common.h"
#include "LDW.h"
#include "math.h"



Uint16 m_iWarningCounter;               // ��������
VEHICLESTATUS m_InputVehicleStatus;		// ��ǰʵ���ɼ����ĳ�����Ϣ
VEHICLESTATUS m_VehicleStatus;			// ����ƽ������ĳ���״̬
Uint16 m_iHighSpeedKeepingTime;         // ������������

int m_iSpeedBuff[8];
int m_iSpeedBuffIndex;
int m_iSpeedBuffCnt;

int m_iPositionBuff[4];
int m_iPosBuffIndex;
int m_iPosBuffCnt;

float m_fOrientationBuff[4];
int m_iOriBuffIndex;
int m_iOriBuffCnt;


Uint8 m_iWarningLevel;				    // �����ȼ�����ֵԽ�󣬱�������Խ�ߣ�0��ʾ���ޱ�����
Uint8	m_iWarningEnabled;				// 0-���������Σ� 1-������������
Uint8	m_iWorkingEnabled;
VEHICLEPROPERTY m_VehicleProperty;
//VEHICLESTATUS m_VechileStatus;

int m_iWarningCnt;				// ����֡���ۼ�������������ֵ�򴥷�����
float m_fTLCTime;				// TLCʱ�䵥λΪ��

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
	m_iWarningLevel = 0;	// ��ֵԽ�󣬱�������Խ�ߣ�0��ʾ���ޱ�����
	m_iWarningEnabled = 0;	// 0-���������Σ� 1-������������
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
//������ĳ�����ƽ��
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
������: UpdateVehicleStatus()
ע��:   ���³�����Ϣ
����:	pVechileStatus	   ����ĵ�ǰ������Ϣ
���:	m_VehicleStatus	   �����������ĳ�����Ϣ
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

	m_VehicleStatus.CarSpeed = UpdateBuff(m_iSpeedBuff, 8, pVechileStatus->CarSpeed, &m_iSpeedBuffIndex, &m_iSpeedBuffCnt); //�Գ��ٽ����˲�

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
//iPos:��λ����
Uint8 TLC(int iPos, float fOri, float fThresh, int iSpeed, int iDepartThresh)
{
	int iDeparturespeed;
	float fDepartTime;

	if (iSpeed<LDW_WORKING_ENABLE_SPEED)
	{
		m_fTLCTime = 0;
		return 0;
	}

	if(fOri>0.005) //����ƫ��
	{
		if(iPos<LANE_WIDTH_WORLD/2)
		{
			return 0;
		}
		iDeparturespeed = (int)(iSpeed*tan(fOri));
		fDepartTime = (float)(iDepartThresh+LANE_WIDTH_WORLD/2-iPos)/iDeparturespeed;
		m_fTLCTime = fDepartTime;
	}

	else if(fOri<-0.005) //����ƫ��
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
		if(fOri<0)   //��ƫ
		{
			return 1;
		}
		else        //��ƫ
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
������: LDW_Warning()
ע��: ������ں���
����:	VehicleStatus	����״̬��Ϣ
		iPos			�������ľ�����೵���߾���
		fOri			�����߽Ƕ�
		bTrust			��֡�����߲�������Ƿ����
		bCrossMark		�����Ƿ����ڿ�Խ������
���:	m_iWarningLevel	�������𣬴���0���ʾ����
*************************************************************************************************************/
DepartureState LDW_Warning(VEHICLESTATUS* VehicleStatus, int iPos, float fOri, Bool bTrust, Bool bCrossMark)
{
	DepartureState DpState;
	Uint8 TLCRe = 0;
	Uint8 CPPRe = 0;
	m_iWarningCounter = (m_iWarningCounter>0)? m_iWarningCounter-1:0;
	
	UpdateVehicleStatus(VehicleStatus);			    // ���³���״̬
	UpdateVehiclePO(iPos, fOri, bCrossMark);		// ����λ����Ϣ

	SetWorkingEnable();								// ���ݳ���״̬���ñ���ģ���Ƿ���
	SetWarningEnable();								// ���ݳ���״̬��ϵͳ����״̬���ñ���ʹ��

	if(m_iWarningEnabled)							// �籨��ʹ��
	{
		if(bTrust)									// ������������
		{
			//if(m_bLaneWidthOK)						// ������ȷ
			{
			    //m_iCurWarningMethod = 1;
				TLCRe = TLC(m_iPosition, m_fOrientation, 1.5, m_VehicleStatus.CarSpeed, LANE_WIDTH_WORLD / 2 - m_VehicleProperty.tread);
				CPPRe = 0; //CPP(m_iPosition, m_fOrientation, LANE_WIDTH_WORLD / 2 - m_VehicleProperty.tread, bCrossMark);
				if(TLCRe || CPPRe)	// ������ϱ�������
				{
					m_iWarningCnt = min(m_iWarningCnt+1, 3);		// ����֡����������1.
				}
				else												// ��������
				{
					m_iWarningCnt = 0;
				}
			}
			/*
			else									// ���Ȳ���ȷ
			{
				if(CPP(m_iPosition, m_fOrientation, LANE_WIDTH_WORLD/2-m_VehicleProperty.tread, bCrossMark))		// ����CPP�㷨
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
		else										    // ��ǰ֡������
		{
			m_iWarningCnt = max(m_iWarningCnt-1, 0);	// ����֡����������1.
		}

		if(m_iWarningCnt>2)							// ����֡��������������ֵ
		{
			m_iWarningLevel = 1;					// ����
			m_iWarningCounter = (m_iWarningCounter>0)? m_iWarningCounter:LDW_WARNING_LENGTH;
		}
		else
		{
			m_iWarningLevel = 0;					// ������
		}
	}
	else											// ����ģ�鲻����
	{
		m_iWarningCnt = 0;
		m_iWarningLevel = 0;
	}

	DpState.DepartureState = m_iWarningLevel;
	DpState.DepartureDir = TLCRe | CPPRe;

	return DpState;

}








