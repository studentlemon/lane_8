#ifndef __LDW__
#define __LDW__

#include "Common.h"
//#define LDW_AND_FCW   // 2012.06.31 added


//�궨��
#define PI 3.141592653589793f
#define VEHICLE_SIGNAL_LIFE 50  //�����ź���Чʱ��
#define LWD_MAX_LANE_ANGLE (3.5/180*PI)
#define LANE_WIDTH_WORLD 3750   //���������³����߿��

#define TIME_INTERVAL 0.04 //ͼ��ɼ������ʱ������������������Ҫ�����㷨�ٶ��޸�

#define OFFSET_LANE_WIDTH_WORLD 350 //ͳһ���������Ч��Χ

#ifndef min
#define min(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef max
#define max(a,b) (((a)>(b))?(a):(b))
#endif

//�������Ͷ���
struct sCPoint
{
    int x;
    int y;
};
typedef struct sCPoint CPoint;

struct sTRAPEZOID{						// ��֧�ָ���
	Int16 top;
	Int16 bottom;
	Int16 leftbottom;
	Int16 rightbottom;
	Int16 lefttop;
	Int16 righttop;
};
typedef struct sTRAPEZOID TRAPEZOID;

struct sCFPoint
{
	float x;
	float y;
};
typedef struct sCFPoint CFPoint;

struct sCLine
{
	int x1;
	int y1;
	int x2;
	int y2;
};
typedef struct sCLine CLine;

struct sCFLine
{
	float x1;
	float y1;
	float x2;
	float y2;
};
typedef struct sCFLine CFLine;

struct sLineFunc
{
	Uint8 bValidate;	// ������Ч�� 

	float k;		    // б��
	float b;		    // �ؾ�
	Uint8 trust;
	
	CPoint EPoint;      // ֱ���յ�	Top
	CPoint SPoint;      // ֱ�����	Bottom
	Bool bVert;		    // �Ƿ���ֱ
	Uint16 PointCnt;	// ����
	Uint32 degree;	    // ���Ŷ�
//	Bool bLastValid;	// ��֡�Ƿ���Ч
//	Uint8 iWarningLevel;

    unsigned int iFailure; //����ʧ�ܵ�֡�� 20130308 changan

	Bool RefOtherSide;

	int Angle;
	char FlagWarning;
};
typedef struct sLineFunc LineFunc;

struct sDepartureState
{
	Bool DepartureState;  //�����Ƿ���ƫ�룬0��û�з���ƫ�룻1������ƫ��
	Uint8 DepartureDir;   //����ƫ�뷽��0��û��ƫ�룻1������ƫ�룻2������ƫ��
};
typedef struct sDepartureState DepartureState;

struct sVEHICLE_STATUS 
{
	Uint8 CarLfLight;		// 0-nonlight, 1- right headlight, 2- left headlight...
	Uint8 CarRtLight;
	Uint8 Brake;			// 0-nonbrake, 1-brake

	Uint8 CarLfLightS;		// 0-nonlight, 1- right headlight, 2- left headlight...
	Uint8 CarRtLightS;
	Uint8 BrakeS;			// 0-nonbrake, 1-brake

	Uint16 CarSpeed;		// in mm/s		
	Uint16 CarAccelerate;	// in mm/s^2
	Uint8 CarLight;         //�����źţ�ָʾ����Ƴ����Ƿ�򿪵��ź�
	Uint8 CarWiper;         //��ˢ�ź�

	Int16 SteeringAngle;      //������ת��
	Uint8 SteeringAngleSpeed; //������ת������
	Uint8 FogLight;           //����źţ��������ƣ�˵����ǰ����
	Uint8 AccPedalPosition;   //����̤��λ�ã�0 - 100
	Uint8 LowBeam;
	Uint8 HighBeam;
	Uint8 PositionLamp;       //λ�õ�
	Uint8 ActualGear;         //ʵ�ʵ�λ
};
typedef struct sVEHICLE_STATUS VEHICLESTATUS;

struct sVEHICLE_PROPERTY
{
	Uint16 tread;  //�־�
};
typedef struct  sVEHICLE_PROPERTY VEHICLEPROPERTY;


struct sCRect
{
	int left;
	int top;
	int right;
	int bottom;
};
typedef struct sCRect CRect;

//���峵����ʶ������ص����ݽṹ
struct sCCanLaneRes
{
	Uint8 ObjectNum;          //�ϰ������
	Uint8 TypeofLeftLane;       //�󳵵�������,00:û��ʶ�𵽳����ߣ�01���ߣ�10ʵ�ߣ�11˫ʵ��
	Uint8 TypeofRightLane;      //�ҳ���������,00:û��ʶ�𵽳����ߣ�01���ߣ�10ʵ�ߣ�11˫ʵ��
	Uint8 QualityOfLane;       //����������
	Uint8 IsCrossingLane;       //�Ƿ��Խ������00δ��Խ��01�Ҳ࣬10��ࣩ
	Int16 Dist2LeftLane;       //���������󳵵��ߵľ���
	Int16 Dist2RightLane;      //���������ҳ����ߵľ���
	Uint16 Dist2NextLeftLane;    //�����������󳵵��ߵľ���
	Uint16 Dist2NextRightLane;   //�����������ҳ����ߵľ���
	Int16 AngleofVehicle2Lane;   //�����복���ߵļнǣ���λ��deg/10������������
	Uint16 CurvOfLane;        //����������
	Uint8  DerivOfLaneCurv;    //���������ʵ���
};
typedef struct sCCanLaneRes CCanLaneRes;

//�����ϰ���ʶ����ص����ݽṹ
struct sCCanObjectRes
{
	Uint16 TargetVDist;        //Ŀ����������룬��λ��m/100
	Int16 TargetHDist;        //Ŀ���������룬��λ��m/100������������
	Uint8  TypeofTarget;       //�ϰ������ͣ�1000��С������0001������,0010�����ˣ�0011�����ֳ���0100����������0101���źŵƣ�0110����־��
	Int16  TargetRelSpeed;      //Ŀ��������ٶȣ�+��Զ�룻-������
	Uint8  Time2Crash;        //��ײʱ��TTC
};
typedef struct sCCanObjectRes CCanObjectRes;



struct  sMaxValue
{
	Uint32 value;           //���ֵ
	Int16 Dist;             //����
	Uint16 AngleNumber;     //�Ƕ�
} ;
typedef struct sMaxValue MaxValue;

// FCW��LDW����״̬�ź�
typedef enum eStateTypeTag
{
	STATE_CLOSE   = 0,    //�ر�
	STATE_STANDBY = 1,    //����
	STATE_WORK    = 2,    //����
	STATE_NO_WORK = 3,    //�ǹ���ģʽ
	RESERVED              //����ģʽ
} eStateType;

typedef enum eLDWWarningTypeTag
{
	LDW_NO_WARNING = 0, //�������κα���
	LDW_LEFT_WARNING_IV = 1, //��෢��ͼ�����������
	LDW_LEFT_WARNING_I = 2, //����ʱ�������ֻ����ͼ�񱨾�����������������
	LDW_RIGHT_WARNING_IV = 3, //�Ҳ෢��ͼ�����������
	LDW_RIGHT_WARNING_I = 4,  //�Ҳ�ֻ����ͼ�񱨾�
	LDW_WARNING_RESERVED

}eLDWWarningType;



// FCW��������
typedef enum eCarWarningTypeTag
{
	NO_WARNING = 0,
	WARN_NEAR_DISTANCE   = 1, //�����뱨��
	REMOVE_NEAR_DISTANCE = 2, //�����뱨�����
	WARN_PRE_COLLISIOM_WARNING   = 3, //Ԥ��������һ����ײ��ʾ������һ������
	ReMOVE_PRE_COLLISIOM_WARNING = 4, //Ԥ�������
	WARN_COLLISION_WARNING   = 5,     //��ײ������Ҳ����Σ�ձ���
	REMOVE_COLLISION_WARNING = 6      //��ײ�������
} eCarWarningType;

void func_lane_detect();

#endif


