#ifndef __LDW__
#define __LDW__

#include "Common.h"
//#define LDW_AND_FCW   // 2012.06.31 added


//宏定义
#define PI 3.141592653589793f
#define VEHICLE_SIGNAL_LIFE 50  //车辆信号有效时间
#define LWD_MAX_LANE_ANGLE (3.5/180*PI)
#define LANE_WIDTH_WORLD 3750   //世界坐标下车道线宽度

#define TIME_INTERVAL 0.04 //图像采集输入的时间间隔，可能做不到，要根据算法速度修改

#define OFFSET_LANE_WIDTH_WORLD 350 //统一车道宽度有效范围

#ifndef min
#define min(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef max
#define max(a,b) (((a)>(b))?(a):(b))
#endif

//数据类型定义
struct sCPoint
{
    int x;
    int y;
};
typedef struct sCPoint CPoint;

struct sTRAPEZOID{						// 不支持负数
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
	Uint8 bValidate;	// 跟踪有效期 

	float k;		    // 斜率
	float b;		    // 截距
	Uint8 trust;
	
	CPoint EPoint;      // 直线终点	Top
	CPoint SPoint;      // 直线起点	Bottom
	Bool bVert;		    // 是否竖直
	Uint16 PointCnt;	// 点数
	Uint32 degree;	    // 可信度
//	Bool bLastValid;	// 上帧是否有效
//	Uint8 iWarningLevel;

    unsigned int iFailure; //连续失败的帧数 20130308 changan

	Bool RefOtherSide;

	int Angle;
	char FlagWarning;
};
typedef struct sLineFunc LineFunc;

struct sDepartureState
{
	Bool DepartureState;  //表明是否发生偏离，0：没有发生偏离；1：发生偏离
	Uint8 DepartureDir;   //表明偏离方向，0：没有偏离；1：向左偏离；2：向右偏离
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
	Uint8 CarLight;         //车灯信号，指示近光灯车灯是否打开的信号
	Uint8 CarWiper;         //雨刷信号

	Int16 SteeringAngle;      //方向盘转角
	Uint8 SteeringAngleSpeed; //方向盘转角速率
	Uint8 FogLight;           //雾灯信号，如果打开雾灯，说明当前有雾
	Uint8 AccPedalPosition;   //加速踏板位置，0 - 100
	Uint8 LowBeam;
	Uint8 HighBeam;
	Uint8 PositionLamp;       //位置灯
	Uint8 ActualGear;         //实际档位
};
typedef struct sVEHICLE_STATUS VEHICLESTATUS;

struct sVEHICLE_PROPERTY
{
	Uint16 tread;  //轮距
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

//定义车道线识别结果相关的数据结构
struct sCCanLaneRes
{
	Uint8 ObjectNum;          //障碍物个数
	Uint8 TypeofLeftLane;       //左车道线类型,00:没有识别到车道线，01虚线，10实线，11双实线
	Uint8 TypeofRightLane;      //右车道线类型,00:没有识别到车道线，01虚线，10实线，11双实线
	Uint8 QualityOfLane;       //车道线质量
	Uint8 IsCrossingLane;       //是否跨越车道（00未跨越，01右侧，10左侧）
	Int16 Dist2LeftLane;       //车辆距离左车道线的距离
	Int16 Dist2RightLane;      //车辆距离右车道线的距离
	Uint16 Dist2NextLeftLane;    //车辆距离临左车道线的距离
	Uint16 Dist2NextRightLane;   //车辆距离临右车道线的距离
	Int16 AngleofVehicle2Lane;   //车辆与车道线的夹角：单位：deg/10，方向：右正左负
	Uint16 CurvOfLane;        //车道线曲率
	Uint8  DerivOfLaneCurv;    //车道线曲率导数
};
typedef struct sCCanLaneRes CCanLaneRes;

//定义障碍物识别相关的数据结构
struct sCCanObjectRes
{
	Uint16 TargetVDist;        //目标物纵向距离，单位：m/100
	Int16 TargetHDist;        //目标物横向距离，单位：m/100，方向：右正左负
	Uint8  TypeofTarget;       //障碍物类型，1000：小汽车，0001：货车,0010：行人，0011：三轮车，0100：公交车，0101：信号灯，0110：标志牌
	Int16  TargetRelSpeed;      //目标物相对速度，+：远离；-：靠近
	Uint8  Time2Crash;        //碰撞时间TTC
};
typedef struct sCCanObjectRes CCanObjectRes;



struct  sMaxValue
{
	Uint32 value;           //最大值
	Int16 Dist;             //距离
	Uint16 AngleNumber;     //角度
} ;
typedef struct sMaxValue MaxValue;

// FCW，LDW工作状态信号
typedef enum eStateTypeTag
{
	STATE_CLOSE   = 0,    //关闭
	STATE_STANDBY = 1,    //待机
	STATE_WORK    = 2,    //工作
	STATE_NO_WORK = 3,    //非工作模式
	RESERVED              //保留模式
} eStateType;

typedef enum eLDWWarningTypeTag
{
	LDW_NO_WARNING = 0, //不发出任何报警
	LDW_LEFT_WARNING_IV = 1, //左侧发出图像和语音报警
	LDW_LEFT_WARNING_I = 2, //报警时间过长，只发出图像报警，不发出语音报警
	LDW_RIGHT_WARNING_IV = 3, //右侧发出图像和语音报警
	LDW_RIGHT_WARNING_I = 4,  //右侧只发出图像报警
	LDW_WARNING_RESERVED

}eLDWWarningType;



// FCW报警类型
typedef enum eCarWarningTypeTag
{
	NO_WARNING = 0,
	WARN_NEAR_DISTANCE   = 1, //近距离报警
	REMOVE_NEAR_DISTANCE = 2, //近距离报警解除
	WARN_PRE_COLLISIOM_WARNING   = 3, //预报警：第一次碰撞提示报警，一级报警
	ReMOVE_PRE_COLLISIOM_WARNING = 4, //预报警解除
	WARN_COLLISION_WARNING   = 5,     //碰撞报警，也就是危险报警
	REMOVE_COLLISION_WARNING = 6      //碰撞报警解除
} eCarWarningType;

void func_lane_detect();

#endif


