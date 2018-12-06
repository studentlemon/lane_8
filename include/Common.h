
#ifndef __COMMON__
#define __COMMON__

#define FALSE               0
#define TRUE                1
#define LDW_WARNING_LENGTH 17   //报警持续的次数
#define VEHICLE_SIGNAL_LIFE 50  //车辆信号有效时间，在计算车身信号时也会遇到
#define LANE_WIDTH_WORLD 3750   //世界坐标下车道线宽度
#define ENABLE_VEHICLE_SIGNAL

#define LDW_WARNING_ENABLE_SPEED 11111          // (int)(40*10000/36)
#define LDW_WORKING_ENABLE_SPEED 8333           // (int)(30*10000/36) mm/s
#define LDW_MAX_ACCElERATE 300                  // Max acceleration in frame in mm
 
#define L_PI 3.14159265358979323846

 #ifndef min
	#define min(a,b) (((a)<(b))?(a):(b))
 #endif

 #ifndef max
	#define max(a,b) (((a)>(b))?(a):(b))
 #endif

typedef unsigned char       Uint8;    /* unsigned 8 bit integer    */
typedef unsigned short      Uint16;   /* unsigned 16 bit integer   */
typedef unsigned int        Uint32;   /* unsigned 32 bit integer   */
typedef char                Int8;     /* signed 8 bit integer      */
typedef short               Int16;    /* signed 16 bit integer     */
typedef int                 Int32;    /* signed 32 bit integer     */
typedef float               float32;  /* 32 bit floating point     */
typedef double              float64;  /* 64 bit floating point     */
typedef unsigned char       Bool;     /* bool */

#endif 




