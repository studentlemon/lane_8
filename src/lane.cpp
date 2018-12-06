#include <iostream>
#include <deque>
#include <boost/thread/thread.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/bind.hpp>
#include <string>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <math.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include "lane_detection.hpp"
#include "can_serial.hpp"

#include "../include/LDW_Warning.hpp"
#include "../include/LDW.h"
#include "../include/LDW_Measure.hpp"

#define L_PI 3.14159265358979323846
#define WARNING
#define y480distance 276

using namespace std;
using namespace cv;

//-----------------初始化传入可执行文件的参数,给定参数说明和默认值,详见run.sh--------------------
DEFINE_string(model_file, "",
    "The model file used to initialize network architecture (*.prototxt).");
DEFINE_string(weights_file, "",
    "The weights file used to initialize network parameter (*.caffemodel)");
DEFINE_int32(camera_id, 0,
    "The camera ID");
DEFINE_string(ipm_file, "",
    "The ipm file");
DEFINE_string(output_video, "",
    "The output video file");
DEFINE_double(confidence_threshold, 0.2,
    "Only store detections with score higher than the threshold.");
DEFINE_int32(width, 0, "Set image width");
DEFINE_int32(height, 0, "Set image height");
DEFINE_int32(fps, 0, "Set capture fps");
DEFINE_int32(DEBUG, 0, "Whether need to debug");
DEFINE_int32(SERIAL_OPEN, 1, "Whether need to open serial port");
DEFINE_int32(VIDEO_STORE, 0, "Whether need to store video");

//-------------------------用于存储通过gflag传入的值-------------------------------
string output_video, IPM_file;
int camera_id, im_width, im_height, im_fps, _DEBUG, _SERIAL_OPEN, _VIDEO_STORE;

//-------------------------中间信息打印至文件,用于调试-------------------------------
ofstream location_out;
//location_out.open("location_out.txt",std::ios::out | std::ios::app);
ofstream location_out_new;
//location_out_new.open("location_out_new.txt");
ofstream save_distance("result.txt");
ofstream buffer("buffer.txt");

//串口数据传输部分变量
String SerialLaneDetect;
int fd2, nread2, i2;

//global variable for sharing between threads
bool DETECT_FLAG = true;
bool OVER = false;//视频流结束标志
vector<cv::Point> lanes; //理想车道线识别结果（四个坐标）
back_lines final_lanes;//车道线识别返回值
cv::Mat img_temp;
cv::Mat frame_global, frame, canvas, canvas_detect, road, img;
bool NEW_DETECT = false;
bool TRACK_FLAG = false;
bool NEW_IMAGE = false;

//Crop image region
Rect crop_region = Rect(Point(0,0), Point(640,480));

vector<int> left_distance_new;//离左车道线的距离
vector<int> right_distance_new;//离右车道线的距离

extern VEHICLESTATUS m_InputVehicleStatus;
extern VEHICLEPROPERTY m_VehicleProperty;
Bool m_bCrossMark;
int m_iCrossMarkSideNum; //0:Left side; 1:right side; 
int m_iNearestMark;
LineFunc m_LineLf;
LineFunc m_LineRt;
int warning_level;
DepartureState DprtSta;

double _ANGLEl;		//add by nh
double _ANGLEr;		//add by nh

//-------------存储左右车道线的坐标位置------------
double g_LeftLane_x0, g_LeftLane_x1;
double g_LeftLane_y0, g_LeftLane_y1;
double g_RightLane_x2, g_RightLane_x3;
double g_RightLane_y2, g_RightLane_y3;
double r_distance,l_distance;
double r_distance1,l_distance1;
double r_distance2,l_distance2;

//-------------根据识别结果需计算的参数---------------
Uint8 g_objectnum;//目标数量
Uint8 g_typeofleftlane;//左车道线类型
Uint8 g_typeofrightlane;//右车道线类型
Uint8 g_qualityoflane;//车道线质量
Uint8 g_iscrossinglane;//是否换道
Int16 g_dist2leftlane;//离左车道线的距离
Int16 g_dist2righttlane;//离右车道线的距离
Uint16 g_dist2nextleftlane;//离左边临车道的距离
Uint16 g_dist2nextrightlane;//离右边临车道的距离
Int16 g_angleofvehicle2lane;//本车与本车道的夹角
Uint16 g_curveoflane;//本车道车道线曲率
Uint8 g_derivoflanecurv;//本车道车道线曲率导数


//-----------------------------视频存储--------------------------------
VideoWriter roadVideo; //车道线视频
VideoWriter originalVideo;//原始视频
Size s;
int framecount = 0;//读入的图像帧数
//add from ipm start by nh 20171115
typedef struct IPMInfo
{
    //width
    int ipmwidth;
    //height
    int ipmheight;
    //portion of image height to add to y-coordinate of
    //vanishing point
    float vpPortion;
    //Left point in original image of region to make IPM for
    float ipmLeft;
    //Right point in original image of region to make IPM for
    float ipmRight;
    //Top point in original image of region to make IPM for
    float ipmTop;
    //Bottom point in original image of region to make IPM for
    float ipmBottom;
    //interpolation to use for IPM (0: bilinear, 1:nearest neighbor)
    int ipmInterpolation;
}IPMInfo;
//Camera Calibration info
typedef struct CameraInfo
{
    //focal length in x and y
    Point2f focalLength;
    //optical center coordinates in image frame (origin is (0,0) at top left)
    Point2f opticalCenter;
    //height of camera above ground
    float cameraHeight;
    //pitch angle in radians (+ve downwards)
    float pitch;
    //yaw angle in radians (+ve clockwise)
    float yaw;
    //width of images
    float imageWidth;
    //height of images
    float imageHeight;
}CameraInfo;

//=============================串口数据发送函数=====================================
void serial_send_lane(int fd2) {
    int nread2;
    nread2 = write(fd2, SerialLaneDetect.c_str(), SerialLaneDetect.length());
    cout<<"send_lane"<<endl;
    SerialLaneDetect = "";
}

string DoubleToString(double input_double) {
    ostringstream Oss;
    Oss << input_double;
    return Oss.str();
}

string IntToString(int input_int) {
    ostringstream Oss2;
    Oss2 << input_int;
    return Oss2.str();
}

Mat Transformimage2ground(Mat uv,CameraInfo cameraInfo)
{
    Mat inPoints4=(Mat_<float>(uv.rows+2,uv.cols));
    Mat inPoints3=(Mat_<float>(3,1)<<uv.at<float>(0,0),uv.at<float>(1,0),1);
    //cout<<inPoints3<<endl;
    float c1 = cos(cameraInfo.pitch*PI/180);
    float s1 = sin(cameraInfo.pitch*PI/180);
    float c2 = cos(cameraInfo.yaw*PI/180);
    float s2 = sin(cameraInfo.yaw*PI/180);
    Mat matp=(Mat_<float>(4,3)<<
                              -cameraInfo.cameraHeight*c2/cameraInfo.focalLength.x, cameraInfo.cameraHeight*s1*s2/cameraInfo.focalLength.y,  (cameraInfo.cameraHeight*c2*cameraInfo.opticalCenter.x/cameraInfo.focalLength.x)-(cameraInfo.cameraHeight *s1*s2* cameraInfo.opticalCenter.y/cameraInfo.focalLength.y) - cameraInfo.cameraHeight *c1*s2,
            cameraInfo.cameraHeight*s2 /cameraInfo.focalLength.x,  cameraInfo.cameraHeight *s1*c2 /cameraInfo.focalLength.y,
            (-cameraInfo.cameraHeight *s2* cameraInfo.opticalCenter.x/cameraInfo.focalLength.x)-(cameraInfo.cameraHeight *s1*c2*cameraInfo.opticalCenter.y /cameraInfo.focalLength.y) -cameraInfo.cameraHeight *c1*c2,0,
            cameraInfo.cameraHeight *c1 /cameraInfo.focalLength.y,(-cameraInfo.cameraHeight *c1* cameraInfo.opticalCenter.y /cameraInfo.focalLength.y) + cameraInfo.cameraHeight *s1,0,
            -c1 /cameraInfo.focalLength.y,(c1* cameraInfo.opticalCenter.y /cameraInfo.focalLength.y) - s1);
    inPoints4=matp*inPoints3;
    //cout<<inPoints4<<endl;
    float div=inPoints4.at<float>(3,0);
    //cout<<"div:"<<div<<endl;
    Mat inpoint4_row1 = Mat_<float>(1,1)<<inPoints4.at<float>(0,0)/div;
    //cout<<inpoint4_row1<<endl;
    Mat inpoint4_row2 = Mat_<float>(1,1)<<inPoints4.at<float>(1,0)/div;

    Mat xy =(Mat_<float>(2,1));
    vconcat(inpoint4_row1,inpoint4_row2,xy);
    return xy;
}
//===============================根据识别出车道线的类别绘制车道线===================================
void draw_detected_lane(Mat &frame_, back_lines &final_lanes, const int frameCount)
{
	Point2f tmpline1;
	Point2f tmpline2;
	Scalar leftlanecolor = Scalar(0, 100, 0);
	Scalar rightlanecolor = Scalar(0, 100, 0);


    if (frameCount > 1)
    {
        //---------------------------------绘制左车道线----------------------------------------
		if (final_lanes.left_solid_line == 0) //绘制实线
		{
			for (int i = 1; i < final_lanes.left_lines.size(); ++i) 
			{

				line(frame_, final_lanes.left_lines[i - 1] + Point2f(MY_boundingBox.tl()),
				      final_lanes.left_lines[i] + Point2f(MY_boundingBox.tl()), Scalar(0, 255, 0), 2, CV_AA);
			}
		}
		else if (final_lanes.left_solid_line == 1) //绘制虚线
		{
			for (int i = final_lanes.left_lines.size() - 1; i >= 30; i = i - 50) 
			{

				 line(frame_, final_lanes.left_lines[i] + Point2f(MY_boundingBox.tl()), 
				        final_lanes.left_lines[i - 30] + Point2f(MY_boundingBox.tl()), Scalar(0, 255, 0), 2, CV_AA);
			}
		}
		else if (final_lanes.left_solid_line == 2) //未识别
		{
			//PASS
		}

        //--------------------------------绘制右车道线--------------------------------------
		if (final_lanes.right_soild_line == 0) 
		{
			for (int i = 1; i < final_lanes.right_lines.size(); ++i) 
			{

				 line(frame_, final_lanes.right_lines[i - 1] + Point2f(MY_boundingBox.tl()),
				      final_lanes.right_lines[i] + Point2f(MY_boundingBox.tl()), Scalar(0, 255, 0), 2, CV_AA);
			}
		}
		else if (final_lanes.right_soild_line == 1) 
		{
			for (int i = final_lanes.right_lines.size() - 1; i >= 30; i = i - 50) 
			{


				 line(frame_, final_lanes.right_lines[i] + Point2f(MY_boundingBox.tl()),
				      final_lanes.right_lines[i - 30] + Point2f(MY_boundingBox.tl()), Scalar(0, 255, 0), 2, CV_AA);
			}
		}
		else if (final_lanes.right_soild_line == 2) 
		{
			//PASS
		}

        //----------------------------一边实线一边虚线---------------------------------
		if ((final_lanes.left_solid_line == 0 || final_lanes.left_solid_line == 1) &&
			(final_lanes.right_soild_line == 0 || final_lanes.right_soild_line == 1))
        {
//--------若未定义临车道识别------------
#ifndef SECOND_LINE_ANTI_VANISH
            if (final_lanes.left_second_valid == true)
            {
                line(frame_, final_lanes.second_lines[0] + Point2f(MY_boundingBox.tl()),
                     final_lanes.second_lines[1] + Point2f(MY_boundingBox.tl()), Scalar(200, 200, 200), 0.5, CV_AA);
            }

            if (final_lanes.right_second_valid == true)
            {
                line(frame_, final_lanes.second_lines[2] + Point2f(MY_boundingBox.tl()),
                     final_lanes.second_lines[3] + Point2f(MY_boundingBox.tl()), Scalar(0, 0, 200), 0.5, CV_AA);
            }//presumably adjacent lane line
#elif defined(SECOND_LINE_ANTI_VANISH)
            if (final_lanes.left_second_valid == true) {
                line(frame_, final_lanes.second_lines[0] + Point2f(MY_boundingBox.tl()),
                     final_lanes.second_lines[1] + Point2f(MY_boundingBox.tl()), Scalar(0, 0, 200), 0.5, CV_AA);
//				cout << "[Cond1]———————————————>" << second_line_vanish_counter << "<———————————————\n";
            } else if (final_lanes.left_second_valid ==false \
                and 0 <= second_line_vanish_counter
                and second_line_vanish_counter <= second_line_vanish_threshold) {
                second_line_vanish_counter += 1;
                line(frame_, final_lanes.second_lines[0] + Point2f(MY_boundingBox.tl()),
                     final_lanes.second_lines[1] + Point2f(MY_boundingBox.tl()), Scalar(0, 0, 200), 0.5, CV_AA);
//				cout << "[Cond2]———————————————>" << second_line_vanish_counter << "<———————————————\n";
            } else if (final_lanes.left_second_valid ==false \
                and second_line_vanish_counter > second_line_vanish_threshold){
                second_line_vanish_counter = 0;
//				cout << "[Cond3]———————————————>" << second_line_vanish_counter << "<———————————————\n";
            } else {
//				cout << "[Cond4]———————————————>" << second_line_vanish_counter << "<———————————————\n";
            }

            if (final_lanes.right_second_valid == true) {
                line(frame_, final_lanes.second_lines[2] + Point2f(MY_boundingBox.tl()),
                     final_lanes.second_lines[3] + Point2f(MY_boundingBox.tl()), Scalar(0, 0, 200), 0.5, CV_AA);
//				cout << "[Cond1]———————————————>" << second_line_vanish_counter << "<———————————————\n";
            } else if (final_lanes.left_second_valid ==false \
                and 0 <= second_line_vanish_counter
                       and second_line_vanish_counter <= second_line_vanish_threshold) {
                second_line_vanish_counter += 1;
                line(frame_, final_lanes.second_lines[2] + Point2f(MY_boundingBox.tl()),
                     final_lanes.second_lines[3] + Point2f(MY_boundingBox.tl()), Scalar(0, 0, 200), 0.5, CV_AA);
//				cout << "[Cond2]———————————————>" << second_line_vanish_counter << "<———————————————\n";
            } else if (final_lanes.left_second_valid ==false \
                and second_line_vanish_counter > second_line_vanish_threshold){
                second_line_vanish_counter = 0;
//				cout << "[Cond3]———————————————>" << second_line_vanish_counter << "<———————————————\n";
            } else {
//				cout << "[Cond4]———————————————>" << second_line_vanish_counter << "<———————————————\n";
            }
#endif
        }
            //车道中心预瞄点绘制
			/*Point2f centerline1,centerline2;
			centerline1.x = (final_lanes.center_lines[0].x + final_lanes.center_lines[2].x) / 2 + (MY_boundingBox.x);
			centerline1.y = (final_lanes.center_lines[0].y + final_lanes.center_lines[2].y) / 2 + (MY_boundingBox.y);
			centerline2.x = (final_lanes.center_lines[1].x + final_lanes.center_lines[3].x) / 2 + (MY_boundingBox.x);
			centerline2.y = (final_lanes.center_lines[1].y + final_lanes.center_lines[3].y) / 2 + (MY_boundingBox.y);
			circle(frame_, 0.7*centerline1 + 0.3* centerline2, 5, cv::Scalar(0, 255, 0), 1);*/
            g_LeftLane_x0 = final_lanes.center_lines[0].x + MY_boundingBox.x;
            g_LeftLane_y0 = final_lanes.center_lines[0].y  + MY_boundingBox.y;
            g_LeftLane_x1 = final_lanes.center_lines[1].x + MY_boundingBox.x;
            g_LeftLane_y1 = final_lanes.center_lines[1].y  + MY_boundingBox.y;
            g_RightLane_x2 = final_lanes.center_lines[2].x + MY_boundingBox.x;
            g_RightLane_y2 = final_lanes.center_lines[2].y  + MY_boundingBox.y;
            g_RightLane_x3 = final_lanes.center_lines[3].x + MY_boundingBox.x;
            g_RightLane_y3 = final_lanes.center_lines[3].y  + MY_boundingBox.y;

    }

}

//================================车辆离车道线的距离,跨线标志计算====================================
void lane_distance_compute()
{
  CameraInfo cameraInfo;
  IPMInfo IPMinfo;
  cameraInfo.focalLength.x=1105.6;
  cameraInfo.focalLength.y=1107.8;
  cameraInfo.opticalCenter.x=350.3636;
  cameraInfo.opticalCenter.y=169.6710;
  cameraInfo.cameraHeight=1240;
  cameraInfo.pitch=6.2902;
  cameraInfo.yaw=0.303;
  cameraInfo.imageWidth=640;
  cameraInfo.imageHeight=480;

  IPMinfo.ipmwidth=640;
  IPMinfo.ipmheight=480;
  IPMinfo.ipmLeft=0;
  IPMinfo.ipmRight=640;
  IPMinfo.ipmTop=0;
  IPMinfo.ipmBottom=480;

  /* 	control value */
  unsigned char isGetLanes = 0x01;//是否检测到车道线标志,0-No,1-Yes
  CCanLaneRes LaneRes;

  int visionPreview_dist;
  int visionAngle_angle;

  //add by nh to calculate the real distance
  double k_left, k_right;	//定义图像坐标系下左右两条车道线的斜率
  double b_left, b_right;	//定义图像坐标系下左右两条车道线直线方程的b值
  double g_ThetaLeft, g_ThetaRight;	//定义图像坐标系下左右两条车道线与x轴正方向所成的夹角
  //求出两条识别出来的车道线的直线方程
  k_left = (double)(g_LeftLane_y0 - g_LeftLane_y1)/(g_LeftLane_x0 - g_LeftLane_x1);
  k_right = (double)(g_RightLane_y2 - g_RightLane_y3)/(g_RightLane_x2 - g_RightLane_x3);
  b_left = g_LeftLane_y0 - k_left * g_LeftLane_x0;
  b_right = g_RightLane_y2 - k_right * g_RightLane_x2;
  
  //利用图像最下端y=480的像素坐标带入两条直线方程得到左右两条直线上的这个y坐标下的横坐标
  float x_left, x_right;
  float y_img = 480.0;
  x_left = (y_img - b_left) / k_left;
  x_right = (y_img - b_right) / k_right;
  float u_left, v_left;
  float u_right, v_right;
  u_left = x_left;
  v_left = y_img;
  u_right = x_right;
  v_right = y_img;
  Mat uv_left = (Mat_<float>(2,1)<<u_left, v_left);
  Mat uv_right = (Mat_<float>(2,1)<<u_right, v_right);

  Mat xy_left =(Mat_<float>(2,1));
  Mat xy_right =(Mat_<float>(2,1));
  xy_left = Transformimage2ground(uv_left,cameraInfo);
  xy_right = Transformimage2ground(uv_right,cameraInfo);

  double l_realDist, r_realDist;
  double angle_l, angle_r;
  angle_l = _ANGLEl*PI/180;
  angle_r = _ANGLEr*PI/180;
  double angle;
  angle = (angle_l + angle_r) / 2;
  double lDist_calculation, rDist_calculation;
	double x_calfinal,y_calfinal;
  if (angle <=0)
  {
    lDist_calculation = xy_left.at<float>(0,0)/10 + y480distance*sin(angle);
		x_calfinal= (xy_left.at<float>(0,0)/10/tan(angle)+276)*sin(angle);
    rDist_calculation = xy_right.at<float>(0,0)/10 + y480distance*sin(angle);
		y_calfinal= (xy_right.at<float>(0,0)/10/tan(angle)-276)*sin(angle);
  }
  else
  {
    lDist_calculation = xy_left.at<float>(0,0)/10 - y480distance*sin(angle);
    rDist_calculation = xy_right.at<float>(0,0)/10 - y480distance*sin(angle);
  }
  l_realDist = abs(lDist_calculation);
  r_realDist = abs(rDist_calculation);
  LaneRes.ObjectNum = g_objectnum;  //先固定障碍物为5个
  LaneRes.QualityOfLane = 2; //固定车道线质量
  LaneRes.CurvOfLane =(int)final_lanes.first_pramater_left;    //曲率
  LaneRes.DerivOfLaneCurv = (int)(2*final_lanes.first_pramater_left); //曲率的导数
  /* compute and transmit control value */
  if (framecount > 1)
  {
    //判断车道线类型
    LaneRes.TypeofRightLane = 0;
    LaneRes.TypeofLeftLane = 0;
    LaneRes.QualityOfLane = 0;
    if ((final_lanes.left_solid_line == 0 || final_lanes.left_solid_line == 1)&& (final_lanes.right_soild_line == 0 || final_lanes.right_soild_line == 1))
    {
      if (final_lanes.left_solid_line == 0 )
      {
        LaneRes.TypeofLeftLane=0x02;  //output soild left lane flag
        LaneRes.QualityOfLane = 2;
      }
      else if (final_lanes.left_solid_line == 1 )
      {
        LaneRes.TypeofLeftLane =0x01;  //output dash left lane flag
        LaneRes.QualityOfLane = 2;
      }

      if (final_lanes.right_soild_line == 0)
      {
        LaneRes.TypeofRightLane =0x02;  //output soild right lane flag
        LaneRes.QualityOfLane = 2;
      }
      else if (final_lanes.right_soild_line == 1)
      {
        LaneRes.TypeofRightLane =0x01;  //output dash right lane flag
        LaneRes.QualityOfLane = 2;
      }
 		}
    else
    {
      //无法检测到车道线
      isGetLanes = 0x00;
      LaneRes.TypeofRightLane = 0;
      LaneRes.TypeofLeftLane = 0;
      LaneRes.QualityOfLane = 0;

    }
    //输出车道线距离及跨越车道线标识判断-start add by nh 20171115
    double temp_dist2leftlane;
    double temp_dist2rightlane;
    double temp_dist2leftlanecal;
    double temp_dist2rightlanecal;
    if ((LaneRes.TypeofRightLane != 0) && (LaneRes.TypeofLeftLane != 0))
    {
      LaneRes.IsCrossingLane = 0;
      LaneRes.Dist2RightLane = (int)abs(r_realDist);
      LaneRes.Dist2LeftLane = (int)abs(l_realDist);  //Camera到左侧车道线的距离
      //解决当左右车道线距离突变情况下的跨线信息输出-start
      if (temp_dist2rightlane<=100 && LaneRes.Dist2RightLane>=220)
      {
        LaneRes.IsCrossingLane = 1;		//向右压线-1
        LaneRes.Dist2RightLane = temp_dist2rightlane;		//output the last value
        temp_dist2rightlanecal = rDist_calculation;
      }

      else if ((temp_dist2rightlanecal<=100 && rDist_calculation>=220)||(rDist_calculation < 0))
      {
        LaneRes.IsCrossingLane = 1;		//向右压线-1
        LaneRes.Dist2RightLane = temp_dist2rightlane;		//output the last value
        temp_dist2rightlanecal = rDist_calculation;
      }
      if (temp_dist2leftlane<=100 && LaneRes.Dist2LeftLane>=220)
      {
        LaneRes.IsCrossingLane = 2;		//向左压线-2
        LaneRes.Dist2LeftLane = temp_dist2leftlane;		//output the last value
        temp_dist2leftlanecal = lDist_calculation;
      }
      else if ((temp_dist2leftlanecal>=-100 && lDist_calculation<=-220)||(lDist_calculation >= 0))
      {
        LaneRes.IsCrossingLane = 2;		//向左压线-2
        LaneRes.Dist2LeftLane = temp_dist2leftlane;		//output the last value
        temp_dist2leftlanecal = lDist_calculation;
      }
      //解决当左右车道线距离突变情况下的跨线信息输出-end
      if (lDist_calculation < 0)
      {
        temp_dist2leftlane = LaneRes.Dist2LeftLane;		//始终保存上一帧的非0值
        temp_dist2leftlanecal = lDist_calculation;
      }
      if(left_distance_new.size()<3)
        left_distance_new.push_back(LaneRes.Dist2LeftLane);
      else if(left_distance_new.size()==3 and abs(LaneRes.Dist2LeftLane-left_distance_new[2])<20)
        left_distance_new.push_back(LaneRes.Dist2LeftLane);

  
      if (rDist_calculation > 0)
      {
        temp_dist2rightlane = LaneRes.Dist2RightLane;		//始终保存上一帧的非0值
        temp_dist2rightlanecal = rDist_calculation;
      }
      if(right_distance_new.size()<3)
        right_distance_new.push_back(LaneRes.Dist2RightLane);
      else if(right_distance_new.size()==3 and abs(LaneRes.Dist2RightLane-right_distance_new[2])<20)
        right_distance_new.push_back(LaneRes.Dist2RightLane);

      if (final_lanes.left_second_valid == true)
      {
        LaneRes.Dist2NextLeftLane = LaneRes.Dist2LeftLane + LANE_WIDTH_WORLD / 10;  //=距离左侧车道线的距离+车道线的宽
        if (LaneRes.Dist2NextLeftLane > 800)
        {
          waitKey(0);
        }
      }
      else
      {
        LaneRes.Dist2NextLeftLane = 0;
      }


      if (final_lanes.right_second_valid == true)
      {
        LaneRes.Dist2NextRightLane = LaneRes.Dist2RightLane+ LANE_WIDTH_WORLD / 10;//=距离右侧车道线的距离+车道线的宽
      }
      else
      {
      	LaneRes.Dist2NextRightLane = 0;
      }
      LaneRes.AngleofVehicle2Lane = (int)(m_fFinalAngle * 10);
    }


        //follow the lane of the last frame when missing the lane
    else
    {
      LaneRes.Dist2LeftLane = temp_dist2leftlane;		//没有车道线的时候，输出上一帧的值
      LaneRes.Dist2RightLane = temp_dist2rightlane;		//没有车道线的时候，输出上一帧的值
      temp_dist2leftlanecal = lDist_calculation;
      temp_dist2rightlanecal = rDist_calculation;
      LaneRes.IsCrossingLane = 3;			//车道线丢失时跨越车道线的值为3
      //if ((LaneRes.TypeofRightLane != 0) && (LaneRes.TypeofLeftLane != 0))
      //break;
    }
    //输出车道线距离及跨越车道线标识判断-end add by nh 20171115
    //屏幕打印距离左右车道线距离结果以及跨越车道线标识-start add by nh 20171115
    if(_VIDEO_STORE)
    {
			originalVideo << frame;
		}	//储存源视频写入帧数据
    
		ostringstream text_temp0;//set up a stream
    text_temp0 << abs(xy_left.at<float>(0,0)/10);//abs(l_real/10);
    putText(road, "New, DistleftLane2OpticAxis:",Point(30,20),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,0),1,8);
    putText(road, text_temp0.str(),Point(300,20),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,0),1,8);

   	ostringstream text_temp1;//set up a stream
    text_temp1 << abs(xy_right.at<float>(0,0)/10);//abs(r_real/10);
    putText(road, "New, DistRightLane2OpticAxis:",Point(30,50),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,0),1,8);
    putText(road, text_temp1.str(),Point(300,50),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,0),1,8);

    ostringstream text_temp2;//set up a stream
    text_temp2<< abs(l_realDist)+abs(r_realDist);
  	putText(road, "New, DistLaneWidth:",Point(30,80),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,0),1,8);
  	putText(road, text_temp2.str(),Point(300,80),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,0),1,8);

    ostringstream text_temp3;//set up a stream
   	text_temp3<< LaneRes.Dist2LeftLane;
    putText(road, text_temp3.str(),Point(500,110),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,0),1,8);

    ostringstream text_temp4;//set up a stream
    text_temp4<< LaneRes.Dist2RightLane;
    putText(road, "perspective transformation, DistRightLane2OpticAxis:",Point(30,140),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,0),1,8);
    putText(road, text_temp4.str(),Point(500,140),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,0),1,8);

    ostringstream text_temp5;//set up a stream
		text_temp5<<xy_left.at<float>(0,0)/10;
    putText(road, "original left distance:",Point(30,170),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,0),1,8);
    putText(road, text_temp5.str(),Point(250,170),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,0),1,8);

    ostringstream text_temp6;//set up a stream
    text_temp6<< xy_right.at<float>(0,0)/10; 
    putText(road, "original right distance:",Point(30,200),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,0),1,8);
    putText(road, text_temp6.str(),Point(250,200),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,0),1,8);

    ostringstream text_temp7;//set up a stream
    text_temp7<< angle;
    putText(road, "CalculateAngle:",Point(30,230),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,0),1,8);
    putText(road, text_temp7.str(),Point(250,230),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,0),1,8);
		
		ostringstream text_temp8;
		text_temp8<<x_calfinal;
	  putText(road, "x_calfinal:",Point(30,260),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,0),1,8);
    putText(road, text_temp8.str(),Point(250,260),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,0),1,8);

		ostringstream text_temp9;
		text_temp9<<y_calfinal;
	  putText(road, "y_calfinal:",Point(30,290),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,0),1,8);
    putText(road, text_temp9.str(),Point(250,290),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,0),1,8);

    draw_detected_lane(road, final_lanes, framecount);	//在road图像上画理想识别的车道线直线
    roadVideo << road;//储存road视频写入帧数据
    //屏幕打印距离左右车道线距离结果以及跨越车道线标识-end add by nh 20171115
    LaneRes.AngleofVehicle2Lane = (int)(m_fFinalAngle * 1000);
    //cout << "________________"<<LaneRes.Dist2LeftLane<<"________________"<<endl;
    //cout << "________________"<<LaneRes.Dist2RightLane<<"________________"<<endl;
    //在buffer文件中储存车道线距离及跨越车道线标志信息用于Excel数据分析-start add by nh 20171115
    buffer<<"[300]: "<< lDist_calculation <<"[400]: "<< rDist_calculation<<"[000]:" << LaneRes.Dist2LeftLane<<"[001]:" << LaneRes.Dist2RightLane<<"[002]:"<<(size_t)LaneRes.IsCrossingLane<<endl;
    //在buffer文件中储存车道线距离及跨越车道线标志信息用于Excel数据分析-end add by nh 20171115
  }

  g_typeofleftlane=LaneRes.TypeofLeftLane;
  g_typeofrightlane=LaneRes.TypeofRightLane;
  g_qualityoflane=LaneRes.QualityOfLane;
  g_iscrossinglane=LaneRes.IsCrossingLane;
  g_dist2leftlane=LaneRes.Dist2LeftLane;
  g_dist2righttlane=LaneRes.Dist2RightLane;

  g_dist2nextleftlane=LaneRes.Dist2NextLeftLane;
  g_dist2nextrightlane=LaneRes.Dist2NextRightLane;
  g_angleofvehicle2lane=LaneRes.AngleofVehicle2Lane;
  g_curveoflane=LaneRes.CurvOfLane;
  g_derivoflanecurv=LaneRes.DerivOfLaneCurv;
}



//==============================车道线识别线程=====================================
void func_lane_detect()
{
	Mat H_transform;//逆透视矩阵
	FileStorage fs(IPM_file, FileStorage::READ);
	fs["matrix"] >> H_transform;//从文件读入矩阵
	fs.release();

  //-------------------------从摄像头/视频中读入图像流-------------------------
	VideoCapture inputVideo("/media/nvidia/3261-33382/right.avi");

	if (!inputVideo.isOpened())
	{
  	LOG(FATAL) << "Open video error !";
	}
	
	// if read camera and specify the size, set the parameters
	if(im_width > 0)
	{
  	inputVideo.set(CV_CAP_PROP_FRAME_WIDTH, im_width);
  	LOG(INFO) << "Set image width is  " << im_width ;
	}
	if(im_height > 0)
	{
  	inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT, im_height);
  	LOG(INFO) << "Set image height is  " << im_height ;
	}
	if(im_fps > 0)
	{
    inputVideo.set(CV_CAP_PROP_FPS, im_fps);
  	LOG(INFO) << "Set capture fps is  " << im_fps ;
	}

	s = Size((int)inputVideo.get(CV_CAP_PROP_FRAME_WIDTH), (int)inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT));
	cout << "TotalFrame  " << inputVideo.get(CV_CAP_PROP_FRAME_COUNT) << endl;
	cout << "FPS   "  << inputVideo.get(CV_CAP_PROP_FPS) << endl;
	cout << "Size  " << s.width << "x" << s.height << endl;
 
	CWarning(); //初始化相应的值？？？
	vector<Mat> _debug_images;//存储车道线检测的中间图像,用于调试
	while(true)
	{
    usleep(5000);//加入短暂延时,防止循环过快导致卡顿
  	inputVideo >> frame;

  	if(!frame.empty())
		{
			save_distance<<framecount<<";";
  		road = frame.clone();
   		double t = (double)cv::getTickCount();
   		_debug_images.clear();
      //--------------------------------车道线检测---------------------------------------
  		final_lanes = lane_detector(road, framecount, H_transform, _debug_images, _DEBUG);
  		t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();

        lane_distance_compute();
	  	
	    //---------------------------实时更新车道线识别信息,存入数组用于传输---------------------------
	    unsigned char lane_buffer[8] = {0};
	    unsigned short temp = 0;

	    lane_buffer[0] = g_objectnum<< 4; //固定为5个障碍物
	    lane_buffer[1] = (g_typeofleftlane<<6)|(g_typeofrightlane<<2);  //左右车道线的类型
	    lane_buffer[2] = (g_qualityoflane)<<6;    //车道线质量
	    lane_buffer[3] = g_iscrossinglane<<6;      //判断是否在跨越车道线
	    lane_buffer[4] = (g_dist2leftlane>>2)&0xff;
	    lane_buffer[5] = ((g_dist2leftlane&0x03)<<6) | ((g_dist2righttlane >> 6) & 0x0f);//?
	    lane_buffer[6] = (g_dist2righttlane & 0x3f) << 2;//?
	    lane_buffer[7] = 0;

	    lane_buffer[0] = (g_dist2nextleftlane>>4)&0xff;
	    lane_buffer[1] = ((g_dist2nextleftlane & 0x0f)<<4)|((g_dist2nextrightlane>>8)&0x0f);
	    lane_buffer[2] = g_dist2nextrightlane & 0xff;
	    lane_buffer[3] = (g_angleofvehicle2lane >> 2)&0xff;
	    lane_buffer[4] = (g_angleofvehicle2lane &0x03)<<6;
	    lane_buffer[5] = (g_curveoflane>>8)&0xff;
	    lane_buffer[6] = g_curveoflane &0xff;
	    lane_buffer[7] = g_derivoflanecurv;


	    if(_DEBUG && framecount > 1)
	    {
		    //Show the debug images
		    if(_debug_images.size() == 6)
	    	{
		    	imshow("edge", _debug_images[0]);
		    	imshow("vertical", _debug_images[1]);
		    	imshow("edge2", _debug_images[2]);
		    	imshow("horizontal", _debug_images[3]);
		    	imshow("test", _debug_images[4]);
	        imshow("perspective", _debug_images[5]);
		    }
	      imshow("Lanes Detection Result", road);
		    waitKey(1);
	    }

			if(DETECT_FLAG == true & framecount > 1)
			{

				DETECT_FLAG = false;

				img_temp = frame.clone();

				lanes.clear();

	        // if lanes are detected, pass them to the global variable "vector<> lanes" for the using of object detection
				if ((final_lanes.left_solid_line == 0 || final_lanes.left_solid_line == 1)
				&& (final_lanes.right_soild_line == 0 || final_lanes.right_soild_line == 1))
				{

					for (size_t l = 0; l < 4; l++) 
					{

						Point2f pt_temp = Point2f(final_lanes.center_lines[l].x+MY_boundingBox.x, final_lanes.center_lines[l].y+MY_boundingBox.y);

						lanes.push_back(pt_temp);

					}
				}
	 		//Serial_send
      	if ((final_lanes.left_solid_line == 0 || final_lanes.left_solid_line == 1)
          && (final_lanes.right_soild_line == 0 || final_lanes.right_soild_line == 1)) 
					{
          	SerialLaneDetect = "$" + IntToString(framecount) + "," + "lane" + "," + "1" + "," +
                             IntToString(final_lanes.left_solid_line) + "," +IntToString(g_dist2leftlane)+","+
                             IntToString(final_lanes.right_soild_line) + IntToString(g_dist2righttlane)+IntToString			(g_angleofvehicle2lane)+"!\n";
          serial_send_lane(fd2);
        } 
				else 
				{
          SerialLaneDetect = "$" + IntToString(framecount) + "," + "lane" + "," + "0" + "!\n";
          serial_send_lane(fd2);
        }
			}
			framecount++;
		}
    else OVER = true;//Video end
	}
  if(location_out.is_open() && location_out_new.is_open())
  {
    location_out.close();
    location_out_new.close();
  }
}


int main(int argc, char** argv)
{
  	::google::InitGoogleLogging(argv[0]);
  	FLAGS_alsologtostderr = 1;
  	gflags::SetUsageMessage("Read video file and save video.\n"
        "Usage:\n"
        "    TAGE_Camera --model_file *.prototxt --weights_file *.caffemodel\n"
        "         --camera_id 0 or 1 --output_video *.avi --ipm_file *.xml\n"
        "         --confidence_threshold 0.2\n");
 	 	gflags::ParseCommandLineFlags(&argc, &argv, true);

    //-------------------gflag传入的值---------------------
  	camera_id = FLAGS_camera_id;
  	output_video = FLAGS_output_video;
  	IPM_file = FLAGS_ipm_file;
  	im_width = FLAGS_width;
  	im_height = FLAGS_height;
  	im_fps = FLAGS_fps;
  	_DEBUG = FLAGS_DEBUG;
  	_SERIAL_OPEN = FLAGS_SERIAL_OPEN;
  	_VIDEO_STORE = FLAGS_VIDEO_STORE;

//------------------------------------------------------------打开串口-------------------------------------------------

    if (0) {
        //串口开启
        if ((fd2 = open_port(fd2, 1)) < 0) {
            perror("Open USB Serial Port Error !");
        }
        //串口设置
        if ((i2 = set_opt(fd2, 9600, 8, 'N', 1)) < 0) {
            perror("USB Serial Port set_opt Error");
        }
    }
   //-----------------------------------------------打开串口END---------------------------------------------------

  	//----------------------开启车道线识别线程--------------------------
  	boost::thread thrd(&func_lane_detect);
    
    //视频存储
    if(_DEBUG)
		{	
			roadVideo.open("./video/output_road.avi", CV_FOURCC('M','J','P','G'),20, Size(640,480));
  	}
  	while(true)
  	{
    	if(!img_temp.empty())
    	{

        if(_DEBUG)LOG(INFO)<< "The frame is " << framecount;

        //绘制车道线
        draw_detected_lane(img_temp,final_lanes,framecount);
		    
				if(_DEBUG)
		    {
			    imshow("Detected All", img_temp);
			    waitKey(1);
			   // objectVideo << img_temp;
		    }
			
			img_temp.~Mat();
		}
		if(OVER==true)
		{
			break;
		}
	}
	return 0;
}
