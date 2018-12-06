#ifndef LANE_DETECTION_HPP_
#define LANE_DETECTION_HPP_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include "LDW.h"

using namespace std;
using namespace cv;

//逆透视图像中相邻两条车道线间距的像素约束范围
//-----------lingyun camera---------------
//#define desitance_above_min 40
//#define desitance_above_max 60
//#define desitance_below_min 45
//#define desitance_below_max 65

#define desitance_above_min 40
#define desitance_above_max 60
#define desitance_below_min 45
#define desitance_below_max 65

float Distance_W_Left[1000] = { 0 };    //世界坐标系下车辆中心距离左侧车道线的距离
float Distance_W_Right[1000] = { 0 };   //世界坐标系下车辆中心距离right车道线的距离
int framecnt_w = 0;
extern double _ANGLEl;  //世界坐标系下的偏航角
extern double _ANGLEr;  
extern int _DEBUG;

//在原始图像中定义ROI
Rect MY_boundingBox(Point2f(0, 150), Size(640, 330));


// [!] 连续n帧丢失才丢失　START
//#define SECOND_LINE_ANTI_VANISH
int second_line_vanish_threshold = 9;
int second_line_vanish_counter = 0;
// [!] END
//存储初步筛选的线段
struct line_order
{
	Vec4i m_line;//俩端点的x,y坐标
	float m_slope;//斜率
	float m_intersect;//截距
};
//存储进一步筛选的线段
struct line_final
{
	Vec4i m_line;
	int   m_pix;
    float m_AveragePix;
	float m_slope;
	float m_intersect;
};
//存储映射至原始图像的车道线识别参数结果
struct back_lines
{
	vector<Point2f> second_lines;//临车道
	vector<Point2f> left_lines;//左车道
	vector<Point2f> right_lines;//右车道
	vector<Point2f> center_lines;//中心线
	int left_solid_line = 0;//0表示实线,1表示虚线,2表示未检测
	int right_soild_line = 0;//0表示实线,1表示虚线,2表示未检测
	bool left_second_valid = false;
	bool right_second_valid = false;
	float m_fCentre2LeftDist;  //世界坐标系下车辆中心距离左侧车道线的距离
	float m_fCentre2RightDist;
	float m_fAngle;
	//在原始图像中拟合的车道线曲线参数
	float first_pramater_left;  //a
	float second_pramater_left;  //b 
	float third_pramater_left;  //c
	float first_pramater_right;  //a
	float second_pramater_right;  //b 
	float third_pramater_right;  //c
};
//==========================================存储初步筛选的线段集合================================================
void MyPushback(std::vector<line_order> & line_temp, const Vec4i &m1, const float &m2, const float &m3)
{
	line_order test;
	test.m_line = m1;
	test.m_slope = m2;
	test.m_intersect = m3;
	line_temp.push_back(test);
}
//==========================================存储进一步筛选的线段集合=======================================================
void final_Pushback(std::vector<line_final> & line_temp, const Vec4i &m1, const int &m2, const float &m3, const float &m4, const float &m5)
{
	line_final test;
	test.m_line = m1;
	test.m_pix = m2;
    test.m_AveragePix = m5;
	test.m_slope = m3;
	test.m_intersect = m4;
	line_temp.push_back(test);
}
//===将线段集合按照截距大小排序===
class decrease_intersect
{
public:
	bool operator ()(const line_order& stItem1, const line_order& stItem2)
	{
		return stItem1.m_intersect < stItem2.m_intersect;
	}
};
//===将线段集合按照灰度值总和的大小排序===
class decrease_averagepix
{
public:
	bool operator ()(const line_final& stItem1, const line_final& stItem2)
	{
		return stItem1.m_AveragePix > stItem2.m_AveragePix;
	}
};

//===========================曲线拟合函数==============================
//参数说明--- vec:点集地址, times:拟合曲线的次数, p:拟合曲线的参数
bool fittingCurve(vector<Point2f> &vec, int times, float *p)
{
	float *py = new float[vec.size()];
	float *px = new float[times*vec.size()];
	int i = 0;
	Point2f* P = &vec[0];
	for (vector<Point2f>::iterator itr = vec.begin(); itr != vec.end(); ++itr)
	{
		py[i] = (*itr).y;
		int j = 0;
		while (j < times)
		{
			px[times*i + j] = pow(((*itr).x), float(j));
			j++;
		}
		i++;
	}
	Mat X = Mat(vec.size(), times, CV_32FC1, px);
	float* Xp = &(X.at<float>(0));
	Mat X_T;
	transpose(X, X_T);
	Mat Y = Mat(vec.size(), 1, CV_32FC1, py);
	Mat para = ((X_T*X).inv())*X_T*Y;
	for (int s = 0; s < times; s++)
	{
		p[s] = para.at<float>(s);
	}
	delete[] px;
	delete[] py;
	return true;
}

Mat gray_enhance(Mat& gray,int fa,int fb)
{
    Mat gray_Contrast = Mat::zeros(gray.size(),CV_8UC1);
    double k = 255.0/(fb-fa);
    for (int y = 0; y < gray.rows; y++)
    {
        for (int x = 0; x < gray.cols; x++)
        {
            if (gray.at<uchar>(y, x) < fa)
                gray_Contrast.at<uchar>(y, x) = 0;
            else if (gray.at<uchar>(y, x) >fb)
                gray_Contrast.at<uchar>(y, x) = 255;
            else
                gray_Contrast.at<uchar>(y, x) = (int)(k*(gray.at<uchar>(y, x) - fa));
        }
    }
    return gray_Contrast;
}


//============================================车道线检测核心函数==================================================
//参数说明---　frame:待检测的图像,　count:第几帧,　H_transform:逆透视矩阵,　_Debug_Image:存储调试过程中的中间图像
back_lines lane_detector(Mat frame, int count, Mat H_transform, vector<Mat>& _Debug_Image, bool Isdebug = false)
{
	Mat perspective;//存储逆透视图像
	Rect boundingBox; //定义逆透视图像中的ROI
	boundingBox.x = 230;
	//boundingBox.x = 0;
	boundingBox.y = 20;
	boundingBox.width = 180;
	//boundingBox.width = 640;
	boundingBox.height = 310;

	//车道线识别结果返回的参数
	back_lines final_back_lines;
	//逆透视图像中待拟合的点
	vector<Point2f> left_ransac;
	vector<Point2f> right_ransac;
	//逆透视图像中拟合曲线中的采样点
	vector<Point2f> final_per_point_fit_left;
	vector<Point2f> final_per_point_fit_right;
	bool left_ture = false;
	bool right_ture = false;

    //卡尔曼滤波器初始化
	const int stateNum = 8;//状态空间维数
	const int measureNum = 4;//测量值维数
	static KalmanFilter KF(stateNum, measureNum, 0);//初始化
	static Mat measurement = Mat::zeros(measureNum, 1, CV_32F);//测量矩阵
	static Mat back_transform;

	back_transform = H_transform.inv();//透视矩阵
	static Point2f left_point_1[3] = { Point2f(0,0) };
	static Point2f left_point_2[3] = { Point2f(0,0) };
	static Point2f right_point_1[3] = { Point2f(0,0) };
	static Point2f right_point_2[3] = { Point2f(0,0) };


	static int lane_miss_count = 0;//统计车道线丢失的帧
	static int lane_line_problem = 0;//统计识别出的车道线存在误检的情况
	static int two_lanes = 0;//统计识别出的两条车道线不满足间距约束条件的情况

	int lane_change_label = 0;	//换道标志

	if (count == 0){
		//1.kalman filter setup
		KF.transitionMatrix = Mat::eye(Size(8,8),CV_32FC1);
		Mat_<float> trans_Mat = KF.transitionMatrix;
		for(int i =0,j=4;i<4;i++,j++)
		{
			trans_Mat(i,j)=1;
		}
		setIdentity(KF.measurementMatrix);//设置测量矩阵H
		setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//设置过程噪声Q
		setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//设置测量噪声R
		setIdentity(KF.errorCovPost, Scalar::all(1));//设置最小均方误差P
		setIdentity(KF.statePost, Scalar::all(1e-1));//设置系统初始状态x(0)
	}
	//===========================================图像预处理=================================================
	Mat gray;//存储灰度图
	Mat fram_change, frame_temp;
	frame_temp = frame(MY_boundingBox);//原始图中提取ROI部分
	cvtColor(frame_temp, fram_change, CV_BGR2GRAY);//RGB图像转化为灰度图
	warpPerspective(fram_change, perspective, H_transform, perspective.size(), INTER_LINEAR, BORDER_CONSTANT);//逆透视变换
	gray = perspective(boundingBox);//逆透视图像中再提取ROI部分

    //Mat imageShold;
    //threshold(gray,imageShold,130,255,THRESH_BINARY);
    //imshow("imageShold",imageShold);
   // waitKey(1);

	//======================纵向边缘提取========================
	Mat vertical, edge;//存储提取的纵向边缘图
	//二阶可分离高斯滤波
	GaussianBlur(gray, vertical, Size(1, 41), 0, 0);   //纵向滤波
	GaussianBlur(vertical, vertical, Size(7, 1), 0, 0);//横向滤波 (3,1)

    //Mat gray_Contrast;
    //gray_Contrast = gray_enhance(vertical,50,200);
    //vertical  = gray_Contrast.clone();
    //imshow("gray_Contrast",gray_Contrast);
    //waitKey(1);

//	Canny(vertical, edge, 37, 53, 3);
    Canny(vertical, edge, 15, 35, 3);//Canny算子边缘提取

	//====================横向边缘提取==========================
    Mat horizontal, edge2;//存储提取的横向边缘图
    GaussianBlur(gray, horizontal, Size(25, 1), 0, 0);
    GaussianBlur(horizontal, horizontal, Size(1, 3), 0, 0);
    Canny(horizontal, edge2, 15, 35, 3);


	if(_DEBUG){
		_Debug_Image.push_back(edge);
		_Debug_Image.push_back(vertical);
	}
	imshow("edge", edge);
	waitKey(1);
	imshow("vertical", edge);
	waitKey(1);
	vector<Vec4i> lines;
	vector<line_order> lines_detect, lines_process, line_cluster;
	vector<line_order> lines_detect_hor;
	vector<line_final> left_lines, right_lines;
	vector<Point2f> final_per_point(4), final_per_point_second(4);
	vector<Point2f> final_back_point(4), final_back_point_second(4);

	//===================================初步提取纵向直线并存储=========================================
	float slope, intersect;
	HoughLinesP(edge, lines, 1, CV_PI / 180, 40, 40, 40);//霍夫直线检测
	for (size_t i = 0; i< lines.size(); i++)
	{
		Vec4i l = lines[i];
		float dy = (float)((float)l[3] - (float)l[1] );
		float dx = (float)((float)l[2] - (float)l[0]);
		slope = dx / (dy+0.0001);     //slop of the line
		if ((slope<0.18) && (slope>-0.18))
		{
			intersect = (l[2] * l[1] - l[0] * l[3]) / (l[1] - l[3]+0.0001);
			MyPushback(lines_detect, l, slope, intersect);
		}
	}

	//===================================初步提取横向直线并存储=========================================
	vector<Vec4i> lines2;
	HoughLinesP(edge2, lines2, 1, CV_PI / 180, 10, 12, 9);
	float slope2, intersect2;
	Mat test = Mat::zeros(edge2.size(),CV_8UC3);
	for (size_t i = 0; i< lines2.size(); i++)
	{
		Vec4i l = lines2[i];
		float dy = (float)((float)l[3] - (float)l[1] );
		float dx = (float)((float)l[2] - (float)l[0]);
		slope2 = dy / (dx+0.0001);
		if ((slope2<0.23) && (slope2>-0.23))
		{
		    line(test, Point(l[0], l[1]),Point(l[2], l[3]), Scalar(0, 255, 255), 1, CV_AA);
			intersect2 = (l[2] * l[1] - l[0] * l[3]) / (l[1] - l[3]+0.0001);
			MyPushback(lines_detect_hor, l, slope2, intersect2);//在横向集合中添加检测的线
		}
	}

	if(_DEBUG){
		_Debug_Image.push_back(edge2);
		_Debug_Image.push_back(horizontal);
		_Debug_Image.push_back(test);
	}
	imshow("edge2", edge2);
	waitKey(1);
	imshow("horizontal", horizontal);
	waitKey(1);
	imshow("test", test);
	waitKey(1);
	//===============================根据初步提取出直线的斜率和截距,对距离较近的直线进行合并==================================
	for (unsigned int i = 0; i < lines_detect.size(); ++i)
	{
		for (unsigned int j = i + 1; j < lines_detect.size(); ++j)
		{
            vector<Point2f> mid_point1(1);
            vector<Point2f> mid_point2(1);
	//====================================线段中点坐标=================================
            mid_point1[0]= Point2f((lines_detect[i].m_line[0]+lines_detect[i].m_line[2])/2,(lines_detect[i].m_line[1]+lines_detect[i].m_line[3])/2);
            mid_point2[0]= Point2f((lines_detect[j].m_line[0]+lines_detect[j].m_line[2])/2,(lines_detect[j].m_line[1]+lines_detect[j].m_line[3])/2);
            //cout<<"mid_point1[0]: "<<mid_point1[0]<<" mid_point2[0]: "<<mid_point2[0]<<endl;
            if ((abs(lines_detect[i].m_intersect - lines_detect[j].m_intersect) < 8) && (abs(lines_detect[i].m_slope - lines_detect[j].m_slope) < 0.04)
                && abs(mid_point1[0].y-mid_point2[0].y)<20 && abs(mid_point1[0].x-mid_point2[0].x)<20
                    )
			//if ((abs(lines_detect[i].m_intersect - lines_detect[j].m_intersect) < 8) && (abs(lines_detect[i].m_slope - lines_detect[j].m_slope) < 0.04))
			{
				lines_detect[i].m_line = (lines_detect[i].m_line + lines_detect[j].m_line) / 2;
				lines_detect[i].m_slope = (lines_detect[i].m_slope + lines_detect[j].m_slope) / 2;
				lines_detect[i].m_intersect = (lines_detect[i].m_intersect + lines_detect[j].m_intersect) / 2;
				lines_detect[j].m_intersect = 0;
				lines_detect[j].m_slope = 0;
				lines_detect[j].m_line = 0;
			}
		}
	}
	sort(lines_detect.begin(), lines_detect.end(), decrease_intersect()); //按照截距大小对合并后的直线排序
	//===================================删除合并过后的重复直线==========================================
	for (unsigned int i = 1; i < lines_detect.size(); i++)
	{
		if ((lines_detect[i - 1].m_intersect != lines_detect[i].m_intersect))
		{
			MyPushback(line_cluster, lines_detect[i].m_line, lines_detect[i].m_slope, lines_detect[i].m_intersect);
		}
	}


	//============================按照单条车道线的线宽对提取的直线进行再次筛选===============================
	for (unsigned int i = 0; i < line_cluster.size(); i++) 
	{
		float my_ture_point = 0;
		for (unsigned int y = 0; y < boundingBox.height; y = y + 2)
		{
			int my_count = 0;
			int x = line_cluster[i].m_slope*y + line_cluster[i].m_intersect;
			for (int p = x - 4; p < x + 4; ++p){
				if ((p < vertical.cols) && (p > 0))
				{
					if (edge.at<uchar>(y, p)>0)
					{
						my_count = my_count + 1;
					}
				}
			}
			if (my_count > 1)
			{
				my_ture_point = my_ture_point + 1;
			}
		}
		//cout << "my_ture_point" << my_ture_point << endl;
		//cout << "(my_ture_point / boundingBox.height)" << (my_ture_point / boundingBox.height) << endl;
		if ((my_ture_point / boundingBox.height) > 0.1)
		{
			MyPushback(lines_process, line_cluster[i].m_line, line_cluster[i].m_slope, line_cluster[i].m_intersect);
		}
	}

	//================================将筛选过后的直线变换到原始图像中,判别出左边车道线和右边车道线,并进行存储========================================
	for (unsigned int i = 0; i < lines_process.size(); ++i)
	{
		int sum_right_temp = 0, sum_right_pix = 0;
		int sum_left_temp = 0, sum_left_pix = 0;
        int pix_count = 0;
		vector<Point2f> temp_point(1);
		vector<Point2f> test_point(1);

		if(lines_process[i].m_line[1]>lines_process[i].m_line[3]){     //y1>y2
			temp_point[0] = Point2f(lines_process[i].m_line[0],lines_process[i].m_line[1]);     //(x1,y1)
		}else{
			temp_point[0] = Point2f(lines_process[i].m_line[2],lines_process[i].m_line[3]);     //(x2,y2)
		}

		temp_point[0].x=(boundingBox.height-lines_process[i].m_line[3])/((lines_process[i].m_line[3]-lines_process[i].m_line[1])/(lines_process[i].m_line[2]-lines_process[i].m_line[0]+0.00001)+0.00001)+lines_process[i].m_line[2];

		temp_point[0].y=boundingBox.height;

		temp_point[0] = temp_point[0] + Point2f(boundingBox.x, boundingBox.y);
		perspectiveTransform( temp_point , test_point, back_transform);

		if (test_point[0].x - frame.cols / 2 >= 0)//right lines
		{
			for (int y = 1; y < boundingBox.height; y = y + 2)
			{
				int x = lines_process[i].m_slope*y + lines_process[i].m_intersect;
				for (int p = x - 1; p < x + 2; ++p){
					if (p < vertical.cols){
						sum_right_temp = sum_right_temp + gray.at<uchar>(y, p);
                        pix_count++;
					}
				}
			}
            float average_pix = (float)sum_right_temp/pix_count;
			final_Pushback(right_lines, lines_process[i].m_line, sum_right_temp,
											lines_process[i].m_slope, lines_process[i].m_intersect,average_pix);
		}

		else if (test_point[0].x - frame.cols / 2 < 0)//left lines
		{
			for (int y = 1; y < boundingBox.height; y = y + 2)
			{
				int x = lines_process[i].m_slope*y + lines_process[i].m_intersect;
				for (int p = x - 1; p<x + 2; ++p){
					if (p>0){
						sum_left_temp = sum_left_temp + gray.at<uchar>(y, p);
                        pix_count++;
					}
				}
			}
            float average_pix = (float)sum_left_temp/pix_count;
			final_Pushback(left_lines, lines_process[i].m_line, sum_left_temp,
											lines_process[i].m_slope, lines_process[i].m_intersect,average_pix);
		}
	}

	//======================按照整条直线灰度总和从大到小排列=======================
	sort(right_lines.begin(), right_lines.end(), decrease_averagepix()); //左车道线集合
	sort(left_lines.begin(), left_lines.end(), decrease_averagepix()); //右车道线集合

    //======================删除灰度值较小的直线==========================
    for(vector<line_final>::iterator vi = right_lines.begin();vi!=right_lines.end();)
    {
        if((*vi).m_AveragePix<30)
            vi = right_lines.erase(vi);
        else
            ++vi;
    }

    for(vector<line_final>::iterator vi = left_lines.begin();vi!=left_lines.end();)
    {
        if((*vi).m_AveragePix<30)
            vi = left_lines.erase(vi);
        else
            ++vi;
    }

	//======================================在逆透视图像中绘制筛选过后的直线集合=============================================
    int right_lines_count = min(right_lines.size(),3);
    int left_lines_count = min(left_lines.size(),3);
	for (int i = 0; i < right_lines_count; ++i) {
        cout<<right_lines[i].m_AveragePix<<endl;
		line(perspective, Point(right_lines[i].m_line(0), right_lines[i].m_line(1)) + Point(boundingBox.x, boundingBox.y),
		Point(right_lines[i].m_line(2), right_lines[i].m_line(3)) + Point(boundingBox.x, boundingBox.y), Scalar(0, 0, 255), 1, CV_AA);
	}
    cout<<endl;
	for (int i = 0; i <left_lines_count; ++i) {
		line(perspective, Point(left_lines[i].m_line(0), left_lines[i].m_line(1)) + Point(boundingBox.x, boundingBox.y),
		Point(left_lines[i].m_line(2), left_lines[i].m_line(3)) + Point(boundingBox.x, boundingBox.y), Scalar(0, 0, 255), 1, CV_AA);
        cout<<left_lines[i].m_AveragePix<<endl;
	}

    //========================按照相邻两条车道线的间距约束条件,对初步提取出的左右车道线集合进行进一步的筛选=======================
    //选取灰度最大的直线作为初步识别出的车道线,若未识别出车道线,则使用历史值
    //若左右车道线集合中,灰度最大的直线不满足间距要求,则取灰度第二大的直线
	if(left_lines_count>0)
	{
        left_point_1[0].x = left_lines[0].m_intersect;
        left_point_1[0].y = 0;
        left_point_2[0].x = edge.rows*left_lines[0].m_slope + left_lines[0].m_intersect;
        left_point_2[0].y = edge.rows;
	}
    else
    {
        left_point_1[0] = left_point_1[1]; left_point_2[0] = left_point_2[1];
    }
    if(right_lines_count>0)
    {
        right_point_1[0].x = right_lines[0].m_intersect;
        right_point_1[0].y = 0;
        right_point_2[0].x = edge.rows*right_lines[0].m_slope + right_lines[0].m_intersect;
        right_point_2[0].y = edge.rows;
    }
    else
    {
        right_point_1[0] = right_point_1[1]; right_point_2[0] = right_point_2[1];
    }

    if(left_lines_count>0 || right_lines_count>0)
    {
        Point2f left_test_point1 = Point2f(0,0);
        Point2f left_test_point2 = Point2f(0,0);
        Point2f right_test_point1 = Point2f(0,0);
        Point2f right_test_point2 = Point2f(0,0);

        int update_label = 0;
        for(int i=0;i<left_lines_count;i++)
            for(int j=min(0,i-1);j<=min(right_lines_count,i+1);j++)
            {
                if(update_label==0)
                {
                    if(left_lines_count==0)
                    {
                        left_test_point1.x = left_point_1[0].x;
                        left_test_point2.x = left_point_2[0].x;
                    }
                    else
                    {
                        left_test_point1.x = left_lines[i].m_intersect;
                        left_test_point2.x = edge.rows*left_lines[i].m_slope + left_lines[i].m_intersect;;
                    }
                    if(right_lines_count==0)
                    {
                        right_test_point1.x = right_point_1[0].x;
                        right_test_point2.x = right_point_2[0].x;
                    }
                    else
                    {
                        right_test_point1.x = right_lines[j].m_intersect;
                        right_test_point2.x = edge.rows*right_lines[j].m_slope + right_lines[j].m_intersect;

                    }

                    if ((right_test_point1.x - left_test_point1.x>desitance_above_min) &&
                        (right_test_point1.x - left_test_point1.x<desitance_above_max) &&
                        (right_test_point2.x - left_test_point2.x>desitance_below_min) &&
                        (right_test_point2.x - left_test_point2.x<desitance_below_max))
                    {
                        left_point_1[0].x = left_test_point1.x;
                        left_point_2[0].x = left_test_point2.x;
                        right_point_1[0].x = right_test_point1.x;
                        right_point_2[0].x = right_test_point2.x;
                        update_label = 1;
                    }
                }
            }


        cout << "gap1: " << abs(right_point_1[0].x - left_point_1[0].x) << endl;
        cout << "gap2: " << right_point_2[0].x - left_point_2[0].x << endl;

        if(update_label==0)
		{
			right_point_1[0] = right_point_1[1]; right_point_2[0] = right_point_2[1];
			left_point_1[0] = left_point_1[1]; left_point_2[0] = left_point_2[1];
			two_lanes++;
		}
		else
			two_lanes--;
    }


	if (count == 1){
		KF.statePost.at<float>(0) = left_point_1[0].x;
		KF.statePost.at<float>(1) = left_point_2[0].x;
		KF.statePost.at<float>(2) = right_point_1[0].x;
		KF.statePost.at<float>(3) = right_point_2[0].x;
	}

	if (count > 1)
	{
        int cross_count = 0;
		if ((abs(left_point_1[0].x - left_point_1[1].x) > (desitance_above_min / 2)) || (abs(right_point_1[0].x - right_point_1[1].x) > (desitance_above_min / 2)) || (abs(left_point_2[0].x - left_point_2[1].x) > (desitance_below_min / 2)) || (abs(right_point_2[0].x - right_point_2[1].x) > (desitance_below_min / 2)))
		{
		   //根据在识别出车道线附近检测出横向边缘的数量,过滤掉路面文字标识
            float cen_left_point = (left_point_1[0].x + left_point_2[0].x) / 2;//左车道线中点
            float cen_right_point = (right_point_1[0].x + right_point_2[0].x) / 2;//右车道线中点
           // int cross_count = 0;
            cout<<"cen_left_point: "<<cen_left_point<<endl;
            cout<<"cen_right_point: "<<cen_right_point<<endl;
            for (unsigned int i = 0; i < lines_detect_hor.size(); ++i)
	       {
              if((lines_detect_hor[i].m_line(0) < cen_left_point && lines_detect_hor[i].m_line(2) > cen_left_point) || 
                 (lines_detect_hor[i]. (0) < cen_right_point && lines_detect_hor[i].m_line(2) > cen_right_point))
                 cross_count++;
            }
            cout<<"cross_count: "<<cross_count<<endl;
            if(cross_count>=100)
                lane_change_label = 0;
            else
                 lane_change_label = 1;
           
		}
		else lane_change_label = 0;
		
        if(lane_change_label == 1)
		{
			measurement.at<float>(0) = left_point_1[0].x;
			measurement.at<float>(1) = left_point_2[0].x;
			measurement.at<float>(2) = right_point_1[0].x;
			measurement.at<float>(3) = right_point_2[0].x;

			KF.statePost.at<float>(0) = left_point_1[0].x;
			KF.statePost.at<float>(1) = left_point_2[0].x;
			KF.statePost.at<float>(2) = right_point_1[0].x;
			KF.statePost.at<float>(3) = right_point_2[0].x;	
		}
		else 
		{
			if ((abs(left_point_1[0].x - left_point_1[1].x) > (desitance_above_min / 2)) || (abs(right_point_1[0].x - right_point_1[1].x) > (desitance_above_min / 2)) || (abs(left_point_2[0].x - left_point_2[1].x) > (desitance_below_min / 2)) || (abs(right_point_2[0].x - right_point_2[1].x) > (desitance_below_min / 2)) && (cross_count>=4))
		    {   
				right_point_1[0] = right_point_1[1]; right_point_2[0] = right_point_2[1];
				left_point_1[0] = left_point_1[1]; left_point_2[0] = left_point_2[1];
			}
			
			else
			{

			//2.kalman prediction
			Mat prediction = KF.predict();
			//3.update measurement
			measurement.at<float>(0) = (float)left_point_1[0].x;
			measurement.at<float>(1) = (float)left_point_2[0].x;
			measurement.at<float>(2) = (float)right_point_1[0].x;
			measurement.at<float>(3) = (float)right_point_2[0].x;
			//4.update
			KF.correct(measurement);
			Point2f true_left_p1 = Point2f(KF.statePost.at<float>(0), left_point_1[0].y);
			Point2f true_left_p2 = Point2f(KF.statePost.at<float>(1), left_point_2[0].y);
			Point2f true_right_p1 = Point2f(KF.statePost.at<float>(2), right_point_1[0].y);
			Point2f true_right_p2 = Point2f(KF.statePost.at<float>(3), right_point_2[0].y);
			right_point_2[0] = true_right_p2;
			right_point_1[0] = true_right_p1;
			left_point_2[0] = true_left_p2;
			left_point_1[0] = true_left_p1;
            }

			//若预测的值与上一帧的值偏差过大,则取历史值
			if (abs(left_point_1[0].x - left_point_1[1].x) > (desitance_above_min / 3) || abs(left_point_2[0].x - left_point_2[1].x) > (desitance_below_min / 3) || abs(right_point_1[0].x - right_point_1[1].x) > (desitance_above_min /3) || abs(right_point_2[0].x - right_point_2[1].x) > (desitance_below_min / 3) \
				|| (right_point_1[0].x - left_point_1[0].x< desitance_above_min) || (right_point_1[0].x - left_point_1[0].x>desitance_above_max) || (right_point_2[0].x - left_point_2[0].x<desitance_below_min) || (right_point_2[0].x - left_point_2[0].x>desitance_below_max))
			{
				right_point_1[0] = right_point_1[1]; right_point_2[0] = right_point_2[1];
				left_point_1[0] = left_point_1[1]; left_point_2[0] = left_point_2[1];
			}

			
		}
	

		//=======================根据之前提取出直线位置,在边缘图中选取直线附近的点,用于曲线拟合=========================
		//左车道线
		float dly = (float)(edge.rows);
		float dlx = (float)(left_point_2[0].x - left_point_1[0].x);
		float slope_l = dlx / dly;
		float intersect_l = (left_point_1[0].x*left_point_2[0].y) / (edge.rows);
		float my_left_count = 0;
		for (int y = 0; y < edge.rows; y = y + 5)
		{
			bool my_left = false;
			float temp_x = slope_l*y + intersect_l;
			for (int x = slope_l*y + intersect_l - 4; x < slope_l*y + intersect_l + 4; x = x + 1)
			{
				if ((x>0) && (x<edge.cols))
				{
					int gray_value = edge.at<uchar>(y, x);
					if (gray_value >0)
					{
						temp_x = (temp_x + x) / 2;
						my_left = true;
					}
				}
			}
			left_ransac.push_back(Point2f(y, temp_x));
			if (my_left == true){
				my_left_count = my_left_count + 1;
			}
		}
		//右车道线
		float dry = (float)(edge.rows);
		float drx = (float)(right_point_2[0].x - right_point_1[0].x);
		float slope_r = drx / dry;
		float intersect_r = (right_point_1[0].x*right_point_2[0].y) / (edge.rows);
		float my_right_count = 0;
		for (int y = 0; y < edge.rows; y = y + 5)
		{
			bool my_right = false;
			float temp_x = slope_r*y + intersect_r;

			for (int x = slope_r*y + intersect_r - 4; x < slope_r*y + intersect_r + 4; x = x + 1)
			{
				if ((x>0) && (x<edge.cols))
				{
					int gray_value = edge.at<uchar>(y, x);
					if (gray_value >0)
					{
						temp_x = (temp_x + x) / 2;
						my_right = true;
					}
				}
			}
			right_ransac.push_back(Point2f(y, temp_x));

			if (my_right == true){
				my_right_count = my_right_count + 1;
			}
		}
		//==================根据线长判断实线与虚线===================
		if ((my_left_count / edge.rows) < 0.15){
			final_back_lines.left_solid_line = 1;
		}
		else {
			final_back_lines.left_solid_line = 0;
		}

		if ((my_right_count / edge.rows) < 0.15){
			final_back_lines.right_soild_line = 1;
		}
		else {
			final_back_lines.right_soild_line = 0;
		}

		//==================================三次曲线拟合=========================================
	    float pl[3] = { 0 }, pr[3] = { 0 };
		fittingCurve(left_ransac, 3, pl);
		fittingCurve(right_ransac, 3, pr);

		for (int y = 0; y < edge.rows; y = y + 1) {
			float x = pl[2] * y*y + pl[1] * y + pl[0];
			final_per_point_fit_left.push_back(Point2f(x, y) + Point2f(boundingBox.x, boundingBox.y));
		}

		for (int y = 0; y < edge.rows; y = y + 1) {
			float x = pr[2] * y*y + pr[1] * y + pr[0];
			final_per_point_fit_right.push_back(Point2f(x, y) + Point2f(boundingBox.x, boundingBox.y));
		}

		Point2f draw_left1 = left_point_1[0] + Point2f(boundingBox.x, boundingBox.y);
		Point2f draw_left2 = left_point_2[0] + Point2f(boundingBox.x, boundingBox.y);
		Point2f draw_right1 = right_point_1[0] + Point2f(boundingBox.x, boundingBox.y);
		Point2f draw_right2 = right_point_2[0] + Point2f(boundingBox.x, boundingBox.y);

		final_per_point[0] = draw_left1;
		final_per_point[1] = draw_left2;
		final_per_point[2] = draw_right1;
		final_per_point[3] = draw_right2;

		Point2f left1pnt;
		Point2f left2pnt;
		Point2f right1pnt;
		Point2f right2pnt;

		left2pnt.x = left_point_1[0].x;
		left2pnt.y = 310- left_point_1[0].y;

		left1pnt.x = left_point_2[0].x;
		left1pnt.y = 310 - left_point_2[0].y;
		
		right2pnt.x= right_point_1[0].x;
		right2pnt.y = 310 - right_point_1[0].y;
		
		right1pnt.x = right_point_2[0].x;
		right1pnt.y = 310 - right_point_2[0].y;

		//车辆中心距离left车道线的距离
		float inversek_Left = (left1pnt.x - left2pnt.x) / (left1pnt.y - left2pnt.y);
		float Ang_of_line_Left = atan(inversek_Left);
		float UnpivotvisionAngle_Left = 180 * Ang_of_line_Left / L_PI; //逆透视图像坐标下的角度
		double WorldAngle1 = (double)UnpivotvisionAngle_Left;            //世界坐标系下的偏航角
		_ANGLEl = WorldAngle1;	//angle yinyong
		int LaneWidthNi = right1pnt.x - left1pnt.x;          //逆透视下左右两条车道线的图像上的宽度
		float Lane2LeftNi = 0.0f; //逆透视图像下距离左侧车道线的距离

		//计算距离的两种情况：
		//1.当车辆行驶方向与车道线有一个夹角时：
		if((inversek_Left>0.000001) || (inversek_Left<-0.000001))
		{
			float b = left2pnt.y - 1/ inversek_Left*left2pnt.x;   // y1 – k*x1
			float x = (-10 - b)*inversek_Left; //计算当y=-10时，x的值：x=(y-b)*(1/k)得到x的值。
			Lane2LeftNi = boundingBox.width/2 - x;  
		}
		else   //2.车辆行驶方向与车道线平行
		{
			Lane2LeftNi = boundingBox.width / 2 - left2pnt.x;
		}
		
		float Lane2LeftW = Lane2LeftNi*3.1/ LaneWidthNi;//世界坐标系下车辆中心距离左侧车道线的距离。
		if (framecnt_w < 980)
		{
			Distance_W_Left[framecnt_w] = Lane2LeftW;
		}

		//车辆中心距离右侧车道线的距离
		float inversek_right = (right1pnt.x - right2pnt.x) / (right1pnt.y - right2pnt.y);
		float Ang_of_line_Right = atan(inversek_right);
		float UnpivotvisionAngle_Right = 180 * Ang_of_line_Right / L_PI; //逆透视图像坐标下的角度
		double WorldAngle2 = (double)UnpivotvisionAngle_Right;            //世界坐标系下的偏航角
		_ANGLEr = WorldAngle2;
		//int LaneWidthNi = right1pnt.x - left1pnt.x;          //逆透视下左右两条车道线的图像上的宽度
		float Lane2RightNi = 0.0f; //逆透视图像下距离左侧车道线的距离

								  //计算距离的两种情况：
								  //1.当车辆行驶方向与车道线有一个夹角时：
		if ((inversek_right>0.000001) || (inversek_right<-0.000001))
		{
			float b = right2pnt.y - 1 / inversek_right*right2pnt.x;   // y1 – k*x1
			float x = (-10 - b)*inversek_right; //计算当y=-10时，x的值：x=(y-b)*(1/k)得到x的值。
			Lane2RightNi = x-boundingBox.width / 2;
		}
		else   //2.车辆行驶方向与车道线平行
		{
			Lane2RightNi = right2pnt.x - boundingBox.width / 2;
		}

		float Lane2RightW = Lane2RightNi*3.1 / LaneWidthNi;//世界坐标系下车辆中鑫距离左侧车道线的距离。
		if (framecnt_w < 980)
		{
			Distance_W_Right[framecnt_w++] = Lane2RightW;
		}

		final_back_lines.m_fCentre2LeftDist = Lane2LeftW;
		final_back_lines.m_fCentre2RightDist = Lane2RightW;
		final_back_lines.m_fAngle = (Ang_of_line_Right+ Ang_of_line_Left)/2*(-1);

		if (Lane2LeftW > 4.0)
		{
			cout << "lane departure to right" << endl;
		}
		
		//cout<<"gap1====="<<right_point_1[0].x - left_point_1[0].x<<endl;
		//cout<<"gap2======"<<right_point_2[0].x - left_point_2[0].x<<endl;

		// =================================逆透视图像中临车道的位置=====================================
		//逆透视图像中将左右车道线线向左右推,作为临车道待选区域
		final_per_point_second[0].x = final_per_point[0].x - 55; final_per_point_second[0].y = final_per_point[0].y;
		final_per_point_second[1].x = final_per_point[1].x - 55; final_per_point_second[1].y = final_per_point[1].y;
		final_per_point_second[2].x = final_per_point[2].x + 55; final_per_point_second[2].y = final_per_point[2].y;
		final_per_point_second[3].x = final_per_point[3].x + 55; final_per_point_second[3].y = final_per_point[3].y;

		//统计漏检,误检的累计次数
		if (right_lines.size() > 0 && left_lines.size() > 0)
		{
			if ((right_point_1[0].x - left_point_1[0].x<desitance_above_min) ||
			(right_point_1[0].x - left_point_1[0].x>desitance_above_max) ||
			(right_point_2[0].x - left_point_2[0].x<desitance_below_min) ||
			(right_point_2[0].x - left_point_2[0].x>desitance_below_max))
			{
				lane_line_problem++;
			}
			else
			{
				//if (right_lines.size() > 3 || left_lines.size() > 3)
                if(right_lines[0].m_AveragePix < 100 && right_lines.size() > 3
                        || left_lines[0].m_AveragePix < 100 && left_lines.size() > 3)
                {
					lane_line_problem++;
				}
				else
				{
					lane_line_problem--;
				}
				lane_miss_count--;
			}
		}
		else
		{
			lane_miss_count++;
		}

		if (lane_miss_count>15)
		{
			lane_miss_count = 15;
		}
		else if (lane_miss_count<0)
		{
			lane_miss_count = 0;
		}

		if (lane_line_problem>9)
		{
			lane_line_problem = 9;
		}
		else if (lane_line_problem<0)
		{
			lane_line_problem = 0;
		}

		if (two_lanes>10)
		{
			two_lanes = 10;
		}
		else if (two_lanes<0)
		{
			two_lanes = 0;
		}
		
		cout<<"lane_line_problem: "<<lane_line_problem<<endl;
		cout<<"lane_miss_count: "<<lane_miss_count<<endl;
		cout<<"two_lanes: "<<two_lanes<<endl;

		//若漏检,误检的累计次数超过某一个限定的值,则将标志位设为2
		if ( (lane_line_problem>7)||lane_miss_count > 11 ||(two_lanes > 8))
		{
			final_back_lines.right_soild_line = 2;
			final_back_lines.left_solid_line = 2;
		}
		//===========================================邻车道线检测============================================
		// 调整左右框的xy
		Rect left_boundingBox; //define the left second boundingbox
		Rect right_boundingBox; //define the right second boundingbox
		// 负值左边移动
		left_boundingBox.x = final_per_point_second[0].x - 20;
		left_boundingBox.y = final_per_point_second[0].y - 20;
		left_boundingBox.width = 30;
		left_boundingBox.height = final_per_point[1].y - final_per_point[0].y;

		right_boundingBox.x = final_per_point_second[2].x - 8;
		right_boundingBox.y = final_per_point_second[2].y - 20;
		right_boundingBox.width = 15;
		right_boundingBox.height = final_per_point[3].y - final_per_point[2].y;
		Mat left_temp_image = perspective(left_boundingBox);
		Mat right_temp_image = perspective(right_boundingBox);

		Mat left_vaild_image = left_temp_image.clone();
		Mat right_vaild_image = right_temp_image.clone();
		vector<Vec4i> left_temp_lines;
		vector<Vec4i> right_temp_lines;
		Canny(left_vaild_image, left_vaild_image, 20, 50, 3);
		Canny(right_vaild_image, right_vaild_image, 20, 50, 3);
		HoughLinesP(left_vaild_image, left_temp_lines, 2, CV_PI / 180, 40, 40, 40);
		HoughLinesP(right_vaild_image, right_temp_lines, 2, CV_PI / 180, 40, 40, 40);
		//cout << "left_temp_lines.size()" << left_temp_lines.size() << endl;
		//cout << "right_temp_lines.size()" << right_temp_lines.size() << endl;

		//在指定的区域内检测临车道是否存在
		for (size_t i = 0; i < left_temp_lines.size(); ++i)
		{
			Vec4i l = left_temp_lines[i];
			float dy = (float)((float)l[3] - (float)l[1] + 0.000001);
			float dx = (float)((float)l[2] - (float)l[0]);
			slope = dx / dy;

			if ((slope<0.13) && (slope>-0.13))
			{
				final_back_lines.left_second_valid = true;
				line(left_vaild_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 2);
			}

		}
		for (size_t i = 0; i < right_temp_lines.size(); ++i)
		{
			Vec4i l = right_temp_lines[i];
			float dy = (float)((float)l[3] - (float)l[1] + 0.000001);
			float dx = (float)((float)l[2] - (float)l[0]);
			slope = dx / dy;
			if ((slope<0.13) && (slope>-0.13))
			{
				final_back_lines.right_second_valid = true;
				line(right_vaild_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 2);
			}
		}

        //理想的左右车道线位置（直接将逆透视图像中左右车道线的端点变换至原图,用于后期距离计算）
		perspectiveTransform(final_per_point, final_back_lines.center_lines, back_transform);
		//理想的邻车道位置（直接将逆透视图像中左右车道线的端点变换至原图）
		perspectiveTransform(final_per_point_second, final_back_lines.second_lines, back_transform);
        //实际的左右车道线（将逆透视图像中拟合出的曲线变换至原图,用于原图上的车道线绘制）
		perspectiveTransform(final_per_point_fit_left, final_back_lines.left_lines, back_transform);
		perspectiveTransform(final_per_point_fit_right, final_back_lines.right_lines, back_transform);
		}

		//保存前两帧车道线识别的结果
		right_point_1[2] = right_point_1[1]; 
		right_point_2[2] = right_point_2[1];
		left_point_1[2]  = left_point_1[1]; 
		left_point_2[2]  = left_point_2[1];
		right_point_1[1] = right_point_1[0]; 
		right_point_2[1] = right_point_2[0];
		left_point_1[1]  = left_point_1[0]; 
		left_point_2[1]  = left_point_2[0];


	if(_DEBUG)
	{
		_Debug_Image.push_back(perspective);
	}
	imshow("perspective", perspective);
	waitKey(1);
	return final_back_lines;
}
#endif

