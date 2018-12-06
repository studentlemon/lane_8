#include "opencv2/opencv.hpp"
#include <fstream>
#include <math.h>
using namespace cv;
using namespace std;


#define PI 3.141592653
#define	fv 1754.3 
#define lostpoint_x 320
#define lostpoint_y 135.6552
#define image_width 640
#define image_height 480

double h;
double v0=270.6609;
double pitch_angle_1, pitch_angle_2, pitch_angle_3, pitch_angle_4,pitch_angle_5,pitch_angle_6,pitch_angle_7,pitch_angle_8,
pitch_angle_9,pitch_angle_10,pitch_angle_11,pitch_angle_12,pitch_angle_13,pitch_angle_14,pitch_angle_15,pitch_angle_average,pitch_angle;
double first_pix,second_pix,third_pix, forth_pix,fith_pix,six_pix,seven_pix,ehight_pix,nigh_pix,ten_pix,eleven_pix,twelve_pix,thirteen_pix,fourteen_pix,fifteen_pix;

double calculate_distance(double pixels_y)
{
	h = 1410;
	pitch_angle_1 = 0.909047;
	pitch_angle_2 = 0.862837;
	pitch_angle_3 = 0.862966;
	pitch_angle_4 = 0.847733;
	pitch_angle_5 = 0.887467;
	pitch_angle_6 = 0.89136;
	pitch_angle_7 = 0.935312;
	pitch_angle_8 = 0.872445;
	pitch_angle_9 = 0.892674;
	pitch_angle_10 = 0.944659;
	pitch_angle_11 = 0.902555;
	pitch_angle_12 = 0.938366;
	pitch_angle_13 = 0.945198;
	pitch_angle_14 = 0.95966;
	pitch_angle_15 = 1.02436;
	pitch_angle_average=(pitch_angle_1+pitch_angle_2+pitch_angle_3+pitch_angle_4+pitch_angle_5+pitch_angle_6+pitch_angle_7+pitch_angle_8+pitch_angle_9+pitch_angle_10+pitch_angle_11+pitch_angle_12+pitch_angle_13+pitch_angle_14+pitch_angle_15)/15;
		
		
	first_pix = 216;
	second_pix = 220;
	third_pix = 223;
	forth_pix = 227;
	fith_pix = 230;
	six_pix = 235;
	seven_pix = 240;
	ehight_pix = 250;
	nigh_pix = 260;
	ten_pix = 273;
	eleven_pix = 286;
	twelve_pix = 300;
	thirteen_pix = 320;
	fourteen_pix = 348;
	fifteen_pix = 389;

	double longitudinal;
	if( pixels_y> (image_height-first_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_1+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
	}	
	
	else if(pixels_y< (image_height-first_pix) && pixels_y >=(image_height-second_pix)  )
	{
		longitudinal=h/tan(  (pitch_angle_2+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
	}
		
	else if( pixels_y< (image_height-second_pix) && pixels_y >= (image_height-third_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_3+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
    	}
	
	else if(pixels_y< (image_height-third_pix) && pixels_y>= (image_height-forth_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_4+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
    	}
		
	else if(pixels_y< (image_height-forth_pix) && pixels_y>= (image_height-fith_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_5+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
    	}
		
	else if(pixels_y< (image_height-fith_pix) && pixels_y>= (image_height-six_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_6+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
    	}

    	else if(pixels_y< (image_height-six_pix) && pixels_y>= (image_height-seven_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_7+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
    	}

	else if(pixels_y< (image_height-seven_pix) && pixels_y>= (image_height-ehight_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_8+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
    	}

	 else if(pixels_y< (image_height-ehight_pix) && pixels_y>= (image_height-nigh_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_9+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
    	}

    	else if(pixels_y< (image_height-nigh_pix) && pixels_y>= (image_height-ten_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_10+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
	}
    		
	else if(pixels_y< (image_height-ten_pix) && pixels_y>= (image_height-eleven_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_11+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
	}

    	else if(pixels_y< (image_height-eleven_pix) && pixels_y>= (image_height-twelve_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_12+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
	}

    	else if(pixels_y< (image_height-twelve_pix) && pixels_y>= (image_height-thirteen_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_13+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
	}
    	
	else if(pixels_y< (image_height-thirteen_pix) && pixels_y>= (image_height-fourteen_pix) )
	{
		longitudinal=h/tan(  (pitch_angle_14+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
	}
 
   	else
	{
		longitudinal=h/tan(  (pitch_angle_15+atan(-(pixels_y-v0)/fv)*180/PI)*PI/180     );
   	}		
	
	longitudinal=longitudinal/1000;

	if(longitudinal<0){longitudinal=5;}
	
	return longitudinal;

}


