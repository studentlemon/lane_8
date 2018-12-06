#ifndef MEASURE_DISTANCE_HPP_
#define MEASURE_DISTANCE_HPP_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <map>

using namespace cv;
using namespace std;

std::map<int, float> find_distance;
map<int, float>::iterator iter_1;
map<int, float>::iterator iter_2;

void find_distance_initialization(){

	find_distance.insert(make_pair<int, float>(140, 61));
	find_distance.insert(make_pair<int, float>(150, 43));
	find_distance.insert(make_pair<int, float>(160, 33));
	find_distance.insert(make_pair<int, float>(170, 26));
	find_distance.insert(make_pair<int, float>(180, 22));
	find_distance.insert(make_pair<int, float>(190, 18));
	find_distance.insert(make_pair<int, float>(200, 16));
	find_distance.insert(make_pair<int, float>(210, 14));
	find_distance.insert(make_pair<int, float>(220, 12.5));
	find_distance.insert(make_pair<int, float>(230, 11.3));
	find_distance.insert(make_pair<int, float>(240, 10.5));
	find_distance.insert(make_pair<int, float>(250, 9.4));
	find_distance.insert(make_pair<int, float>(260, 8.6));
	find_distance.insert(make_pair<int, float>(270, 7.8));
	find_distance.insert(make_pair<int, float>(280, 7.2));
	find_distance.insert(make_pair<int, float>(290, 6.8));
	find_distance.insert(make_pair<int, float>(300, 6.3));
	find_distance.insert(make_pair<int, float>(310, 5.9));
	find_distance.insert(make_pair<int, float>(320, 5.45));
	find_distance.insert(make_pair<int, float>(330, 5.1));
	find_distance.insert(make_pair<int, float>(340, 4.8));
	find_distance.insert(make_pair<int, float>(350, 4.5));
	find_distance.insert(make_pair<int, float>(360, 4.2));
	find_distance.insert(make_pair<int, float>(370, 4));
	find_distance.insert(make_pair<int, float>(380, 3.8));
	find_distance.insert(make_pair<int, float>(390, 3.6));
	find_distance.insert(make_pair<int, float>(400, 3.4));
	find_distance.insert(make_pair<int, float>(410, 3.2));
	find_distance.insert(make_pair<int, float>(420, 3));
	find_distance.insert(make_pair<int, float>(430, 2.8));
	find_distance.insert(make_pair<int, float>(440, 2.7));
	find_distance.insert(make_pair<int, float>(450, 2.6));
	find_distance.insert(make_pair<int, float>(460, 2.5));
	find_distance.insert(make_pair<int, float>(470, 2.4));
	find_distance.insert(make_pair<int, float>(480, 2.3));
}

float distance_calculate(Rect &region, Point2f &location ){
	/* unit is meter*/
	float distance;
	float real_location;
	int real_int_location;
	real_location = region.y + location.y; //480 - region.y - region.height
	if (real_location<140){
		distance = 61;
	}
	else if (real_location>479){
		distance = 2.3;
	}else{
		real_int_location = floor(real_location / 10) * 10;//floor means return integer value<=real location
		iter_1 = find_distance.find(real_int_location);
		iter_2 = find_distance.find(real_int_location+10);
		distance = ((iter_1->second) - (iter_2->second))*(real_int_location + 10 - real_location) / 10 + (iter_2->second);
	}
	return distance;
}

#endif
