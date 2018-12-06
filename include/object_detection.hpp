#ifndef OBJECT_DETECTION_HPP_
#define OBJECT_DETECTION_HPP_

#include <iostream>
#include <string>
#include <math.h>
#include <glog/logging.h>
#include "caffe/caffe.hpp"
#include <opencv2/opencv.hpp>

using namespace caffe;
using namespace std;

using caffe::Timer;


class SSD_Detector{
public:
  SSD_Detector(const string& model_prototxt_file, const string& trained_caffemodel_file);

  class Object{
  public:
    Object(cv::Rect& bbox, int label)
	{
      this->boundingbox = bbox;
      this->label = label;
    }
    Object(){}
    cv::Rect boundingbox;
    int label;
    cv::Point2d dist;
	float probability;
    string position;
  };

  vector<int> common_object;
  int FCW_object_index;
  int FCW_label;

  vector<Object> detect(cv::Mat image, const int size, float CONF_THRESH = 0.5);

  vector<Object> Obj_pool;

  vector<string> CLASSES;

private:
  shared_ptr<Net<float> > ssd_net;
};

SSD_Detector::SSD_Detector(const string& model_prototxt_file, const string& trained_caffemodel_file)
{
  ssd_net = shared_ptr<Net<float> >(new Net<float>(model_prototxt_file, caffe::TEST));
  ssd_net ->CopyTrainedLayersFrom(trained_caffemodel_file);
}

//[#] 11.2 Start___________________________________
int tr_count = 0;  // tracking 计数器
int tr_step = 1;   // 每隔1帧检测1次
bool tr_need_to_detect = true;
int tr_detect_number = 0;

class Tracking
{
public:
	Tracking()
	{
		init = true;
		isInImage = true;
		isInRoi = false;
		tracker = KCFTracker(true,true,true,true);
	}
	KCFTracker tracker;
	bool init;
	bool isInImage;
	bool isInRoi;
	cv::Rect Boundingbox;
	string cls; // label
	int label;
	cv::Scalar color; // different color
	virtual  ~Tracking() {}
};
//[#] 11.2 End___________________________________


vector<SSD_Detector::Object> SSD_Detector::detect(cv::Mat src, const int size, float CONF_THRESH)
{
  if(src.empty()){
    LOG(FATAL)<<"Input image is empty !";
  }

  cv::Mat image;
  cv::resize(src, image, cv::Size(size,size));

  cv::Mat input_img(image.rows, image.cols, CV_32FC3, cv::Scalar(0,0,0));

  int height = image.rows;
  int width = image.cols;

  float *data_buf = NULL;
  data_buf = new float[image.rows*image.cols*3];
  const float *detection_out = NULL;
  int output_object_number = 0;

  // substract mean value
	CV_Assert(image.depth() != 3);

	// int channels = cv_img.channels();
	int nRows = image.rows;
	int nCols = image.cols;
	if (image.isContinuous())
	{
		nCols *= nRows;
		nRows = 1;
	}
	for (int h = 0; h < nRows; ++h )
	{
		uchar *p_img = image.ptr<uchar>(h);
		float *p_new = input_img.ptr<float>(h);

		// or *p = cv_img.data
		for (int w = 0; w < nCols; w++)
		{
			// cout<<w<<endl;
			p_new[w*3] = float(p_img[w*3])-float(102.9801);
			p_new[w*3+1] = float(p_img[w*3+1])-float(115.9465);
			p_new[w*3+2] = float(p_img[w*3+2])-float(122.7717);
		}
	}

  for (int h = 0; h < height; ++h)
  {
    for (int w = 0; w < width; ++w)
    {
      data_buf[(0*height+h)*width+w] = float(input_img.at<cv::Vec3f>(cv::Point(w, h))[0]);
      data_buf[(1*height+h)*width+w] = float(input_img.at<cv::Vec3f>(cv::Point(w, h))[1]);
      data_buf[(2*height+h)*width+w] = float(input_img.at<cv::Vec3f>(cv::Point(w, h))[2]);
    }
  }

  ssd_net->blob_by_name("data")->Reshape(1, 3, height, width);
  ssd_net->blob_by_name("data")->set_cpu_data(data_buf);

//  double t = (double)cv::getTickCount();

  ssd_net->ForwardFrom(0);
//  t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
//  cout<< "Net forward time is " << t <<endl;

  detection_out = ssd_net->blob_by_name("detection_out")->cpu_data();
  output_object_number = ssd_net->blob_by_name("detection_out")->height();

  this->Obj_pool.clear();
  for (int i = 0; i < output_object_number; i++)
  {
    if(detection_out[i*7+2]>CONF_THRESH)
    {
      SSD_Detector::Object obj;
      int label = detection_out[i*7+1];

      int xmin = round(detection_out[i*7+3] * src.cols );
      int ymin = round(detection_out[i*7+4] * src.rows );
      int xmax = round(detection_out[i*7+5] * src.cols );
      int ymax = round(detection_out[i*7+6] * src.rows );

      //obj.cls = this->CLASSES[label];
      obj.label = label;
	  obj.probability=detection_out[i*7+2];
      obj.boundingbox = cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax));
      this->Obj_pool.push_back(obj);
    }
  }

  delete []data_buf;
  return this->Obj_pool;
}
cv::Rect ScaleBoundBox(cv::Rect src, double scale)
{
	cv::Point2d center=cv::Point2d(src.x+src.width/2,src.y+src.height/2);
	double half_height = scale*src.height/2;
	double half_width = scale*src.width/2;
	cv::Size2d src_size(2*half_width,2*half_height);
	cv::Point2d lt(center.x-half_width,center.y-half_height);
	return cv::Rect(lt,src_size);
}

#endif
