/*
 * Jan. 23, 2017, David Z
 *
 * Functions to generate point cloud using images 
 *
 * */

#ifndef PC_FROM_IMAGE_H
#define PC_FROM_IMAGE_H

#include "glob_def.h"
#include "cam_model.h"
#include <opencv2/opencv.hpp>

extern void generatePointCloud(cv::Mat& rgb, cv::Mat& dpt, float dpt_scale, CamModel& , pcl::PointCloud<pcl::PointXYZRGBA>&, int skip = 1); 


#endif
