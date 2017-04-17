/*
 * Jan. 23, 2017, David Z 
 *
 * main function: 
 *  
 *  1. extract plane segment, given dpt image, show the result on the intensity image 
 *  2. compute the covariance of the plane 
 *
 * */

#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include "plane.h"
#include "plane_set.h"
#include "vtk_viewer.h"
#include "pc_from_image.h"
#include "SR_reader_cv.h"

string g_file_dir = "/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k";

void plane_on_sr4k(int i = 7); // show the plane extraction 
void test_cam_model(); // test whether the cam model is right 
void copyMat(cv::Mat& f, cv::Mat& t); 

void test_plane_cov(); // test covariance computation of a plane 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "main_plane"); 
  ros::NodeHandle n; 
  // test_cam_model(); 

  for(int i= 1000; i<3000; i+= 7)
    plane_on_sr4k(i); 
  
  // test_plane_cov(); 

  return 0; 
}

void test_plane_cov()
{
    CloudPtr cloud(new Cloud); 
    CPointF3 p1(1, 4, 1); CPointF3 p2(2, 3, -1); CPointF3 p3(3, 2, 1); CPointF3 p4(4, 1, -1); 
    vector<CPointF3> vp(4); vp[0] = p1; vp[1] = p2; vp[2] = p3; vp[3] = p4; 

    toPC(vp, cloud); 
    CPlane p;
    cv::Mat tmp; 
    vector<int> index; 
    // p.computeCOV(tmp, cloud, index); 
    p.computeCOVDense(cloud); 
    return ;

}


void plane_on_sr4k(int i)
{
  stringstream ss; ss<<g_file_dir<<"/d1_"<<setfill('0')<<setw(7)<<i<<".bdat";  // setw(4)
  string data_file = ss.str(); 
  cv::Mat rgb, dpt; 
 
  CSReadCV r4k; 
  if(!r4k.readOneFrameCV(data_file, rgb, dpt))
  {
    ROS_INFO("%s failed to read file %s ", __FILE__, data_file.c_str()); 
    return; 
  }

  // cam model for sr4k  
  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  float dpt_scale = 0.001; 

  // generate point cloud 
  CloudPtr pc(new Cloud); 
  generatePointCloud(rgb, dpt, dpt_scale, sr4k, *pc); 

  // extract planes and show it on the image 
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  CPlaneSet* pp = new CPlaneSet(); 
  int num_P =  pp->extractPlanes(pc, sr4k); 

  // show the result 
  cv::Mat result = cv::Mat(rgb.rows, rgb.cols, CV_8UC3); 
  copyMat(rgb, result); 

  int c = RED; 
  int NC = 6;
  map<int, vector<int> > PATCH; 
  map<int, int> PATCH_CENTER; 
  map<int, vector<int> >::iterator it; 
  int s = 4; 
  for(int i=0; i<num_P; i++)
  {
    // markColor(result, pp->mv_indices[i], static_cast<COLOR>(c++)); 
    patchPointsInPlaneArea(rgb.rows, rgb.cols, pp->mv_indices[i], s, PATCH, PATCH_CENTER); 
    
    // mark color for each PATH 
    vector<int> patch_center_indice; 
    for(it = PATCH.begin(); it!=PATCH.end(); it++)
    {
      vector<int>& PI = it->second; 
      vector<int> patch_indice(PI.size()); 

      for(int j=0; j<it->second.size(); j++)
        patch_indice[j] = (pp->mv_indices[i][PI[j]]); 
      markColor(result, patch_indice, static_cast<COLOR>(c++%NC)); 
      patch_center_indice.push_back(pp->mv_indices[i][PATCH_CENTER[it->first]]); 
    }
  
    // cout <<"Let's markcolor of the patch centrals"<<endl;
    
    // mark the key central points 
    // markColor(result, patch_center_indice, static_cast<COLOR>(c++%NC)); 
    markColor(result, patch_center_indice, BLACK); 

    PATCH.clear(); 
    PATCH_CENTER.clear(); 
  }
  
  cv::imshow("plane_result", result);
  cv::waitKey(0); 

  return ; 
}


void copyMat(cv::Mat& f, cv::Mat& t)
{
  if(f.type() == t.type() )
  {
    t = f.clone();
    return ;
  }

  int color_index, grey_index; 
  int rgb_size = 3; 
  int grey_size =1; 
  if(f.type() == CV_8UC1 && t.type() == CV_8UC3)
  {
    for(int r=0; r<t.rows; r++)
    for(int c=0; c<t.cols; c++)
    {
      color_index = (r*t.cols + c)*rgb_size; 
      grey_index = (r*t.cols + c)*grey_size; 
      t.at<uint8_t>(color_index + 2) = f.at<uint8_t>(grey_index);  // r
      t.at<uint8_t>(color_index + 1) = f.at<uint8_t>(grey_index);  // g 
      t.at<uint8_t>(color_index + 0) = f.at<uint8_t>(grey_index);  // b
    }
  }else
  {
    ROS_ERROR("%s TODO: copyMat with other Mat types", __FILE__); 
  }
  return ;
}


void test_cam_model()
{
  // cam model for sr4k  
  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  
  double x, y, z, ez; 
  float eu, ev, au, av;
  ez = 1;
  eu = ev = 124; 
  sr4k.convertUVZ2XYZ(eu, ev, ez, x, y, z); 
  sr4k.convertXYZ2UV(x, y, z, au, av); 
     
  ROS_INFO("eu = %f ev = %f au = %f av = %f", eu, ev, au, av);

  return ; 
}

/*

  for(int i=0; i<inliers->indices.size(); i++)
  {
    int eu, ev, au, av;
    int index = inliers->indices[i]; 
    ev = index/rgb.cols; eu = index - ev*rgb.cols; 
    Point& pt = pc->points[index]; 
    float u, v; 
    if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) // invalid point 
      continue;
    if(pt.z <= 0) // behind the camera  
      continue; 
    sr4k.convertXYZ2UV(pt.x, pt.y, pt.z, u, v); 
    int a_index = ((int)(v))*rgb.cols + (int)(u);  
    
    ROS_INFO("i = %d eu = %d ev = %d u = %f v = %f e_index = %d a_index = %d", i, eu, ev, u, v, index, a_index);

  }

*/
