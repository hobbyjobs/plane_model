#include "pc_from_image.h"
#include <climits>

// assmue that the size(rgb) == size(dpt), 
void generatePointCloud(cv::Mat& rgb, cv::Mat& dpt, float dpt_scale, CamModel& cam, pcl::PointCloud<pcl::PointXYZRGBA>& pc, int skip ) 
{  
  double z; 
  double px, py, pz; 
  int height = rgb.rows/skip; 
  int width = rgb.cols/skip; 
  int N = (rgb.rows/skip)*(rgb.cols/skip); 

  pc.points.reserve(N); 
  // pc.width = width; 
  // pc.height = height; 

  unsigned char r, g, b; 
  int pixel_data_size = 3; 
  if(rgb.type() == CV_8UC1)
  {
    pixel_data_size = 1; 
  }
  
  int color_idx; 
  char red_idx = 2, green_idx =1, blue_idx = 0;

  Point pt; 

  for(int v = 0; v<rgb.rows; v+=skip)
  for(int u = 0; u<rgb.cols; u+=skip)
  {
    // Point& pt = pc.points[v*width + u]; 
    z = dpt.at<unsigned short>((v), (u))*dpt_scale;
    if(std::isnan(z) || z <= 0.0) 
    {
      pt.x = std::numeric_limits<float>::quiet_NaN();  
      pt.y = std::numeric_limits<float>::quiet_NaN();  
      pt.z = std::numeric_limits<float>::quiet_NaN();  
      continue; 
    }
    cam.convertUVZ2XYZ(u, v, z, px, py, pz); 
  
    pt.x = px;  pt.y = py;  pt.z = pz; 
    color_idx = (v*rgb.cols + u)*pixel_data_size;
    if(pixel_data_size == 3)
    {
      r = rgb.at<uint8_t>(color_idx + red_idx);
      g = rgb.at<uint8_t>(color_idx + green_idx); 
      b = rgb.at<uint8_t>(color_idx + blue_idx);
    }else{
      r = g = b = rgb.at<uint8_t>(color_idx); 
    }
    pt.r = r; pt.g = g; pt.b = b; 
    pc.points.push_back(pt); 
  }
  pc.height = 1; 
  pc.width = pc.points.size(); 
  return ;

}

