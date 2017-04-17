/* 
 * Feb. 16  2017, David Z 
 *
 * Show the extracted planes 
 *
 * */

#include <ros/ros.h>
#include <string> 
#include <tf/tf.h>
#include <iostream>
#include <map>
#include <pcl/io/pcd_io.h>
#include "SR_reader_cv.h"
#include "glob_def.h"
#include "pc_from_image.h"
#include "vtk_viewer.h"
#include "plane.h"
#include "plane_node.h"

using namespace std; 

string g_file_dir; 

void view_frame(int idi); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "view_planes"); 
  ros::NodeHandle n;
  ros::NodeHandle nh("~"); 

  int frame_id; 
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k")); // default parameters
  nh.param("frame_id", frame_id, 7); 
  
  if(argc > 1) 
    frame_id = atoi(argv[1]); 

  view_frame(frame_id);
  return 0; 

}
void view_frame(int idi)
{
  // getframe 
    stringstream sni, snj; 
    sni << g_file_dir<<"/d1_"<<setfill('0')<<setw(7)<<idi<<".bdat"; 
    
    // CSReader r4k; 
    CSReadCV r4k; 

    // generate imgs and dpts 
    cv::Mat tar_cv_d_img(SR_HEIGHT, SR_WIDTH, CV_16UC1); 
    cv::Mat tar_cv_i_img; 

    if(!r4k.readOneFrameCV(sni.str().c_str(), tar_cv_i_img, tar_cv_d_img))
    {
      ROS_INFO("%s failed to read file %s", __FILE__, sni.str().c_str()); 
      return ;
    }

    CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
    sr4k.z_offset = 0.015;  // this is only for sr4k 
    sr4k.setDepthScale(0.001); 
 
    // generate plane node 
    CPlaneNode* pni = new CPlaneNode(); 
    int ret = pni->extractPlanes(tar_cv_i_img, tar_cv_d_img, &sr4k); 
    if(ret <= 0)
    {
      ROS_INFO("what? pni has no plane ,idi = %d", idi); 
      return ;
    }else{
      ROS_INFO("pni has %d planes ", pni->mv_planes.size()); 
    }
    

    // show them in point cloud 
    CloudPtr pci(new Cloud); 
    generatePointCloud(tar_cv_i_img, tar_cv_d_img, 0.001, sr4k, *pci);
    
    int c = RED; 
    int pre_n = pni->mv_planes.size(); 
    for(int i=0; i<pni->mv_planes.size(); i++)
    {
      CPlane* p = pni->mv_planes[i];
      ROS_INFO("p1 plane %d has points %d nv: %f %f %f d: %f", i, pni->mv_indices[i].size(), p->nx_, p->ny_, p->nz_, p->d1_);
      markColor(*pci, pni->mv_indices[i], static_cast<COLOR>(c++%5)); 
    }

    // save pci 
    pcl::io::savePCDFile(string("point_cloud.pcd"), *pci); 

    // markColor(*pci, pni->mv_indices[best_i], GREEN); 
    // markColor(*pcj_ni, pnj->mv_indices[best_j], RED); 
    // markColor(*pci,  GREEN); 
    // markColor(*pcj_ni, RED); 
 
    CVTKViewer<pcl::PointXYZRGBA> v;
    // v.getViewer()->addCoordinateSystem(0.2, 0, 0); 
    v.addPointCloud(pci, "pci"); 
    while(!v.stopped())
    {
      v.runOnce(); 
      usleep(100*1000); 
    } 
}
