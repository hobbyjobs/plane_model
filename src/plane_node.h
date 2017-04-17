/*
 * Jan. 26, 2017 David Z 
 *
 * Given camera measurement and IMU measurement, detects and associates Plane 
 *
 * */

#ifndef PLANE_NODE_H
#define PLANE_NODE_H

#include <vector>
#include "cam_model.h"
#include "plane.h"
#include "plane_set.h"

class CPlane; 

namespace pcl{
  // class PointCloud; 
  // class PointXYZRGBA; 
}

class CPlaneNode
{
  public:
    CPlaneNode(); 
    virtual ~CPlaneNode(); 
    
    int extractPlanes(cv::Mat& rgb, cv::Mat& dpt, CamModel* ); 
    // int extractPlanesWithPrior(cv::Mat& rgb, cv::Mat& dpt, CamModel*, CPlaneNode* , double sigma_d); 
    
    template<typename PointT>
    int extractPlanes(boost::shared_ptr<pcl::PointCloud<PointT> > &in, CamModel* );

    void setDpt(cv::Mat& dpt) {m_dpt = dpt.clone(); }
    bool empty(){return m_dpt.empty();}
    
    bool mergeOverlappedPlanes(int newly_added); // if the newly added planes coincide with the old ones 

    cv::Mat m_dpt;  // store depth information 
    std::vector<CPlane*> mv_planes;  // store planes 
    std::vector< std::vector<int> > mv_indices; // index indicates the location of the plane points 
    std::vector<int> mv_landmark_id;  // whether this plane has been added into the graph 
  
    // for debug 
    void testEigen(); 
    void testPtOnPlane();  // test the distance of the point to the plane, 

  private: 
    CPlaneNode(const CPlaneNode&) {}
    CPlaneNode operator=(CPlaneNode&){} 
};

#include "plane_node.hpp"

#endif
