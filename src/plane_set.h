/*
 *  Oct. 26, 2015 David Z
 *
 *  collection of planes, 
 *
 * */

#ifndef PLANE_SET_H
#define PLANE_SET_H

#include "plane.h"
#include <vector>

class CPlaneSet
{
  public:
    CPlaneSet();
    ~CPlaneSet();

    void clearPlanes();

    // extract planes
    int extractPlanes(cv::Mat& img, cv::Mat& dpt, CamModel& , double squared_dis_threshold = 0.005);

    template<typename PointT>
    int extractPlanes(boost::shared_ptr<pcl::PointCloud<PointT> >& in, double squared_dis_threshold = 0.005); 

    template<typename PointT>
    int extractPlanes(cv::Mat& img, boost::shared_ptr<pcl::PointCloud<PointT> > &in, CamModel& , double squared_dis_threshold = 0.005);

    template<typename PointT>
    int extractPlanes(boost::shared_ptr<pcl::PointCloud<PointT> > &in, CamModel&, double squared_dis_threshold = 0.005 );

    template<typename PointT>
    void extractOutlier(boost::shared_ptr<pcl::PointCloud<PointT> > &in, boost::shared_ptr<pcl::PointCloud<PointT> > &out, 
        pcl::PointIndices::Ptr& inliers);

   template<typename PointT>
    void extractInlier(boost::shared_ptr<pcl::PointCloud<PointT> > &in, boost::shared_ptr<pcl::PointCloud<PointT> > &out, 
        pcl::PointIndices::Ptr& inliers);

    // select floor 
    CPlane* selectFloor();
    float isHorizontal(CPlane* p);

    double d_percent_threshold_; // number of points below this threshold is not considered as a plane
    int m_num_of_pts_threshold;  // minimal number of points to form a plane  
    CPlane* planeAt(int id);
    vector<CPlane*> v_plane_set_; // collection of planes 
    vector<vector<int> > mv_indices;  // collection of indices of the input point cloud 
};


#include "plane_set.hpp"

#endif 
