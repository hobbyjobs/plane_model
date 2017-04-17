/*
 *  Sep 14 2015, David Z 
 *  
 *  to represent a plane collection
 *
 * */


#ifndef PLANE_H
#define PLANE_H

#include <vector>

// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
#include "glob_def.h"
#include "point_xyz.h"
#include "cam_model.h"
#include "cam_cov.h"
#include <limits>
#include <vector>
#include <cassert>
#include <set>
#include <string>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <boost/dynamic_bitset.hpp>

using namespace std;

// class CSNormal; 

void transformPC(VectorPF3& out, VectorPF3& in, tf::Transform tr);

extern bool computeEigenVector(Eigen::Matrix3d& covariance_matrix, Eigen::Vector3d& major_axis, Eigen::Vector3d& middle_axis,
    Eigen::Vector3d& minor_axis, float& major_value, float& middle_value, float& minor_value);
extern bool computeSmallestEivenVector(Eigen::Matrix3d& cov, Eigen::Vector3d& minor_axis);

// not a good idea, cannot work if the keypoint is not in the plane but the other points are in the plane 
extern vector<int> findKeyPointsInPlaneArae(int H, int W, vector<int>& index, int s);  // find the key points in the plane area index & (H,W)%s

// group points fall into a patch
extern int patchPointsInPlaneArea(int H, int W, vector<int>& index, int s, map<int, vector<int> >& PATCH, map<int, int> & PATCH_CENTER); 

class CPlane
{
  public:
    CPlane();
    ~CPlane(); 
    CPlane(CPointF3 nv, float d = 0);
    
  public:
    // vector<CPointF3> genRanom(int N, int BX = 10, int BY = 10, int U=100000); // generate random points on this plane within [BX, BY]
    // vector<CPointF3> genGauss_dis( vector<CPointF3>& ,  float sigma);  // generate points on a plane with d ~ N(0, sigma^2)
    // void setGaussSigma(float s);   // set gauss sigma 
    // double gaussValue();                            // output a random value satisfying gauss distribution 

    void print(std::string str="");     // print its parameters
    double pitch();                                 // ouput pitch angle given nx, ny, nz
    bool ptOnPlane(double px, double py, double pz, double& plx, double& ply, double & plz);                       // project pt to the plane 
    double dis2plane(double px, double py, double pz); 
    float dis2plane(CPointF3);                      // compute the distance between a point to plane 
    float computeZ(float x, float z);               // use plane function to compute Z value 
    void computeParameters(vector<CPointF3>& pts);  // copy point cloud and computePCL(); 
    
    template<typename PointT>
    void computeParameters(boost::shared_ptr<pcl::PointCloud<PointT> >&); 

    void computePCL();            // compute the plane parameters using PCL function
    template<typename PointT>
    void computePCL(boost::shared_ptr<pcl::PointCloud<PointT> >& in, pcl::PointIndices::Ptr& inliers, double dis_threshold = 0.005);   // compute the plane parameters using PCL function, with point cloud in 
    double computeSigma();        // compute the distance sigma of the plane
    void keepInliers();           // only maintain the inliers, delete points not on the plane
    void refineParameters();      // try to refine plane's parameters by casting out more noise data

    CPointF3 projOnPlane(CPointF3 p);                   // project a point into plane 
    vector<CPointF3> projOnPlane(vector<CPointF3> pts); // project these points into plane

    // project a point cloud into an image plane 
    template<typename PointT>
    void projectPC2Image(cv::Mat& img, boost::shared_ptr<pcl::PointCloud<PointT> >& in, CamModel& m, vector<int>& ); 
    
    // compute a plane's parameters: n and d + their COVARIANCE
    template<typename PointT>
    void computeCOVSparse(cv::Mat& img, boost::shared_ptr<pcl::PointCloud<PointT> >& in, vector<int>&, int s);

    template<typename PointT>
    void computeCOVDense(boost::shared_ptr<pcl::PointCloud<PointT> >& in);

    // update the covariance given a small updation 
    void updateCORdelta(Eigen::Matrix3d& ori_C, Eigen::Matrix3d& new_C, int N, Eigen::Matrix<double, 3,1>& xi, Eigen::Matrix<double, 3,1>& epsilon, Eigen::MatrixXd& ); 
  
    // compute Ji at xi 
    void computeJi(Eigen::MatrixXd& Ji, Eigen::Vector3d& xi, Eigen::MatrixXd& PTS, Eigen::Matrix3d& C, int N); 

  public:
    // boost::shared_ptr<CSNormal> pNormal_;     // gauss seed 

  public:
    vector<CPointF3> pts_;
    pcl::PointIndices::Ptr inliers_;  // indicate which point belongs to the plane

    template<typename PointT>
    void computeCenter(boost::shared_ptr<pcl::PointCloud<PointT> >& in, pcl::PointIndices::Ptr& inliers);   // compute the plane parameters using PCL function, with point cloud in 

    template<typename PointT>
    void computeCenter(boost::shared_ptr<pcl::PointCloud<PointT> >& in); 

    template<typename PointT>
    void weightedPlaneFitting(boost::shared_ptr<pcl::PointCloud<PointT> >& in,Eigen::Matrix<double, 3,3>& C, Eigen::Matrix<double, 3, 1>& nv); 

    template<typename PointT>
    double weightPTS(boost::shared_ptr<pcl::PointCloud<PointT> >& in, std::vector<double>& w, void* parameter, std::string method);

    template<typename PointT>
    void calWeightedPCCov(boost::shared_ptr<pcl::PointCloud<PointT> >& in, std::vector<double>& w, double sum_w, Eigen::Matrix<double, 3, 3> & C); 

    // center point 
    double cx_; 
    double cy_; 
    double cz_;

    // different models 
    // // model 1: nx, ny, nz, d, <nx, p> - d = 0
    double nx_; 
    double ny_; 
    double nz_; 
    double d1_;
    void print_m1();

    void loadParameters(vector<float>& p);
    void saveParameters(vector<float>& p);

    // model 2: theta, phi, d, x*cos(theta)*cos(phi) + y*cos(theta)*sin(phi) + z*sin(theta) + d2 = 0
    float theta_; 
    float phi_; 
    float d2_;
    
    // model 3: a, b, d Z = aX + bY + d
    // model 4: a, b, c aX + bY + cZ + 1 = 0
    // Eigen::Matrix4d* m_CP; // covariance of the detected plane  
    
    double m_E_Sdi; // estimate S_di
    double m_CP[4][4]; // store the raw data 
    double getTraceSVN(){return (m_CP[0][0] + m_CP[1][1] + m_CP[2][2]); }
    void saveCP(Eigen::Matrix4d& m, double estimated_sigma_dis=-1); 
    void regularizeCOV(); 

    void getNVCov(Eigen::Matrix3d& C); 
    
    vector<int> mv_patch_area; // record the reached patch center 
    bool overlapped(CPlane*);  // check whther this two planes overlapped 
    bool merge(CPlane*);       // merge with another plane
    double dotProduct(CPlane*); 
    
    int m_npts; // record how many points this plane has 
    inline void normalize(); 
    void testEigen(); 

  // public:
   // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#include "plane.hpp"

#endif 
