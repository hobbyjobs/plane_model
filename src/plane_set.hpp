#include <pcl/filters/extract_indices.h>



template<typename PointT>
int CPlaneSet::extractPlanes(boost::shared_ptr<pcl::PointCloud<PointT> >& in, CamModel& cam, double squared_dis_threshold)
{
  // first clear planes 
  // clearPlanes(); 

  // bool first_plane = true; 
  // cv::Mat tmp = cv::Mat(in->height, in->width, CV_8UC1); 
  cv::Mat tmp = cv::Mat(cam.m_rows, cam.m_cols, CV_8UC1); 
  return extractPlanes(tmp, in, cam, squared_dis_threshold); 
}

template<typename PointT>
int CPlaneSet::extractPlanes(boost::shared_ptr<pcl::PointCloud<PointT> >& in, double squared_dis_threshold)
{
   // first clear planes 
  clearPlanes(); 

  int ret = 0;
  int M = in->points.size(); 
  int i_thresh_number = M * d_percent_threshold_; 
  i_thresh_number = i_thresh_number > m_num_of_pts_threshold ? i_thresh_number : m_num_of_pts_threshold; 
  
  boost::shared_ptr<pcl::PointCloud<PointT> > in_copy = in->makeShared(); 

  do
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
    CPlane* p = new CPlane(); 
    p->computePCL(in_copy, inliers, squared_dis_threshold); 
    if(inliers->indices.size() < i_thresh_number)
      break; 
    int cur_number = in_copy->points.size(); 
    int inlier_number = inliers->indices.size(); 
    p->computeCenter(in_copy, inliers); 
    v_plane_set_.push_back(p); 
    ++ret; 
    if(cur_number - inlier_number < i_thresh_number) 
      break; 
    
    boost::shared_ptr<pcl::PointCloud<PointT> > tmpPC(new pcl::PointCloud<PointT>); 
    extractOutlier(in_copy, tmpPC, inliers); 

    in_copy.swap(tmpPC); 
  }while(1); 

  return ret; 
 
}

template<typename PointT>
int CPlaneSet::extractPlanes(cv::Mat& img, boost::shared_ptr<pcl::PointCloud<PointT> > &in, CamModel& cam, double squared_dis_threshold)
{
  // first clear planes 
  clearPlanes(); 

  int ret = 0;
  int M = in->points.size(); 
  int i_thresh_number = M * d_percent_threshold_; 
  i_thresh_number = i_thresh_number > m_num_of_pts_threshold ? i_thresh_number : m_num_of_pts_threshold; 

  int s = 4; //4; // TODO: this should be parameterized 

  boost::shared_ptr<pcl::PointCloud<PointT> > in_copy = in->makeShared();
  do
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    CPlane * p = new CPlane(); 
    p->computePCL(in_copy, inliers, squared_dis_threshold); 
    if(inliers->indices.size() < i_thresh_number)
    {
      break;  // number of the points must large than i_thresh_number 
    }
    
    int cur_number = in_copy->points.size();
    int inlier_number  = inliers->indices.size();
    p->computeCenter(in_copy, inliers); 
    // p->inliers_->indices = inliers->indices;
    v_plane_set_.push_back(p); 
    CloudPtr inlierPC(new Cloud); 
    extractInlier(in_copy, inlierPC, inliers); 
    vector<int> inlier_index; 

    {
      // mv_indices.push_back(inliers->indices);
      // inlier_index = inliers->indices; 
      p->projectPC2Image(img, inlierPC, cam, inlier_index); 
      mv_indices.push_back(inlier_index); 
      // cout <<" inlier_index has points: "<<inlier_index.size()<<" inlierPC has points: "<<inlierPC->points.size()<<endl; 
    }   

    // for debug 
   /* Eigen::Matrix<double, 4, 4 > C; 
    C<< 3.56733e-07,  1.11311e-08,   2.5493e-09,  -7.8651e-08,
     1.11311e-08,  4.37023e-09, -1.35016e-08,  2.02329e-08,
     2.5493e-09, -1.35016e-08,  4.58673e-08, -7.71521e-08,
     -7.8651e-08,  2.02329e-08, -7.71521e-08,  1.69723e-07; 
    C << 
 9.62378e-08,  7.25163e-09, -7.39674e-09,  1.28696e-08,
 7.25163e-09,  4.15398e-09, -1.07215e-08,  1.98561e-08,
-7.39674e-09, -1.07215e-08,  2.92056e-08, -5.42007e-08,
 1.28696e-08,  1.98561e-08, -5.42007e-08,  1.13758e-07;

    p->saveCP(C);
    p->testEigen(); 
*/ 
    // compute Plane's parameter
    p->computeCOVSparse(img, inlierPC, inlier_index, s); 
    
    ++ret; 
    if(cur_number - inlier_number < i_thresh_number)
    {
      break;  // number of the rest points is less than i_thresh_number
    }
    
    // use the rest points 
    boost::shared_ptr<pcl::PointCloud<PointT> > tmpPC(new pcl::PointCloud<PointT>);
    extractOutlier(in_copy, tmpPC, inliers);
    
    in_copy.swap(tmpPC);

  }while(1);

  return ret;
}

template<typename PointT>
void CPlaneSet::extractOutlier(boost::shared_ptr<pcl::PointCloud<PointT> >& in, 
    boost::shared_ptr<pcl::PointCloud<PointT> >& out, pcl::PointIndices::Ptr& inliers)
{
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (in);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*out);
  return ;
}

template<typename PointT>
void CPlaneSet::extractInlier(boost::shared_ptr<pcl::PointCloud<PointT> >& in, 
    boost::shared_ptr<pcl::PointCloud<PointT> >& out, pcl::PointIndices::Ptr& inliers)
{
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (in);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*out);
  return ;
}


