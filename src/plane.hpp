
#include <fstream>

template<typename PointT>
void toPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc)
{
  int N = pts.size();
  pc->points.resize(N); 
  pc->width = N;
  pc->height = 1;
  for(int i=0; i<N; i++)
  {
    PointT& pt = pc->points[i]; 
    CPointF3& p = pts[i]; 
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2]; 
  }
}
template<typename PointT>
void toPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc, pcl::PointIndices::Ptr& inliers)
{
  int N = inliers->indices.size();
  pc->points.resize(N); 
  pc->width = N;
  pc->height = 1;
  for(int i=0; i<N; i++)
  {
    PointT& pt = pc->points[i]; 
    CPointF3& p = pts[inliers->indices[i]]; 
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2]; 
  }
}

template<typename PointT>
void fromPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc)
{
  int N = pc->points.size(); 
  pts.resize(N); 
  for(int i=0; i<N; i++)
  {
    CPointF3& p = pts[i]; 
    PointT& pt = pc->points[i]; 
    p[0] = pt.x; p[1] = pt.y; p[2] = pt.z;
  } 
}

template<typename PointT>
void fromPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc, pcl::PointIndices::Ptr& inliers)
{
  int N = inliers->indices.size(); // pc->points.size(); 
  pts.resize(N); 
  for(int i=0; i<N; i++)
  {
    CPointF3& p = pts[i]; 
    PointT& pt = pc->points[inliers->indices[i]]; 
    p[0] = pt.x; p[1] = pt.y; p[2] = pt.z;
  }  
}

template<typename PointT>
void CPlane::weightedPlaneFitting(boost::shared_ptr<pcl::PointCloud<PointT> >& pc, Eigen::Matrix3d& cov, Eigen::Vector3d& nv)
{
  // 1. compute weights 
  double Huber_c = 1.2;  // Huber weight parameter 
  vector<double > w; 
  double Tukey_c = 1.; 
  double sum_w = weightPTS(pc, w, &Tukey_c, "Tukey"); 
  if(sum_w == 0)
  {
    sum_w = weightPTS(pc, w, &Huber_c, "Huber"); 
  }
  
  // 2. compute center 
  double tx = 0, ty = 0, tz = 0; 
  for(int i=0; i<w.size(); i++)
  {
    PointT& pt = pc->points[i]; 
    tx += w[i]*pt.x; 
    ty += w[i]*pt.y; 
    tz += w[i]*pt.z; 
  }
  
  cx_ = tx / sum_w; cy_ = ty / sum_w; cz_ = tz / sum_w ;

  // 3. compute covariance matrix 
  calWeightedPCCov(pc, w, sum_w, cov); 

  // 4. compute smallest eigen vector 
  computeSmallestEivenVector(cov, nv); 

  return ;
}

template<typename PointT>
double CPlane::weightPTS(boost::shared_ptr<pcl::PointCloud<PointT> >& in, std::vector<double>& w, void* parameter, string method)
{
  // TODO:
  w.resize(in->size()); 
  double sum_w = 0; 
  double* pk = (double*)(parameter);
  double k = *pk; 
  double dis; 
  int valid_n = 0; 
  for(int i=0; i<w.size(); i++)
  {
    PointT& pt = in->points[i]; 
    dis = sqrt(SQ(pt.x) + SQ(pt.y) + SQ(pt.z)); 

    if(method == "Huber")
    {
    // Huber weight 
    if(dis <= k)
      w[i] = 1; 
    else
      w[i] = k/dis; 
    }else if(method == "Tukey")
    {
    // Tukey
    if(dis <= k)
      w[i] = 0;
    else
    {
      w[i] = SQ(1-SQ(dis/k));
      valid_n ++; 
    }
    }
    sum_w += w[i];
  }
  
  if(method == "Tukey" && valid_n < 1000)
  {
    sum_w = 0;
  }

  return sum_w; 
}

template<typename PointT>
void CPlane::calWeightedPCCov(boost::shared_ptr<pcl::PointCloud<PointT> >& pc, vector<double>& wv, double sum_w, Eigen::Matrix3d& cov)
{
  double X = 0, Y = 0, Z= 0, XY =0, XZ=0, YZ=0;
  double X2 = 0, Y2 =0, Z2 =0;

  double x,y,z;
  double w; 
  for(int i=0; i<pc->size(); i++)
  {
    PointT& pt = pc->points[i]; 
    x = pt.x - cx_; 
    y = pt.y - cy_; 
    z = pt.z - cz_; 
    w = wv[i]; 
    X2 += SQ(x)*w; 
    Y2 += SQ(y)*w; 
    Z2 += SQ(z)*w; 
    XY += x*y*w; 
    XZ += x*z*w; 
    YZ += y*z*w;
  }
  double factor = 1./sum_w; 
  X2 *= factor; 
  Y2 *= factor; 
  Z2 *= factor; 
  XY *= factor; 
  XZ *= factor; 
  YZ *= factor; 

  cov(0,0) = X2;
  cov(1,0) = cov(0,1) = XY; 
  cov(2,0) = cov(0,2) = XZ;
  cov(1,1) = Y2;
  cov(2,1) = cov(1,2) = YZ;
  cov(2,2) = Z2;
  
  return; 
}

template<typename PointT>
bool calPCCorvariance(boost::shared_ptr<pcl::PointCloud<PointT> >& pc, Eigen::Matrix3d& cov)
{
  double X = 0, Y = 0, Z= 0, XY =0, XZ=0, YZ=0;
  double X2 = 0, Y2 =0, Z2 =0;

  double x,y,z;
  for(int i=0; i<pc->size(); i++)
  {
    PointT& pt = pc->points[i];
    x = pt.x; 
    y = pt.y;
    z = pt.z;
    X += x; 
    Y += y; 
    Z += z; 
    XY += x*y;
    XZ += x*z; 
    YZ += y*z;
    X2 += SQ(x);
    Y2 += SQ(y);
    Z2 += SQ(z);
  }
  double N = pc->size(); 
  N = 1./N;
  X *= N; // /= N;
  Y *= N; // /= N;
  Z *= N; // /= N;
  XY *=N; // /= N; 
  XZ *=N; // /= N;
  YZ *=N; // /= N;
  X2 *=N; // /= N;
  Y2 *=N; // /= N;
  Z2 *=N; // /= N; 

  // 
  float s = 1; 
  cov(0,0) = X2 - SQ(X); 
  cov(1,0) = cov(0,1) = s*(XY - X*Y); 
  cov(2,0) = cov(0,2) = s*(XZ - X*Z);
  cov(1,1) = Y2 - SQ(Y);
  cov(2,1) = cov(1,2) = s*(YZ - Y*Z);
  cov(2,2) = Z2 - SQ(Z);
  return true;
}

template<typename PointT>
void CPlane::computeParameters(boost::shared_ptr<pcl::PointCloud<PointT> >& pc)
{
  VectorPF3 pts; 
  fromPC<PointT>(pts, pc); 
  
  computeParameters(pts);
  return ;
}



template<typename PointT>
void CPlane::computeCOVDense(boost::shared_ptr<pcl::PointCloud<PointT> >& in)
{
  // compute variance-covariance matrix, and update plane parameters 
  Eigen::Matrix3d C; 
  Eigen::Vector3d nv; 

  calPCCorvariance(in, C); 
  computeSmallestEivenVector(C, nv); 
  computeCenter(in); 

  weightedPlaneFitting(in, C, nv);  // 

  Eigen::Vector3d mu; mu << cx_, cy_, cz_; 
  // update distance 
  if(nv(2) < 0) nv = nv * -1.; 
  // ROS_INFO("previous n= %f %f %f %f", nx_, ny_, nz_, d1_); 
  nx_ = nv(0);  ny_ = nv(1); nz_ = nv(2); 
  d1_ = -nv.transpose()*mu; 
  // ROS_INFO("after n= %f %f %f %f", nx_, ny_, nz_, d1_); 
  // cout<<"mu = "<<mu<<endl;

  // compute Jacobians
  int N = in->points.size(); 
  Eigen::MatrixXd J(4, 3*N); // Jacobian Matrix 
  // Eigen::MatrixXd PTS = Eigen::Matrix<double, 3, N>::Zero(); 
  Eigen::MatrixXd PTS(3, N); 
  for(int i=0; i<N; i++)
  {
    PointT& pt = in->points[i]; 
    PTS(0, i) = pt.x; PTS(1, i) = pt.y; PTS(2, i) = pt.z; 
  }
  // ROS_INFO("before compute J");

  Eigen::Matrix3d old_C = C; 
  // compute Ji 
  for(int i=0; i<N; i++)
  {
    Eigen::Vector3d xi = PTS.col(i); 
    // compute Ji
    Eigen::MatrixXd Ji(4, 3); 
    // ROS_INFO("before compute Ji at i=%d", i);
    computeJi(Ji, xi, PTS, old_C, N); 
    // cout <<"J"<<i<<" = "<<endl<<Ji<<endl; 
    // ROS_INFO("after compute Ji ");
    J.block(0,3*i,4,3) = Ji; 
  }
  // ROS_INFO("after compute J"); 

  // compute COV_[nv, d] = J*C[x,y,z]*J^t
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> C_xyz(3*N); 
  double Sxx, Syy, Szz; 
  Sxx = Syy = Szz = CamCov::getStaticSigma(); 
  for(int i=0; i<3*N; i++)
  {
    C_xyz.diagonal()(i) = Sxx; 
  }

  // ROS_INFO("after compute C_xyz"); 
  // cout <<"J = "<<endl<<J<<endl;
  // finally, compute the COV_[nv, d]
  // Eigen::Matrix<double, 4, 4> C_plane; 

  //  (*m_CP) = J*C_xyz*J.transpose(); 
  Eigen::Matrix4d M = J*C_xyz*J.transpose(); 
  saveCP(M); 

  // C_plane = J*C_xyz*J.transpose();
  // cout<<"C_plane: "<<endl<<C_plane<<endl;
  return ; 
}

template<typename PointT>
void CPlane::computeCOVSparse(cv::Mat& img, boost::shared_ptr<pcl::PointCloud<PointT> >& in, vector<int>& index, int s)
{
  // need to filter some points fall outside ant key point 
  // vector<int> v_kps = findKeyPointsInPlaneArae(img.rows, img.cols, index, s); 

  // prepare matrix for computing COVARIANCE 
  int N = in->points.size(); 
  m_npts = N; 

  // compute variance-covariance matrix, and update plane parameters 
  Eigen::Matrix3d C; 
  Eigen::Vector3d nv; 
  Eigen::Vector3d mu;
/*
  calPCCorvariance(in, C); 
  computeSmallestEivenVector(C, nv); 
  computeCenter(in); 
  mu << cx_, cy_, cz_; 
  // update distance 
  if(nv(2) < 0) nv = nv * -1.; 
  ROS_INFO("%s previous n= %f %f %f %f", __FILE__, nx_, ny_, nz_, d1_); 
  nx_ = nv(0);  ny_ = nv(1); nz_ = nv(2); 
  d1_ = -nv.transpose()*mu; 
  ROS_INFO("%s after n= %f %f %f %f", __FILE__, nx_, ny_, nz_, d1_); 
*/
  // recompute 
  ROS_INFO("%s previous n= %f %f %f %f", __FILE__, nx_, ny_, nz_, d1_); 
  weightedPlaneFitting(in, C, nv); 
  if(nv(2) < 0) nv = nv * -1.; 
  mu << cx_, cy_, cz_;  nx_ = nv(0);  ny_ = nv(1); nz_ = nv(2); 
  d1_ = -nv.transpose()*mu; 
  ROS_WARN("%s after recompute plane n= %f %f %f %f", __FILE__, nx_, ny_, nz_, d1_); 

  // compute Jacobians
  Eigen::MatrixXd J(4, 3*N); // Jacobian Matrix 
  // Eigen::MatrixXd PTS = Eigen::Matrix<double, 3, N>::Zero(); 
  Eigen::MatrixXd PTS(3, N); 
  for(int i=0; i<N; i++)
  {
    PointT& pt = in->points[i]; 
    PTS(0, i) = pt.x; PTS(1, i) = pt.y; PTS(2, i) = pt.z; 
  }
  
  double sum_dis_square = 0; 

  // compute Jacobians 
  map<int, vector<int> > PATCH; 
  map<int, int> PATCH_CENTRAL; 
  patchPointsInPlaneArea(img.rows, img.cols, index, s, PATCH, PATCH_CENTRAL); 

  Eigen::Matrix3d old_C = C; 
  map<int, vector<int> >::iterator it = PATCH.begin(); 
  mv_patch_area.reserve(PATCH.size()); 
  while(it != PATCH.end())
  {
    // compute Ji at central point
    int i = PATCH_CENTRAL[it->first]; // index of the 
    Eigen::Vector3d xi = PTS.block<3,1>(0,i); 
      
    // record patch center
    mv_patch_area.push_back(it->first); 

    // compute Ji
    Eigen::MatrixXd Ji(4, 3); 
    computeJi(Ji, xi, PTS, old_C, N); 

    // for all the rest points in the PATCH, equal to Ji
    for(int j=0; j<it->second.size(); j++)
    {
      int col = 3*it->second[j];
      J.block(0,col,4,3) = Ji;  // block<p,q>(i,j) = block(i, j, p, q); 
      
      double px = PTS(0,it->second[j]); 
      double py = PTS(1,it->second[j]); 
      double pz = PTS(2,it->second[j]); 
      double dis = dis2plane(px, py, pz); 
      sum_dis_square += dis*dis; 
    }

    ++it; 
  }

  // compute COV_[nv, d] = J*C[x,y,z]*J^t
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> C_xyz(3*N); 
  double Sxx, Syy, Szz; 
  Sxx = Syy = Szz = CamCov::getStaticSigma(); 
  for(int i=0; i<3*N; i++)
  {
    C_xyz.diagonal()(i) = Sxx; 
  }

  // finally, compute the COV_[nv, d]
  // Eigen::Matrix<double, 4, 4> C_plane; 
  // C_plane = J*C_xyz*J.transpose(); 
  // (*m_CP) = J*C_xyz*J.transpose(); 
  
  Eigen::Matrix4d M = J*C_xyz*J.transpose(); 

  /*cout <<"compute M: "<<endl<<M<<endl;
  Eigen::Matrix4d MM;
  MM <<  3.56733e-07,  1.11311e-08,   2.5493e-09,  -7.8651e-08,
     1.11311e-08,  4.37023e-09, -1.35016e-08,  2.02329e-08,
     2.5493e-09, -1.35016e-08,  4.58673e-08, -7.71521e-08,
     -7.8651e-08,  2.02329e-08, -7.71521e-08,  1.69723e-07;*/
  /* MM <<  M(0,0),  M(0,1),   M(0,2), M(0,3),
          M(1,0),  M(1,1),   M(1,2), M(1,3),
          M(2,0),  M(2,1),   M(2,2), M(2,3),
          M(3,0),  M(3,1),   M(3,2), M(3,3);*/
  // M = MM; 

  sum_dis_square /= N; 
  saveCP(M, sum_dis_square); 

  // record patch center  
  //for debug
  // cout <<__FILE__<<" after compute COV testEigen()"<<endl;
  // testEigen(); 

  // cout<<"C_plane: "<<endl<<C_plane<<endl;
  return; 

}

template<typename PointT>
void CPlane::computeCenter(boost::shared_ptr<pcl::PointCloud<PointT> >& in)
{
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
  int N = in->points.size(); 
  inliers->indices.resize(N); 
  for(int i=0; i<in->points.size(); i++)
    inliers->indices[i] = i; 
  computeCenter(in, inliers); 
  return ; 
}

template<typename PointT>
void CPlane::computeCenter(boost::shared_ptr<pcl::PointCloud<PointT> >& in, pcl::PointIndices::Ptr& inliers)
{
  double tx =0; double ty = 0; double tz = 0; 
  int N = inliers->indices.size(); 
  int cnt = 0;
  for(int i=0; i<inliers->indices.size(); i++)
  {
    PointT& pt = in->points[inliers->indices[i]]; 
    tx += pt.x; ty += pt.y; tz += pt.z; 
    ++cnt;
  }
  if(cnt>0) 
  {
    double s = 1./(double)(cnt); 
    cx_ = tx * s; 
    cy_ = ty * s; 
    cz_ = tz * s;
  }
}

// project a point cloud into an image plane 
template<typename PointT>
void CPlane::projectPC2Image(cv::Mat& img, boost::shared_ptr<pcl::PointCloud<PointT> >& in, CamModel& m, vector<int>& U)
{
  int N = in->points.size(); 
  U.reserve(N); 
  float u, v; 
  int index; 
  int nan_n = 0, z_n = 0, range_n = 0; 
  int iu, iv; 
  // ofstream ouf("rest_pt_map.log"); 
  for(int i=0; i<in->size(); i++)
  {
    PointT& pt = in->points[i]; 
    if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) // invalid point 
    {
      // ++nan_n; 
     continue;
    }
    if(pt.z <= 0) // behind the camera  
    {
      // ++z_n; 
      continue; 
    }
    m.convertXYZ2UV(pt.x, pt.y, pt.z, u, v); 
    if(u < 0 || u >= img.cols || v < 0 || v >= img.rows) // out of range 
    {
      // ouf<<u<<" "<<v<<" "<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
      // ++range_n; 
      continue; 
    }
    // index = ((int)(v+0.5))*img.cols + (int)(u+0.5);  
    // index = ((int)(v+0.5))*img.cols + (int)(u+0.5);  
    iv = (int)(v+0.5); 
    iu = (int)(u+0.5); 
    if(iv >= img.rows) iv--; 
    if(iu >= img.cols) iu--; 
    index = iv * img.cols + iu; 
    U.push_back(index); 
  }
  
  // cout <<" nan_n : "<<nan_n<<" z_n: "<<z_n <<" range_n: "<<range_n<<" index.size = "<<U.size()<<endl;

  return; 
}

template<typename PointT>
void CPlane::computePCL(boost::shared_ptr<pcl::PointCloud<PointT> >& in, pcl::PointIndices::Ptr& inliers, double dis_threshold )
{
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);    
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(dis_threshold); // 0.0036 0.049 0.01 0.0025
  
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setInputCloud(in);
  seg.segment(*inliers, *coefficients);
  
  nx_ = coefficients->values[0];
  ny_ = coefficients->values[1];
  nz_ = coefficients->values[2];
  d1_ = coefficients->values[3];

  return ;
}

