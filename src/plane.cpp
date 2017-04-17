#include "plane.h"
#include <stdlib.h>
// #include "normal.h"
#include <iostream>
#include <algorithm>
#include "mean_sigma.h"

namespace{
  // find consistent 
  void flipNV(Eigen::Vector3d& nv, Eigen::Vector3d& u)
  {
    // cos(OU, NU)
    if(nv.transpose()*u < 0)
    {
      nv = -nv; 
    }
  }
}

template<>
void toPC(vector<CPointF3>& pts, CloudPtr& pc)
{
  int N = pts.size();
  pc->points.resize(N); 
  pc->width = N;
  pc->height = 1;
  for(int i=0; i<N; i++)
  {
    Point& pt = pc->points[i]; 
    CPointF3& p = pts[i]; 
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2]; 
  }
}

template<>
void toPC(vector<CPointF3>& pts, CloudPtr& pc, pcl::PointIndices::Ptr& inliers)
{
  int N = inliers->indices.size();
  pc->points.resize(N); 
  pc->width = N;
  pc->height = 1;
  for(int i=0; i<N; i++)
  {
    Point& pt = pc->points[i]; 
    CPointF3& p = pts[inliers->indices[i]]; 
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2]; 
  } 
}

template<>
void fromPC(vector<CPointF3>& pts, CloudPtr& pc)
{
  int N = pc->points.size(); 
  pts.resize(N); 
  for(int i=0; i<N; i++)
  {
    CPointF3& p = pts[i]; 
    Point& pt = pc->points[i]; 
    p[0] = pt.x; p[1] = pt.y; p[2] = pt.z;
  }
}

template<>
void fromPC(vector<CPointF3>& pts, CloudPtr& pc, pcl::PointIndices::Ptr& inliers)
{
  int N = inliers->indices.size(); 
  pts.resize(N); 
  for(int i=0; i<N; i++)
  {
    CPointF3& p = pts[i]; 
    Point& pt = pc->points[inliers->indices[i]]; 
    p[0] = pt.x; p[1] = pt.y; p[2] = pt.z;
  }
}


void transformPC(VectorPF3& out, VectorPF3& in, tf::Transform tr)
{
  int N = in.size();
  out.resize(N);
  for(int i=0; i<N; i++)
  {
    CPointF3 & p = in[i];
    tf::Vector3 from(p[0], p[1], p[2]); 
    tf::Vector3 to = tr*from; 
    out[i] = CPointF3(to.x(), to.y(), to.z());
  }
}



CPointF3::CPointF3()
{
}

CPointF3::CPointF3(float x, float y, float z)
{
  _p[0] = x; _p[1] = y; _p[2] = z;
}

CPointF3::CPointF3(float v)
{
  _p[0] = _p[1] = _p[2] = v;
}

bool CPointF3::isValid()
{
  return (!IS_INF(_p[0]) && !IS_INF(_p[1]) && !IS_INF(_p[2]));
}

////////////////////////////////////////////////////////////

CPlane::CPlane(): 
// pNormal_(new CSNormal()),
inliers_(new pcl::PointIndices),
  nx_(0), 
  ny_(0),
  nz_(1),
  d1_(0)
 {
  // m_CP = new Eigen::Matrix4d; 
 }
CPlane::~CPlane()
{
  // if(m_CP != 0) 
  //  delete m_CP; 
}

CPlane::CPlane(CPointF3 nv, float d):
// pNormal_(new CSNormal()),
inliers_(new pcl::PointIndices)
{
  nx_ = nv[0]; ny_ = nv[1]; nz_ = nv[2]; 
  d1_ = d;
  // m_CP = new Eigen::Matrix4d; 
}



//TODO compute m2 parameters from m1  
// void CPlane::m1_2_m2() //
// {}
//
//

vector<CPointF3> CPlane::projOnPlane(vector<CPointF3> pts)
{
  int N = pts.size();
  vector<CPointF3> ret(N); 
  for(int i=0; i<N; i++)
    ret[i] = projOnPlane(pts[i]);
  return ret;
}

CPointF3 CPlane::projOnPlane(CPointF3 p)
{
  float d = p[0]*nx_ + p[1]*ny_ + p[2]*nz_ + d1_;
  if(fabs(d) <= 1e-6)
  {
    return p;
  }
  CPointF3 ret;
  if(d >= 0) 
  {
    double l = d ; //sqrt(d); 
    ret[0] = p[0] - l*nx_; ret[1] = p[1] - l*ny_; ret[2] = p[2] - l*nz_;
  }else
  {
    double l = -1.*d; //sqrt(-1.*d); 
    ret[0] = p[0] + l*nx_; ret[1] = p[1] + l*ny_; ret[2] = p[2] + l*nz_;
  }
  return ret;
}

void CPlane::getNVCov(Eigen::Matrix3d& C)
{
  // C = (*m_CP).block<3,3>(0,0);
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      C(i,j) = m_CP[i][j]; 
}

double CPlane::dis2plane(double px, double py, double pz)
{
  double ret = nx_*px + ny_ * py + nz_*pz + d1_; 
  return ret; 
}

float CPlane::dis2plane(CPointF3 p)
{
  float ret = nx_*p[0] + ny_*p[1] + nz_*p[2] + d1_; 
  // cout<<p<<" nx_: "<<nx_<<" ny_: "<<ny_<<" nz: "<<nz_<< " d: "<<d1_<<endl; 
  // cout<< "dot: "<< nx_*p[0] + ny_*p[1] + nz_*p[2]<<endl;
  return fabs(ret);
}

float CPlane::computeZ(float x, float y)
{
  float z = d1_; 
  assert(nz_ != 0); 
  z = (-nx_*x - ny_*y - d1_) / nz_;
  return z;
}

void CPlane::print_m1()
{
  cout<<"model_1: nv: "<<nx_<<", "<<ny_<<", "<<nz_<<", d: "<<d1_<<" pitch: "<<R2D(pitch())<<endl;
}


/*
vector<CPointF3>  
CPlane::genRanom(int N, int BX, int BY, int U) // generate random points on this plane 
{
  srand((unsigned int)(NULL));
  vector<CPointF3> ret(N);
  float ux = (float)(BX)/(float)(U); 
  float uy = (float)(BY)/(float)(U);
  float px, py, pz;
  for(int i=0; i<N; i++) 
  {
    px = (rand()%U)*ux; 
    py = (rand()%U)*uy;
    pz = computeZ(px, py);
    ret[i] = CPointF3(px, py, pz);

    // cout<<px<<" "<<py<<" "<<pz<<" dis: "<<dis2plane(ret[i])<<endl;
  }
  return ret;
}

void CPlane::setGaussSigma(float sigma)
{
  if(pNormal_->sigma_ != sigma)
    pNormal_->setSigma(sigma);
}

double CPlane::gaussValue()
{
  return pNormal_->normal(); 
}

vector<CPointF3> CPlane::genGauss_dis(vector<CPointF3>& pts, float sigma)
{
  setGaussSigma(sigma);
  int N = pts.size();
  vector<CPointF3> ret(N); 
  double lambda; 
  float px, py,pz;
  for(int i=0; i<N; i++)
  {
    CPointF3 & p = pts[i]; 
    lambda = pNormal_->normal(); 
    if(lambda < 0)
    {
      // lambda = -sqrt(-1.*lambda);
      lambda = -1.*lambda;
    }else
    {
      // lambda = sqrt(lambda);
      lambda = lambda;
    }
    px = p[0] + lambda * nx_; 
    py = p[1] + lambda * ny_; 
    pz = p[2] + lambda * nz_;
    ret[i] = CPointF3(px, py, pz);
  }
  return ret;
}
*/
double CPlane::pitch()
{
  double theta ; 
  if(ny_ < 0)
  {
    theta = atan2(-nz_, -ny_); 
  }
  else{
    theta = atan2(nz_, ny_); 
  }
  return theta;
}

void CPlane::computeParameters(vector<CPointF3>& pts)
{
  pts_ = pts; 
  computePCL();
}

void CPlane::saveParameters(vector<float>& p)
{
  p.resize(4); 
  p[0] = nx_; p[1] = ny_; p[2] = nz_; p[3] = d1_;
  
  // TODO: add theta  
}

void CPlane::loadParameters(vector<float>& p)
{
  nx_ = p[0]; ny_ = p[1]; nz_ = p[2]; d1_ = p[3];
  
  // TODO: add theta
}

/*
void CPlane::computePCL(CloudPtr& in)
{
  // Create the segmentation object
  pcl::SACSegmentation<Point> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);    
  seg.setMethodType(pcl::SAC_RANSAC);
  // seg.setDistanceThreshold(0.5);
  seg.setDistanceThreshold(0.01); // 0.0025

  // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // int N = in->points.size();
  // inliers_->indices.resize(N);
  // for(int i=0; i<N; i++) 
  //  inliers_->indices[i] = i;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setInputCloud(in);
  seg.segment(*inliers_, *coefficients);
  
  nx_ = coefficients->values[0];
  ny_ = coefficients->values[1];
  nz_ = coefficients->values[2];
  d1_ = - coefficients->values[3];
  return ;
}*/

void CPlane::computePCL()
{
  // 
  CloudPtr in(new Cloud);
  toPC(pts_, in);
  
  // cout<<"plane.cpp: fit plane parameters: nv: "<<nx_<<" "<<ny_<<" "<<nz_<<" d: "<<d1_<<endl;
  
  computePCL(in, inliers_);

  return ;
}

void CPlane::keepInliers()
{
  int N = inliers_->indices.size(); 
  if(N <=0 || pts_.size() <=0 ) return; 
  VectorPF3 tmp(N); 
  // cout<<"keepInliers() N = "<<N<<endl;
  // ofstream ouf("tmp2.log");
  for(int i=0; i<N; i++)
  {
    tmp[i] = pts_[inliers_->indices[i]];
    // ouf<<inliers_->indices[i]<<" "<<tmp[i]<<" "<<tmp[i].dis_()<<endl;
  }
  // dumpPC2File(tmp, "tmp.log");
  pts_.swap(tmp);
}

double CPlane::computeSigma()
{
  int M = inliers_->indices.size(); 
  vector<double> dis(M); 
  for(int i=0; i<M; i++)
  {
    CPointF3& pt = pts_[inliers_->indices[i]]; 
    // distance of a point to this plane 
    dis[i] = fabs(pt[0]*nx_ + pt[1]*ny_ + pt[2]*nz_ + d1_);
  }
  
  double mean, sigma ; 
  compute_mu_sigma(&dis[0], M, mean, sigma);
  ROS_INFO("plane.cpp: dis mean %f and sigma %f", mean, sigma);
  
  return sigma;
}

void CPlane::refineParameters()
{
  //
  keepInliers();  // delete outliers 
  VectorPF3 ori_pts = copyPercent(pts_, 0.5); 
  CloudPtr ori_plane(new Cloud);
  toPC(ori_pts, ori_plane);

  // for test 
  Eigen::Matrix3d cov; 
  calPCCorvariance<Point>(ori_plane, cov); 
  Eigen::Vector3d nv; 
  computeSmallestEivenVector(cov, nv); 

  print_m1();

  nx_ = nv(0); ny_ = nv(1); nz_ = nv(2);
  cout<<"plane.cpp: using PCA nv: "<<nv(0)<<" "<<nv(1)<<" "<<nv(2)<<endl;

  print_m1(); 

  pts_ = ori_pts; 
  computePCL(); 
  cout<<"plane.cpp: using PCL compute: "<<endl;
  print_m1();

  // 
  return ;
}

    // compute Ji at xi 
void CPlane::computeJi(Eigen::MatrixXd& Ji, Eigen::Vector3d& xi, Eigen::MatrixXd& PTS, Eigen::Matrix3d& C, int N)
{
  double e = 1e-3;
  double de = 1./(2*e);
  Eigen::Matrix3d new_C; 
  Eigen::Matrix3d epsilon_m = Eigen::Matrix3d::Identity() * e; 
  
  // positive and negative  
  Eigen::Vector3d nv_p, nv_n, mu_p, mu_n; 
  double d_p, d_n;   
  Eigen::Vector3d mu; mu << cx_, cy_, cz_; 
  Eigen::Vector3d nv; nv << nx_, ny_, nz_;

  for(int i = 0; i<=2; i++) // x, y, z
  {
    // f(x + dx)
    Eigen::Vector3d epsilon = epsilon_m.col(i); 
    // ROS_INFO("before update new_C postive"); 
    updateCORdelta(C, new_C, N, xi, epsilon, PTS); 
    computeSmallestEivenVector(new_C, nv_p); 
    mu_p = mu + epsilon/N; 
    // cout<<"nv_p: "<<nv_p<<" mu: "<<mu<<" cos(nv_p, mu) = "<<nv_p.transpose()*mu<<endl;
    flipNV(nv_p, nv); 
    d_p = - nv_p.transpose()*mu_p; 
    // cout<<"at i= "<<i<<" pos+ new_C: "<<endl<<new_C<<endl<<" nv_p: "<<nv_p<<endl;

    // f(x - dx)
    epsilon = -1*epsilon; 
    // ROS_INFO("before update new_C negative"); 
    updateCORdelta(C, new_C, N, xi, epsilon, PTS); 
    computeSmallestEivenVector(new_C, nv_n); 
    mu_n = mu + epsilon/N; 
    // flipNV(nv_n, nv); 
    flipNV(nv_n, nv_p);
    // cout<<"at i= "<<i<<" neg- new_C: "<<endl<<new_C<<endl<<" nv_n: "<<nv_n<<endl;
    d_n = -nv_n.transpose()*mu_n;

    // cout <<"at i = "<<i<<" nv: "<<nv<<" nv_p: "<<nv_p<<" nv_n: "<<nv_n<<endl;

    // ROS_INFO("before update Ji.col(j) with j = %d", i); 
    Ji.block( 0, i, 3, 1) =  (nv_p - nv_n)*de; 
    Ji(3,i) = (d_p - d_n) * de; 
    // cout<<"J"<<i<<" = "<<endl<<Ji.col(i)<<endl;
  }
  return ; 
}

 // update the covariance given a small updation 
void CPlane::updateCORdelta(Eigen::Matrix3d& ori_C, Eigen::Matrix3d& new_C, int N, Eigen::Matrix<double, 3,1>& xi, Eigen::Matrix<double, 3,1>& epsilon, Eigen::MatrixXd& PTS)
{
  Eigen::Matrix<double, 3,1> d_mu = epsilon/N; 
  
  // compute new variance-covariance matrix given delta_epsilon 
  Eigen::MatrixXd d_mu_M = -d_mu.replicate(1, N); 
  new_C = Eigen::Matrix3d::Zero(); 
  new_C = d_mu_M * PTS.transpose() + PTS * d_mu_M.transpose() + N * d_mu * d_mu.transpose() - 2 * d_mu * epsilon.transpose(); 
  new_C = new_C + xi*epsilon.transpose() + epsilon*xi.transpose() + epsilon*epsilon.transpose();
  new_C = new_C/N + ori_C; 
  return ; 
}

double CPlane::dotProduct(CPlane* p)
{
  return (nx_*p->nx_ + ny_*p->ny_ + nz_*p->nz_); 
}

bool CPlane::ptOnPlane(double px, double py, double pz, double& plx, double& ply, double& plz)
{
  double a = nx_ * px + ny_ * py + nz_*pz; 
  if(fabs(a) < 1e-7) return false; 
  double k = -d1_/a; 

  plx = k*px; 
  ply = k*py;
  plz = k*pz; 
  return true; 
}

void CPlane::normalize()
{
  // normalize ? 
  double norm = sqrt(nx_*nx_ + ny_*ny_ + nz_*nz_); 
  nx_ /= norm; ny_ /= norm; nz_ /= norm; 
}

void CPlane::print(string str)
{
  cout << str<<endl; 
  cout <<"normal vector: "<<nx_<<" "<<ny_<<" " <<nz_<<" dis: "<<d1_<<endl;
  cout <<"covariance of plane: "<<endl;
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)
    {
      cout << m_CP[i][j]<<" ";
    }
    cout <<endl;
  }
}

void CPlane::regularizeCOV()
{
    // diagonalization 
    for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
    {
      if(i == j) 
      {
        m_CP[i][j] = fabs(m_CP[i][j]) + 1e-7;
      }
      else
        m_CP[i][j] = 0; //  (float)((int)(m(i,j)*1e12))*1e-12; 
    }
}

void CPlane::saveCP(Eigen::Matrix4d& m, double estimated_sigma_dis)
{
  for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
    {
      // it's a bug, (int)1000,000,000,0 < 0, will change its signs 
      // m_CP[i][j] = (float)((int)(m(i,j)*1e10))*1e-10; 
      m_CP[i][j] = m(i,j);
    }

  
  // for debug 
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > M(m_CP[0]); 
  // cout<<"initial: M "<<endl<<M<<endl;

  Eigen::Matrix<double, 4, 4> MINV = M.inverse(); 
  if(MINV(0,0) != MINV(0,0)) // nan
  {
    // cout<<"MINV has nan: "<<endl<<MINV<<endl;
    regularizeCOV();
  }else{
    if(M(0,0) < 0 || M(1, 1) < 0 || M(2, 2)  < 0 || M(3,3) < 0 || 
        MINV(0,0) < 0 || MINV(1,1) < 0 || MINV(2,2) < 0 || MINV(3,3) < 0)
    {
      regularizeCOV(); 
    }else
    {
      // Eigen::Matrix<double, 4, 4> T = M * MINV; 
      Eigen::Matrix<double, 3 ,3> T = M.block<3,3>(0,0); 
      // Eigen::Matrix4d I = Eigen::Matrix4d::Identity(); 

      // if(!MatrixEqual(T, I, 1e-3))
      if(!MatrixCheck(T))
      {
        regularizeCOV(); 
      }
    }
  }
  if(m_CP[3][3] > 10) // impossible case 
  {
      // diagonalization 
    for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
    {
      if(i == j) 
      {
        m_CP[i][j] = 0.014*0.014;
      }
      else
        m_CP[i][j] = 0; //  (float)((int)(m(i,j)*1e12))*1e-12; 
    }
  }

  for(int i=0;i<=3; i++)
  {
    // m_CP[i][i] += 0.014*0.014; // camera's depth covariance 
      // m_CP[3][i] = 0;
  }

  if(estimated_sigma_dis > 0)
  {
    // m_CP[3][3] = estimated_sigma_dis; 
    cout <<__FILE__<<" set estimated S_dis = "<<estimated_sigma_dis <<" m_CP[3][3] = "<<m_CP[3][3]<<endl;
    /*
    double k = estimated_sigma_dis / m_CP[3][3]; 
      // diagonalization 
    for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
    {
        m_CP[i][j] = k*m_CP[i][j]; //  (float)((int)(m(i,j)*1e12))*1e-12; 
    }
    cout <<__FILE__<<" k= "<<k <<" m_CP[3][3] = "<<m_CP[3][3]<<endl;
    */
    m_CP[3][3] = estimated_sigma_dis;
  }
  m_E_Sdi = estimated_sigma_dis; 
  // cout <<"at the end M: "<<endl<<M<<endl;
}

void CPlane::testEigen()
{
  // Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor > > C(m_CP[0]); 
   /*(*m_CP)<< 3.56733e-07,  1.11311e-08,   2.5493e-09,  -7.8651e-08,
     1.11311e-08,  4.37023e-09, -1.35016e-08,  2.02329e-08,
     2.5493e-09, -1.35016e-08,  4.58673e-08, -7.71521e-08,
     -7.8651e-08,  2.02329e-08, -7.71521e-08,  1.69723e-07; */

  Eigen::Matrix4d C; 
    C << m_CP[0][0], m_CP[0][1], m_CP[0][2], m_CP[0][3], 
         m_CP[1][0], m_CP[1][1], m_CP[1][2], m_CP[1][3], 
         m_CP[2][0], m_CP[2][1], m_CP[2][2], m_CP[2][3], 
         m_CP[3][0], m_CP[3][1], m_CP[3][2], m_CP[3][3]; 

    Eigen::Matrix<double, 3, 3> C1 = C.block<3,3>(0,0); 
    
    // C1 << 3.56733e-07,  1.11311e-08,   2.5493e-09,
    //      1.11311e-08,  4.37023e-09, -1.35016e-08,
    //       2.5493e-09,  -1.35016e-08,  4.58673e-08;
    Eigen::Matrix<double, 3, 3> CN = C1.inverse(); 
    Eigen::Matrix<double, 3, 3> T = C1*CN ; 
    // cout << fixed<<setprecision(15)<<"T1: "<<endl<<C1<<endl;
    cout <<"T1: "<<endl<<C1; 
    cout <<"T1N: "<<endl<<CN<<endl;
    cout <<"T1*T1N: "<<endl<<T<<endl;
  
}

bool CPlane::merge(CPlane* p)
{
  double cosA = dotProduct(p); 
  double nx = p->nx_; 
  double ny = p->ny_; 
  double nz = p->nz_; 
  double d = p->d1_; 
  double w1 = (double)(m_npts)/(double)(m_npts + p->m_npts); 
  double w2 = 1-w1; 
  if(cosA < 0)
  {
    nx = -nx; ny = -ny; nz = -nz; d = -d; 
  }  
  
  // Eigen::Matrix<double, 4, 4 > C; 
  /* m_CP<< 3.56733e-07,  1.11311e-08,   2.5493e-09,  -7.8651e-08,
     1.11311e-08,  4.37023e-09, -1.35016e-08,  2.02329e-08,
     2.5493e-09, -1.35016e-08,  4.58673e-08, -7.71521e-08,
     -7.8651e-08,  2.02329e-08, -7.71521e-08,  1.69723e-07; */

  Eigen::Vector4d u1, u2, u3; 
  u1 << nx_, ny_, nz_, d1_; 
  u2 << nx, ny, nz, d; 
  // nx_ = w1 * nx_ + w2 * nx; 
  // ny_ = w1 * ny_ + w2 * ny; 
  // nz_ = w1 * nz_ + w2 * nz; 
  // d1_ = w1 * d1_ + w2 * d ;

  // cout <<"before m_CP: "<<endl<<*m_CP<<endl;
  // cout <<"p->m_CP: "<<endl<<*(p->m_CP)<<endl;

 /*p->m_CP<< 2.27868e-07,  1.48699e-08, -1.89067e-09,  4.83273e-08,
 1.48699e-08,  8.45465e-09, -2.07119e-08,  3.89065e-08,
-1.89067e-09, -2.07119e-08,  5.66524e-08, -9.87533e-08,
 4.83273e-08,  3.89065e-08, -9.87533e-08,  2.04974e-07;*/

  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor > >  CP1( m_CP[0]); 
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor > > CP2( p->m_CP[0]); //  = *(p->m_CP); 
 
  Eigen::Matrix<double, 4, 4> C1N = CP1.inverse(); 
  Eigen::Matrix<double, 4, 4> C2N = CP2.inverse(); 
  Eigen::Matrix<double, 4, 4> C3N = w1 * C1N + w2 * C2N; 
  Eigen::Matrix<double, 4, 4> C3 = C3N.inverse(); 

  u3 = w1 * C1N * u1 + w2 * C2N * u2; 
  u3 = C3*u3; 
/*
  cout <<"C1_INV: "<<endl<<C1N<<endl; 
  Eigen::Matrix<double, 4, 4> T = CP1 * C1N; 
  cout <<"C1*C1_INV: "<<endl<<T<<endl;
  cout <<"C2_INV: "<<endl<<C2N<<endl;
  T = CP2 * C2N; 
  cout <<"C2*C2_INV: "<<endl<<T<<endl;
  cout <<"C3_INV: "<<endl<<C3N<<endl;
  cout <<"C3 : "<<endl<<C3<<endl; 
  T = C3*C3N; 
  cout <<"C3*C3_INV: "<<endl<<T<<endl;


 // update covariance and mean 
 
  Eigen::Matrix<double, 3, 3> C1 = CP1.block<3,3>(0,0); //m_CP.block<3,3>(0,0); 
  Eigen::Matrix<double, 3, 3> C1_INV = C1.inverse(); 
  Eigen::Matrix<double, 3, 3> C2 = CP2.block<3,3>(0,0); 
  Eigen::Matrix<double, 3, 3> C2_INV = C2.inverse(); 
  Eigen::Matrix<double, 3, 3> C3_INV = w1 * C1_INV + w2 * C2_INV; 
  Eigen::Matrix<double, 3, 3> C3 = C3_INV.inverse(); 
  
  cout <<"C1: "<<endl<<C1<<endl; 
  cout <<"C1_INV: "<<endl<<C1_INV<<endl; 
  Eigen::Matrix<double, 3, 3> T = C1 * C1_INV; 
  cout <<"C1*C1_INV: "<<endl<<T<<endl;
  cout <<"C2: "<<endl<<C2<<endl; 
  cout <<"C2_INV: "<<endl<<C2_INV<<endl;
  cout <<"C3_INV: "<<endl<<C3_INV<<endl;
  cout <<"C3 : "<<endl<<C3<<endl; 

  Eigen::Vector3d tu = w1*C1_INV*u1 + w2*C2_INV*u2; 
  
  u3 = C3*tu; 
  cout <<"tu: "<<endl<<tu<<endl;

  // d1_ = (*m_CP)(3,3) * (w1*d1_/(*m_CP)(3,3) + w2*p->d1_/(*(p->m_CP))(3,3)); 
  d1_ = m_CP[3][3] * (w1*d1_/ m_CP[3][3] + w2*p->d1_/p->m_CP[3][3]); 
*/

  // (*m_CP).block<3,3>(0,0) = C3; 
  // (*m_CP)(3,3) = 1./(w1/(*m_CP)(3,3) + w2/(*(p->m_CP))(3,3));
  // save cp 
  
  saveCP(C3); 

  // cout <<"u1: "<<endl<<u1<<endl;
  // cout <<"u2: "<<endl<<u2<<endl;
  // cout <<"u3: "<<endl<<u3<<endl;
 
  // cout <<"after m_CP: "<<endl<<*m_CP<<endl;
  
  // cout <<"d1_ = "<<d1_<<endl;
  nx_ = u3(0); ny_ = u3(1); nz_ = u3(2); d1_ = u3(3); 
  normalize(); 

  cx_ = w1 * cx_ + w2 * p->cx_; 
  cy_ = w1 * cy_ + w2 * p->cy_; 
  cz_ = w1 * cz_ + w2 * p->cz_;

  m_npts += p->m_npts; 
  
  return true;
}

bool CPlane::overlapped(CPlane* p)
{
  static double cos_ten = cos(10.*M_PI/180.);
  double cosA = dotProduct(p); 
  if(cosA < 0) cosA = -cosA; 
  if(cosA < cos_ten) return false; 

  for(int i=0; i<mv_patch_area.size(); i++)
  {
    int p_center = mv_patch_area[i]; 
    for(int j=0; j<p->mv_patch_area.size(); j++)
    {
      if(p_center == p->mv_patch_area[j])
      {
        return true; 
      }
    }
  }
  return false; 
}

// group points fall into a patch
int patchPointsInPlaneArea(int H, int W, vector<int>& in, int s, map<int, vector<int> >& PATCH, 
    map<int, int>& PATCH_CENTER) 
{
  /*
  static int sH = 0; 
  static int sW = 0; 
  static vector<int> all; 
  
  if(sH != H || sW != W)
  {
    sH = H; sW = W; 
    int index; 
    all.clear(); 
    all.reserve((H/s)*(W/s));
    for(int row=s; row<H; row += s)
      for(int col=s; col <W; col += s)
      {
        index = row * W + col ;
        all.push_back(index); 
      }
  }*/
  
  // patch points 
  // map<int, vector<int> > PATCH; 
  int p_u = W/s; 
  int p_v = H/s; 
  int index, u, v, tu, tv, patch_id; 
  int h_pu = (p_u / 2) ; 
  int h_pv = (p_v / 2) ; 
  map<int, vector<int> >::iterator it; 
  for(int i=0; i<in.size(); i++)
  {
    // find where this point falls 
    index = in[i]; 
    v = index / W; 
    u = index - v*W; 
    tv = ((v )/p_v)*p_v + h_pv;  // which patch this point lies 
    tu = ((u )/p_u)*p_u + h_pu; 
    patch_id = tv * W + tu; 
    it = PATCH.find(patch_id); 
    if(it == PATCH.end()) // new patch 
    { 
      // cout<<"new PATCH id : "<<patch_id<<" tu ,tv = "<<tu<<" "<<tv<<" u, v = "<<u<<" "<<v<<endl;
      vector<int> g; g.reserve(p_u*p_v);  
      g.push_back(i); 
      // PATCH.insert(make_pair<int, vector<int> >(patch_id, g)); 
      // PATCH_CENTER.insert(make_pair<int, int>(patch_id, i)); 
      PATCH.insert(pair<int, vector<int> >(patch_id, g)); 
      PATCH_CENTER.insert(pair<int, int>(patch_id, i)); 

    }else{
      it->second.push_back(i); 
      
      // update PATCH center 
      int curr_patch_center = in[PATCH_CENTER[it->first]]; 
      if(curr_patch_center == it->first) // hit the exact center 
        continue; 
      // whether a closer center is found, approximately, better way to compare rely on location (u,v) 
      if(abs(curr_patch_center - it->first) > abs(index - it->first)) 
        PATCH_CENTER[it->first] = i; 
    }
  }
  return 1; 
}


// find the key points in the plane area index & (H,W)%s
vector<int> findSubsetForCov(int H, int W, vector<int>& in, int s)
{
  static int sH = 0; 
  static int sW = 0; 
  static vector<int> all; 
  
  if(sH != H || sW != W)
  {
    sH = H; sW = W; 
    int index; 
    all.clear(); 
    all.reserve((H/s)*(W/s));
    for(int row=s; row<H; row += s)
      for(int col=s; col <W; col += s)
      {
        index = row * W + col ;
        all.push_back(index); 
      }
  }

  if(s == 1) return in;

  // find all the key points in the plane area 
  vector<int> ret; 
  ret.reserve(in.size()/s);

  int j = 0; 
  for(int i=0; i<all.size(); i++)
  {
    int key = all[i]; 
    while(j < in.size())
    {
      if(in[j] > key) break; 
      if(in[j] == key) { 
        ret.push_back(key); 
        break; 
      }

      j++;
    }
  }
  
  return ret; 
}

bool computeSmallestEivenVector(Eigen::Matrix3d& cov, Eigen::Vector3d& minor_axis)
{
  float major_v, middle_v, minor_v; 
  Eigen::Vector3d middle_axis, major_axis; 
  bool ret = computeEigenVector(cov, major_axis, middle_axis, minor_axis, major_v, middle_v, minor_v);
  return ret;
}

bool computeEigenVector(Eigen::Matrix3d& covariance_matrix, Eigen::Vector3d& major_axis, Eigen::Vector3d& middle_axis,
    Eigen::Vector3d& minor_axis, float& major_value, float& middle_value, float& minor_value)
{
  Eigen::EigenSolver <Eigen::Matrix <double, 3, 3> > eigen_solver;
  eigen_solver.compute (covariance_matrix);

  Eigen::EigenSolver <Eigen::Matrix <double, 3, 3> >::EigenvectorsType eigen_vectors;
  Eigen::EigenSolver <Eigen::Matrix <double, 3, 3> >::EigenvalueType eigen_values;
  eigen_vectors = eigen_solver.eigenvectors ();
  eigen_values = eigen_solver.eigenvalues ();

  unsigned int temp = 0;
  unsigned int major_index = 0;
  unsigned int middle_index = 1;
  unsigned int minor_index = 2;

  if (eigen_values.real () (major_index) < eigen_values.real () (middle_index))
  {
    temp = major_index;
    major_index = middle_index;
    middle_index = temp;
  }

  if (eigen_values.real () (major_index) < eigen_values.real () (minor_index))
  {
    temp = major_index;
    major_index = minor_index;
    minor_index = temp;
  }

  if (eigen_values.real () (middle_index) < eigen_values.real () (minor_index))
  {
    temp = minor_index;
    minor_index = middle_index;
    middle_index = temp;
  }

  major_value = eigen_values.real () (major_index);
  middle_value = eigen_values.real () (middle_index);
  minor_value = eigen_values.real () (minor_index);

  major_axis = eigen_vectors.col (major_index).real ();
  middle_axis = eigen_vectors.col (middle_index).real ();
  minor_axis = eigen_vectors.col (minor_index).real ();

  major_axis.normalize ();
  middle_axis.normalize ();
  minor_axis.normalize ();

  float det = major_axis.dot (middle_axis.cross (minor_axis));
  if (det <= 0.0f)
  {
    major_axis (0) = -major_axis (0);
    major_axis (1) = -major_axis (1);
    major_axis (2) = -major_axis (2);
  }
  return true;
} 

