
#include "plane_node.h"

using namespace std; 

CPlaneNode::CPlaneNode()
{

}

CPlaneNode::~CPlaneNode()
{
  for(int i=0; i<mv_planes.size(); i++)
  {
    delete mv_planes[i]; 
    mv_planes[i] = NULL; 
  }
  mv_planes.clear(); 
}

int CPlaneNode::extractPlanes(cv::Mat& rgb, cv::Mat& dpt, CamModel* p_cam)
{
  CPlaneSet pSet; 
  int ret = pSet.extractPlanes(rgb, dpt, *p_cam); 

  if(ret > 0)
  {
    mv_planes.swap(pSet.v_plane_set_);
    mv_indices.swap(pSet.mv_indices); 

    // 
    mv_landmark_id.resize(ret, -1); // -1 mean not been added yet

    if(m_dpt.empty())
      m_dpt = dpt.clone();  // save the depth of this frame 
  }
  return ret; 
}

bool CPlaneNode::mergeOverlappedPlanes(int newly_added) // if the newly added planes coincide with the old ones 
{
    bool ret = false; 
    map<int, int> merged_set; 
    int m = mv_planes.size(); 
    // first check this node itself
    for(int i=m-1; i>= m-newly_added; i--)
    {
      CPlane* pnew = mv_planes[i]; 
      for(int j=0; j<m-newly_added; j++)
      {
        CPlane* pold = mv_planes[j]; 
        if(pold->overlapped(pnew))
        {
          // merged_set.insert(make_pair<int,int>(j,i)); 
          merged_set.insert(pair<int,int>(j,i)); 
          ret = true; 
          break;
        }
      }
    }
    
    if(!ret) return ret; // no need to merge 

    // merge planes 
    map<int, int>::iterator it = merged_set.begin(); 
    while(it != merged_set.end())
    {
      mv_planes[it->first]->merge(mv_planes[it->second]); 
      mv_indices[it->first].insert(mv_indices[it->first].end(), 
          mv_indices[it->second].begin(), mv_indices[it->second].end());

      if(mv_landmark_id[it->first] < 0)
      {
        ROS_ERROR("%s after plane merging, the second plane's id %d the first plane's id %d", __FILE__, mv_landmark_id[it->second], mv_landmark_id[it->first]); 
      }
      delete mv_planes[it->second]; 
      mv_planes[it->second] = NULL; 
      mv_indices[it->second].clear(); 
      it++;
    }

    // delete the redundant planes 
    vector<CPlane*>::iterator itp = mv_planes.begin(); 
    vector<vector<int> >::iterator iti  = mv_indices.begin();
    vector<int>::iterator itl = mv_landmark_id.begin();
    while(itp != mv_planes.end() )
    {
      if(*itp == NULL)
      {
        itp = mv_planes.erase(itp); 
        iti = mv_indices.erase(iti);
        itl = mv_landmark_id.erase(itl);
      } else{
        ++itp;
        ++iti;
        ++itl;
      }
    }
    return ret; 
}

void CPlaneNode::testEigen()
{
  for(int i=0; i<mv_planes.size(); i++)
  {
    mv_planes[i]->testEigen(); 
  }
}


void CPlaneNode::testPtOnPlane()
{
  CamModel* pcam = CamModel::gCamModel(); 
  for(int i=0; i<mv_planes.size(); i++)
  {
    ofstream ouf("pt_on_plane.log"); 
    vector<int>& indices = mv_indices[i]; 
    // vector<double> dis_p2p(indices.size()); 
    // vector<double> dis_p2pl(indices.size()); 
    double dis_p2pl = 0;
    double dis_p2p = 0; 
    int N = 0; 
    CPlane* pl = mv_planes[i]; 
    int index, u, v; 
    double px, py, pz; 
    double plx, ply, plz; 
    for(int j=0; j<indices.size(); j++)
    {
      index = indices[j]; 
      v = index / m_dpt.cols; 
      u = index - v * m_dpt.cols;
      pz = m_dpt.at<unsigned short>(index)*(pcam->m_z_scale); 
      pcam->convertUVZ2XYZ(u,v, pz, px, py, pz); 
      
      // calculate the pt's distance to plane and to the point projected onto the plane  
      // dis_p2pl.push_back(d2pl); 
      bool valid = pl->ptOnPlane(px, py, pz, plx, ply, plz); 
      if(valid)
      {
        double d2pl = pl->dis2plane(px, py, pz); 
        if(d2pl != d2pl) 
        {
          cout <<"what? d2pl = "<<d2pl<<" px = "<<px<<" py = "<<py<<" pz = "<<pz<<endl;
        }
        double dp2p = SQ(px-plx) + SQ(py-ply) + SQ(pz-plz); 
        N++; 
        dis_p2p += dp2p; 
        dis_p2pl += SQ(d2pl); 
        ouf<<px<<" "<<py<<" "<<pz<<endl;
      }
    }
    if(N == 0){
      cerr <<__FILE__<<" N=0, something error here"<<endl;
    }
    dis_p2p /= N; 
    dis_p2p = sqrt(dis_p2p); 
    dis_p2pl /= N; 
    dis_p2pl = sqrt(dis_p2pl); 
      
    // show the result on plane i
    pl->print("plane 's information"); 
    cout <<"Std dis between pt on the plane is "<<dis_p2p<<" Sigma: "<<SQ(dis_p2p)<<endl; 
    cout <<"Std dis between pt to the plane is "<<dis_p2pl<<" Sigma: "<<SQ(dis_p2pl)<<endl;
  }
}
