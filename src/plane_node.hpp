
template<typename PointT>
int CPlaneNode::extractPlanes( boost::shared_ptr<pcl::PointCloud<PointT> >& in, CamModel* cam)
{
  CPlaneSet pSet; 
  int ret = pSet.extractPlanes(in, *cam); 
  
  if(ret > 0)
  {
    std::vector<CPlane*> tmp_planes;  // store planes 
    std::vector< std::vector<int> > tmp_indices; // index indicates the location of the plane points 

    tmp_planes.swap(pSet.v_plane_set_); 
    tmp_indices.swap(pSet.mv_indices); 
  
    // cout <<" tmp_indices[0] has points : "<<tmp_indices[0].size()<<endl;

    mv_planes.insert(mv_planes.end(), tmp_planes.begin(), tmp_planes.end());
    // mv_indices.insert(mv_indices.end(), tmp_indices.begin(), tmp_indices.end()); 

    for(int i=0; i<ret; i++)
    {      
      mv_landmark_id.push_back(-1); 
      mv_indices.push_back(tmp_indices[i]);
    }
    // if(m_dpt.empty())
  }
  return ret; 
}


