

#include <cmath>

template<typename M>
bool MatrixEqual(M& m1, M& m2, double tolerance)
{
  if(m1.rows() != m2.rows() || m1.cols() != m2.cols()) return false; 
  for(int i=0; i<m1.rows(); i++)
    for(int j=0; j<m1.cols(); j++)
    {
      if(fabs(m1(i,j) - m2(i,j)) > tolerance) 
        return false; 
    }
  return true; 
}

template<typename M>
bool MatrixCheck(M& m)
{
  M m_inv = m.inverse(); 
  M I = M::Identity(); 
  m_inv = m*m_inv; 

  return MatrixEqual(m_inv, I, 1e-7);
}

template<typename M>
bool DominateCheck(M& m)
{
  for(int i=0; i<m.rows(); i++)
  {
    double K = m(i,i); 
    if(K <= 0 ) return false; 
    for(int j=0; j <m.cols(); j++)
    {
      if(j==i) continue; 
      if(K < fabs(m(i,j))) return false; 
    }
  }
  return true; 
}

template<typename M>
void TriangleMatrix(M& m)
{
  for(int i=0 ;i <m.rows(); i++)
    for(int j=0; j<m.cols(); j++)
    {
      if(j==i) continue; 
      m(i,j) = 0; 
    }
}


