#include "IModel.h"
#include "Matrix.h"

using namespace std;


double
IModel::
getRadiusToJoint(const EdgeList& _el, int _jointIndex, bool _useFirst) const {
  const Edge& edge = _el[_jointIndex];
  Point3d p = _useFirst ? edge.first : edge.second;
  return sqrt(pow(m_center[0]-p[0],2.0) + pow(m_center[2]-p[2],2.0));
}


double
IModel::
getBBXRadius() const {
  double xrange = fabs(m_bbx[1] - m_bbx[0]) / 2.0;
  double zrange = fabs(m_bbx[5] - m_bbx[4]) / 2.0;
//  if(xrange > zrange) return xrange; //return smallest
//  else return zrange;
  return max(xrange, zrange); // above code returns largest, comment is wrong.
}


void
IModel::
updateBBX(const Point3d& _p) {
  //x
  m_bbx[0] = min(m_bbx[0], _p[0]);
  m_bbx[1] = max(m_bbx[1], _p[0]);
  //y_
  m_bbx[2] = min(m_bbx[2], _p[1]);
  m_bbx[3] = max(m_bbx[3], _p[1]);
  //z_
  m_bbx[4] = min(m_bbx[4], _p[2]);
  m_bbx[5] = max(m_bbx[5], _p[2]);
}


void
IModel::
resetBBX() {
  m_bbx.clear();
  m_bbx = {1e10, -1e10, 1e10, -1e10, 1e10, -1e10};
}


void
IModel::
skelBBX(const EdgeList& _el) {
  if(m_bbx.empty())
    resetBBX();
  for(const auto& e : _el) {
    const Point3d& p1 = e.first;
    const Point3d& p2 = e.second;
    updateBBX(p1);
    updateBBX(p2);
  }

  m_center[0] = (m_bbx[0] + m_bbx[1]) / 2.0;
  m_center[1] = (m_bbx[2] + m_bbx[3]) / 2.0;
  m_center[2] = (m_bbx[4] + m_bbx[5]) / 2.0;

  cout << " BBX: [" ;
  for(size_t i = 0; i < 6; ++i)
    cout <<  m_bbx[i] << " ";
  cout << "]"
       << "\n\tcenter: " << m_center
       << "\n\theight: " << m_bbx[3] - m_bbx[2] << endl;
}


void
IModel::
genUniquePts() {
  if(m_uniquePts.empty()) {
    for(const auto& e : m_edgeList) {
      const Point3d& p1 = e.first;
      const Point3d& p2 = e.second;
      bool addP1 = true;
      bool addP2 = true;
      for(const auto& p : m_uniquePts) {
        double dist1 = (p1 - p).norm();
        double dist2 = (p2 - p).norm();
        if(dist1 < m_tolerance) addP1 = false;
        if(dist2 < m_tolerance) addP2 = false;
      }
      if(addP1)
        m_uniquePts.push_back(p1);
      if(addP2)
        m_uniquePts.push_back(p2);
    }
  }
}


void
IModel::
genDir(EdgeList& _el1, EdgeList& _el2, size_t _index) {
  const Edge& e1 = _el1[_index];
  const Edge& e2 = _el2[_index];
  const Point3d& p1 = e1.first;
  const Point3d& p2 = e2.first;
  m_dir = m_directionChange = p2 - p1;
  m_distanceChange = m_directionChange.norm();
}


void
IModel::
ComputeFaceNormal() {
  m_triN.reserve(m_triP.size());
  m_normals.reserve(m_triP.size());

  int index = 0;
  for(const auto& tri : m_triP) {
    const Point3d& p1 = m_points[tri[0]];
    const Point3d& p2 = m_points[tri[1]];
    const Point3d& p3 = m_points[tri[2]];

    Vector3d n = ((p2 - p1) % (p3 - p1)).normalize();
    m_normals.push_back(n);
    m_triN.push_back(IModel::Tri(index,index,index));
    ++index;
  }
}

/*------------------------------- Dead Code ----------------------------------*/

/*
void
IModel::
transform(EdgeList& el, double radius, double height) {
     typedef EdgeList::iterator EIT;
     Vector3d vecToCenter(m_center[0],m_center[1],m_center[2]);
     Matrix2x2 rotM(cos(m_rot), -1*sin(m_rot), sin(m_rot), cos(m_rot));
     Vector2d W_dir(0,1);

     double bbx_radius = getBBXRadius();
     double bbx_height = fabs(m_bbx[3]-m_bbx[2]);
     double scale = radius/bbx_radius;//model with get scaled by this factor
     cout << " r1: " << radius << " r2: " << bbx_radius << " scale: " << scale << endl;
     m_dir = scale * m_dir;
     m_directionChange = scale * m_dir;
     m_distanceChange *= scale;
     for(EIT eit=el.begin(); eit!=el.end(); eit++) {
     Edge& edge = *eit;
     Point3d& p1 = edge.first;                    //get points
     Point3d& p2 = edge.second;
     Point3d p1_new = p1 + -1.0 * vecToCenter;    //bring back to center
     Point3d p2_new = p2 + -1.0 * vecToCenter;
     Vector2d v1(p1_new[0],p1_new[2]);            //init 2d vec using x,z vals
     Vector2d v2(p2_new[0],p2_new[2]);
     Vector2d v1_new = rotM * v1;                 //rotate
     Vector2d v2_new = rotM * v2;
     Vector3d v1_3d_new(v1_new[0], p1_new[1], v1_new[1]); //make 3d vectors for scaling
     Vector3d v2_3d_new(v2_new[0], p2_new[1], v2_new[1]);
     v1_3d_new = scale * v1_3d_new;               //scale
     v2_3d_new = scale * v2_3d_new;

     p1[0] = v1_3d_new[0];                           //use new vals to update pos
  //p1[1] = v1_3d_new[1];                           //should now be facing z dir
  p1[2] = v1_3d_new[2];
  p2[0] = v2_3d_new[0];
  //p2[1] = v2_3d_new[1];
  p2[2] = v2_3d_new[2];
  }//endfor
  resetBBX();
  skelBBX(el);
  double y_offset = m_bbx[2];
  if(1)
  for(EIT eit=el.begin(); eit!=el.end(); eit++) {
  Edge& edge = *eit;
  Point3d& p1 = edge.first;                    //get points
  Point3d& p2 = edge.second;
  p1[1] = height* (p1[1]-y_offset) / bbx_height;
  p2[1] = height* (p2[1]-y_offset) / bbx_height;
  }//endfor
  resetBBX();
  skelBBX(el);
}
*/
