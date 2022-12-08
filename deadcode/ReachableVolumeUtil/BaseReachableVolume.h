#ifndef Base_Reachable_Voleme_h
#define Base_Reachable_Voleme_h

#include <boost/shared_ptr.hpp>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// holds info for reachable volume
/// Computes reachable volume from linkages
/// Include function for getting random sample from volume
/// include function for testing if sample is within reachable volume
////////////////////////////////////////////////////////////////////////////////
class BaseReachableVolume{
public:
  string type;
  double m_rmax;
  double m_rmin;
  Vector3d m_base;  //location of base joint of reachable volume in the rv space of the robot
  Vector3d m_baseWorkspace;  //location of base joint of reachable volume in workspace
  int m_nLinks;
  static const bool m_debug = false;

  BaseReachableVolume(){}

  /*
  ReachableVolume(double _rmin, double _rmax, int _nLinks=1){
    m_rmax = _rmax;
    m_rmin = _rmin;
    m_nLinks=_nLinks;
  }


  ReachableVolume(const ReachableVolume &_r1, const ReachableVolume &_r2){
    m_nLinks=_r1.m_nLinks+_r2.m_nLinks;
    m_rmax = _r1.m_rmax + _r2.m_rmax;
    if(_r1.m_rmin>_r2.m_rmin){
      m_rmin = max((double)0,_r1.m_rmin - _r2.m_rmax);
    }else{
      m_rmin = max((double)0,_r2.m_rmin - _r1.m_rmax);
    }
    m_base=_r1.m_base+_r2.m_base;
  }

  //Computes Minkowski sum of reachable volumes r1 and r2
  //if r1 and r2 are the reachable volumes of 2 chains then this will be the reachable volume of the merged chain
  ReachableVolume(ReachableVolume *_r1, ReachableVolume *_r2){
    m_nLinks=_r1->m_nLinks+_r2->m_nLinks;
    m_rmax = _r1->m_rmax + _r2->m_rmax;
    if(_r1->m_rmin>_r2->m_rmin){
      m_rmin = max((double)0,_r1->m_rmin - _r2->m_rmax);
    }else{
      m_rmin = max((double)0,_r2->m_rmin - _r1->m_rmax);
    }
    m_base=_r1->m_base+_r2->m_base;
  }

  void intersectCosentricRV(const ReachableVolume &_r){
    if(_r.m_rmin>m_rmin){
      m_rmin = max((double)0,_r.m_rmin - m_rmax);
    }else{
      m_rmin = max((double)0,m_rmin - _r.m_rmax);
    }
    m_rmax = _r.m_rmax + m_rmax;
  }
  */

  //Euclidean distance between 2 3D vectors
  const inline static double distance(const Vector3d &_p1, const Vector3d &_p2){
    return sqrt(pow(_p1[0]-_p2[0],2) + pow(_p1[1]-_p2[1],2) + pow(_p1[2]-_p2[2],2));
  }

  //Returns true if _point is in the reachable volume stored in this class
  //is in reachable volume of point (0,0,0)
  bool IsInReachableVolume(const Vector3d& _point);


  //Returns true if _point is in the reachable volume stored in this class
  //in in reachable volume of base
  //if point - base in reachable volume
  bool IsInReachableVolumeBase(const Vector3d& _point);

  //Returns true if _point is in the reachable volume stored in this class
  bool IsInEpsOfReachableVolumeBase(const Vector3d& _point, double _eps);

  //_r, _psi, and _theta are the position of the point with respect to the base of the chain
  //r = distance from origin, _psi = rotation in x y plane, _theta = inclination in z direction
  //Returns the Euclidean coordinates of this point
  //The point returned is a point in the linkage's reachable volume space (where the origin is located at the base of the chain)
  inline static Vector3d convertRadialToEuclidean(double _r, double _psi, double _theta){
    Vector3d p;
    p[2] = sin(_theta)*_r;
    double rxyPlane = cos(_theta)*_r;
    p[1] = sin(_psi)*rxyPlane;
    p[0] = cos(_psi)*rxyPlane;
    return p;
  }

  //Returns a random point in the reachable volume
  inline Vector3d GetPointInReachableVolume();



  //returns true if rv is contained within this reachable volume
  //bool isContained(const ReachableVolume &_rv){}


  //returns true if the reachable volumes are the same
  //inline bool isEqual(ReachableVolume &_rv){}

  const inline void print() const;

  const inline Vector3d getNearestPointInRV(const Vector3d &_p);

};

#endif
