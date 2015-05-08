//holds info for reachable volume
//Computes reachable volume from linkages
//Include function for getting random sample from volume
//include function for testing if sample is within reachable volume
#ifndef Reachable_Voleme_h
#define Reachable_Voleme_h
#include <boost/shared_ptr.hpp>
#include "BaseReachableVolume.h"
#include "BaseReachableVolume.h"



////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class ReachableVolume : public BaseReachableVolume {
public:
  string type;
  double m_rmax;
  double m_rmin;
  Vector3d m_base;  //location of base joint of reachable volume in the rv space of the robot
  Vector3d m_baseWorkspace;  //location of base joint of reachable volume in workspace
  int m_nLinks;
  static const bool m_debug = false;

  ReachableVolume(){}

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

  /*Moved to base class
  //Euclidean distance between 2 3D vectors
  const inline static double distance(const Vector3d &_p1, const Vector3d &_p2){
    return sqrt(pow(_p1[0]-_p2[0],2) + pow(_p1[1]-_p2[1],2) + pow(_p1[2]-_p2[2],2));
  }
  */

  //Returns true if _point is in the reachable volume stored in this class
  bool IsInReachableVolume(const Vector3d& _point){
    Vector3d root (0,0,0);
    double dist = distance(root, _point);
    return (dist >= m_rmin && dist <= m_rmax);
  }


  //Returns true if _point is in the reachable volume stored in this class
  bool IsInReachableVolumeBase(const Vector3d& _point){
    double dist = distance(m_base, _point);
    return (dist >= m_rmin && dist <= m_rmax);
  }

  //Returns true if _point is in the reachable volume stored in this class
  bool IsInEpsOfReachableVolumeBase(const Vector3d& _point, double _eps){
    double dist = distance(m_base, _point);
    if(m_debug){
      cout<<"******************"<<endl;
      print();
      cout<<"point = "<<_point<<endl;
      cout<<"dist = "<<dist<<endl;
      cout<<"returning "<<(dist >= (m_rmin-_eps) && dist <= (m_rmax+_eps))<<endl;
    }
    return (dist >= (m_rmin-_eps) && dist <= (m_rmax+_eps));
  }

  /*  Moved to base class
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
  */

  //Returns a random point in the reachable volume
  inline Vector3d GetPointInReachableVolume(){
    double r=m_rmin+(float)rand()/(float)RAND_MAX * (m_rmax - m_rmin);
    double psi = (float)rand()/(float)RAND_MAX * 2 * PI;
    double theta= (float)rand()/(float)RAND_MAX * 2 * PI;
    Vector3d point = convertRadialToEuclidean(r,psi,theta);
    return point;
  }


  //returns true if rv is contained within this reachable volume
  bool isContained(const ReachableVolume &_rv){
    if(m_debug){
      cout<<"is contained, comparing joints"<<endl;
      print();
      _rv.print();
    }
    double dist = distance(m_base,_rv.m_base);
    if(dist-_rv.m_rmax >= m_rmin && dist + _rv.m_rmax <= m_rmax){
      if(m_debug) cout<<"true1"<<endl;
       return true;
    }
    if(dist+_rv.m_rmax <= m_rmax && _rv.m_rmin-dist >= m_rmin){
      if(m_debug) cout<<"true2"<<endl;
      return true;
    }
    if(m_debug) cout<<"false"<<endl;
    return false;
  }


  //returns true if the reachable volumes are the same
  inline bool isEqual(ReachableVolume &_rv){
    if(m_debug){
      cout<<"is equal, comparing joints"<<endl;
      print();
      _rv.print();
    }
    bool isEq =  _rv.m_rmin==m_rmin && _rv.m_rmax==m_rmax && _rv.m_base[0]==m_base[0] && _rv.m_base[1]==m_base[1] && _rv.m_base[2]==m_base[2];
    if(m_debug) cout<<isEq<<endl;
    return isEq;
  }

  const inline void print() const{
    cout<<m_rmin<<", "<<m_rmax<<", ("<<m_base<<")"<<endl;
  }

  const inline Vector3d getNearestPointInRV(const Vector3d &_p){
    double d = distance(m_base,_p);
    if(d>m_rmax){
      Vector3d nearest = m_base;
      nearest[0]+=m_rmax*(_p[0]-nearest[0])/d;
      nearest[1]+=m_rmax*(_p[1]-nearest[1])/d;
      nearest[2]+=m_rmax*(_p[2]-nearest[2])/d;
      return nearest;
    }
    if(d<m_rmin){
      Vector3d nearest = m_base;
      nearest[0]+=m_rmin*(_p[0]-nearest[0])/d;
      nearest[1]+=m_rmin*(_p[1]-nearest[1])/d;
      nearest[2]+=m_rmin*(_p[2]-nearest[2])/d;
      return nearest;
    }
    return _p;
  }
};

#endif
