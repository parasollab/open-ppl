//holds info for directed reachable volume
//stored intersection of drv with xy plane
//Include function for getting random sample from volume
//include function for testing if sample is within reachable volume


//convention:
//Origin located at the base joint of the robot
//y axis is the axis of the base joint
//  for rotational joints, rotation base joint is about y axis
//  for planar joints, motion of base joint is in xy plane
//DRV formed by rotating DRV_xy about y axis

#ifndef DirectedReachable_Voleme_h
#define DirectedReachable_Voleme_h
#include <boost/shared_ptr.hpp>
#include "BaseReachableVolume.h"
#include "ReachableVolumeJoint.h"

class ReachableVolumeJoint;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// segment of boarder of DRV_xy
////////////////////////////////////////////////////////////////////////////////
class Segment_DRV_xy{
 public:
  bool m_Top;  //is the segment part of the top or bottom of DRV_xy

  //segment is the arc defined by d and center
  double m_d;
  Vector2d m_center;

  Vector2d m_endpoint_1;  //endpoint shared with segment before it in chain
  Vector2d m_endpoint_2; //endpoint shared with segment after it in chain



  inline double distance(Vector2d _p1, Vector2d _p2){
    return sqrt((_p1[0]-_p2[0])*(_p1[0]-_p2[0] + (_p1[1]-_p2[1])*_p1[1]-_p2[1]));
  }

  bool Below(Vector2d _p){
    if(distance(_p,m_center)<m_d) return true;
    return false;
  }


  bool Internal(Vector2d _p){
    if(m_Top){
      return Below(_p);
    }else{
      return !Below(_p);
    }
  }

  inline void transformVector2dToParentFrame(double _d, double _theta, Vector2d &_p){
    if(_theta<0 || _theta> PI)
      cout<<endl<<"***Warning transformVector2dToParentFrame expactes a value of _theta between 0 and PI passed value of _theta ="<<_theta<<endl;
    _p[1]+=_d;
    _p = rotateCoordinatePlane(_p[0], _p[1], _theta);
  }


 //(x,y)=point in plane
 //rotate the plane by rotateAngle
  //merge with function in reachable volume joint
 static inline Vector2d rotateCoordinatePlane(double x, double y, double rotateAngle){
   double r = sqrt(pow(x,2)+pow(y,2));
   double theta = atan2(y,x);
   Vector2d rotatedPoint;
   rotatedPoint[0]=r*cos(theta+rotateAngle);
   rotatedPoint[1]=r*sin(theta+rotateAngle);
   return rotatedPoint;
 }


  //used to convert from reference frame of p' to reference frame of p'-1
  inline void transformToParentFrame(double _d, double _theta){
    transformVector2dToParentFrame(_d,_theta,m_center);
    transformVector2dToParentFrame(_d,_theta,m_endpoint_1);
    transformVector2dToParentFrame(_d,_theta,m_endpoint_2);
  }

  //returns first point of intersection (ordered from m_endpoint_1 to m_endpoint_2)
  //this would give the point of intersection that is closest to m_endpoint_1 on surface of _s1
  static bool GetIntersection(Segment_DRV_xy &_s1, Segment_DRV_xy &_s2, Vector2d &_p){
    vector<Vector2d> points;
    bool returnVal=GetIntersection(_s1,_s2,points);
    if(points.size()>0){
      _p=points[0];
      return returnVal;
    }
    return false;
  }

  //returns sets _p to be intersection of _s1 and _s2, returns false if _dr1 and _dr2 don't intersect
  static bool GetIntersection(Segment_DRV_xy &_s1, Segment_DRV_xy &_s2, vector<Vector2d> &_points){

    const double a = _s1.m_center[0];
    const double b = _s1.m_center[1];
    const double c = _s2.m_center[0];
    const double d = _s2.m_center[1];
    const double r0 = _s1.distance(_s1.m_center,_s1. m_endpoint_1);
    const double r1 = _s1.distance(_s2.m_center,_s2. m_endpoint_1);
    const double D = sqrt((c-a)*(c-a)+(d-b)*(d-b));  //distance between centers
    if(D>=r0+r1 || D <= abs(r0-r1)){
      //no intersection
      return false;
    }
    double delta = (1/4)*sqrt((D+r0+r1)*(D+r0-r1)*(D-r0+r1)*(-D+r0+r1));
    Vector2d p1;
    Vector2d p2;
    p1[0]=(a+c)/2+(c-a)*(r0*r0-r1*r1)/(2*D*D)+2*(b-d)*delta/(D*D);
    p1[1]=(b+d)/2+(d-b)*(r0*r0-r1*r1)/(2*D*D)-2*(a-c)*delta/(D*D);

    p2[0]=(a+c)/2+(c-a)*(r0*r0-r1*r1)/(2*D*D)-2*(b-d)*delta/(D*D);
    p2[1]=(b+d)/2+(d-b)*(r0*r0-r1*r1)/(2*D*D)+2*(a-c)*delta/(D*D);

    //need to impliment
    return false;
  }

};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class DRV_xy{
  //Holds geometric information for intersection of DRV with xy plane
  //starts with first bottom segment and goes in counter clockwise direction ending with last top secment
 public:
  list<Segment_DRV_xy> m_Boarder;
  list<Segment_DRV_xy>::iterator m_lastBottom;
  list<Segment_DRV_xy>::iterator m_topCrossYAxis;
  list<Segment_DRV_xy>::iterator m_bottomCrossYAxis;




  const void print();

  //called when converting DRV(p,p') to DRV(p,p'-1)
  //Converts DRV(p,p') from reverence frame p'-1
  //Converted DRV(p,p') will then be used to compute  DRV(p,p'-1)
  inline void transformToParentFrame(double _d, double _theta){
    for(list<Segment_DRV_xy>::iterator i = m_Boarder.begin(); i!=m_Boarder.end(); i++){
      i->transformToParentFrame(_d,_theta);
    }
  }


  bool isInDRV_xy(double _x, double _y){
    for(list<Segment_DRV_xy>::iterator i = m_Boarder.begin(); i!=m_Boarder.end(); i++){
      Vector2d p;
      p[0]=_x;
      p[1]=_y;
      if(!i->Internal(p))
	return false;
    }
    return true;
  }

  //finds where _s intersects m_Boarder
  //adds it to m_Boarder between elements of m_Boarder that it intersects with
  //updates endpoints of elements it intersects with to be points of intersection
  //removes any segments between points of intersection
  void AddSegment(Segment_DRV_xy &_s){
    list<Segment_DRV_xy>::iterator beforeInsert;
    list<Segment_DRV_xy>::iterator afterInsert;
    list<Segment_DRV_xy>::iterator temp_i;
    bool afterInstertLaterThanBeforeInsert=false;
    for(list<Segment_DRV_xy>::iterator i = m_Boarder.begin(); i!=m_Boarder.end(); i++){
      if(_s.Below(i->m_endpoint_1) && !_s.Below(i->m_endpoint_2)){
	if(_s.m_Top){
	  afterInstertLaterThanBeforeInsert=true;
	  afterInsert=i;
	}else{
          afterInstertLaterThanBeforeInsert=false;
	  beforeInsert=i;
	}
      }
      if(!_s.Below(i->m_endpoint_1) && _s.Below(i->m_endpoint_2)){
        if(_s.m_Top){
          afterInstertLaterThanBeforeInsert=false;
          beforeInsert=i;
        }else{
          afterInstertLaterThanBeforeInsert=true;
          afterInsert=i;
        }
      }
    }

    _s.GetIntersection(_s,*beforeInsert,_s.m_endpoint_1);
    beforeInsert->m_endpoint_2=_s.m_endpoint_1;
    _s.GetIntersection(_s,*afterInsert,_s.m_endpoint_2);

    if(!afterInstertLaterThanBeforeInsert){
      //removed segments include end points of chain
      temp_i=afterInsert;
      temp_i++;
      m_Boarder.erase(m_Boarder.begin(),temp_i);
      m_Boarder.erase(beforeInsert,m_Boarder.end());
      if(_s.m_Top){
    	m_Boarder.push_back(_s);
      }else{
	m_Boarder.insert(m_Boarder.begin(),_s);
      }
    }else{
      afterInsert->m_endpoint_1=_s.m_endpoint_2;
      temp_i=beforeInsert;
      temp_i++;
      m_Boarder.erase(temp_i,afterInsert);
      m_Boarder.insert(afterInsert,_s);

      if(!beforeInsert->m_Top && _s.m_Top){
	//_s is first top segmetn, set m_lastBottom = to beforeInsert
	m_lastBottom=beforeInsert;
      }
      if(!_s.m_Top && afterInsert->m_Top){
	//_s is last bottom segment, set m_lastBottom = position of _s
        m_lastBottom=beforeInsert;
        m_lastBottom++;
      }
    }
  }

  //mirrors srv_xy about y axis
  //used as final step for computing DRV(p,p'-1)
  void mirrorAboutYAxis(){

    if(m_lastBottom->m_endpoint_2[1]>0){

      //Case 1:  DRV and mirror do not overlap
      //In this case m_endpoint_2 of m_lastBottom is positive
      //add mirror between m_lastBottom and m_lastBottom +1
      list<Segment_DRV_xy> mirror;
      list<Segment_DRV_xy>::iterator newLastBottom;
      for(list<Segment_DRV_xy>::iterator i = m_Boarder.begin(); i!=m_Boarder.end(); i++){
	Segment_DRV_xy temp = *i;
	temp.m_center[1]*=-1;
	temp.m_endpoint_1[1]*=-1;
	temp.m_endpoint_2[1]*=-1;
	mirror.push_back(temp);
	if(!i->m_Top){
	  if(i==m_Boarder.begin()){
	    newLastBottom=m_Boarder.begin();
	  }else{
	    newLastBottom++;
	  }
	}

	m_Boarder.splice(m_lastBottom, mirror);
	m_lastBottom=newLastBottom;
      }
    }else{

	//Case 2 DRV and mirror overlap with continuous boarder
	//Remove part of DRV with negative value (part of DRV to left of y axis)
	//Case 3:  mirror passes through DRV
	//Start at m_lastBottom
	//Itterate wntil you find segment whoes mirror insersects with DRV (done by testing if m_endpoint_2 if mirrior is in DRV)

	list<Segment_DRV_xy>::iterator i = m_lastBottom;

	list<Segment_DRV_xy> mirror;
	while(!isInDRV_xy(i->m_endpoint_2[0],(i->m_endpoint_2[1])*-1)){
	  Segment_DRV_xy temp = *i;
	  temp.m_center[1]*=-1;
	  temp.m_endpoint_1[1]*=-1;
	  temp.m_endpoint_2[1]*=-1;
	  mirror.push_back(temp);
	  i++;
	}



	list<Segment_DRV_xy>::iterator pointerToIntersection=i;
	Segment_DRV_xy segmentOfIntersection=*i;
	segmentOfIntersection.m_center[1]*=-1;
	segmentOfIntersection.m_endpoint_1[1]*=-1;
	segmentOfIntersection.m_endpoint_2[1]*=-1;
	mirror.push_back(segmentOfIntersection);

	i = m_lastBottom;
	Vector2d pointOfIntersection;
	while(!i->GetIntersection(*i,segmentOfIntersection,pointOfIntersection)){
	  i++;
	}

	list<Segment_DRV_xy>::iterator j = m_lastBottom;
	j++;

	i->m_endpoint_2=pointOfIntersection;
	mirror.back().m_endpoint_1=pointOfIntersection;
	int mirror_size=mirror.size();

	m_lastBottom->GetIntersection(*m_lastBottom,mirror.front(),pointOfIntersection);
	m_lastBottom->m_endpoint_2=pointOfIntersection;
	mirror.front().m_endpoint_1=pointOfIntersection;

	m_Boarder.erase(j,i);  //remove portion inside of mirror
	m_Boarder.splice(m_lastBottom, mirror); //insert mirror into m_Boarder
	advance (m_lastBottom,mirror_size);

	//If point of intersection occours on y axis then it is case one, otherwise it is case 2
	if(pointOfIntersection[1]!=0){


	  i = m_lastBottom;
	  //mirror.clear();
	  while(isInDRV_xy(i->m_endpoint_2[0],i->m_endpoint_2[1]*-1)){
	    i++;
	  }
	  pointerToIntersection=i;
	  segmentOfIntersection=*i;
	  segmentOfIntersection.m_center[1]*=-1;
	  segmentOfIntersection.m_endpoint_1[1]*=-1;
	  segmentOfIntersection.m_endpoint_2[1]*=-1;

	  pointerToIntersection->GetIntersection(*pointerToIntersection,segmentOfIntersection,pointOfIntersection);
	  segmentOfIntersection.m_endpoint_2=pointOfIntersection;
	  segmentOfIntersection.m_endpoint_1=pointOfIntersection;

	  mirror.clear();
	  mirror.push_back(segmentOfIntersection);

	  while(i->m_endpoint_2[1]<=0){
	    i++;
	    Segment_DRV_xy temp = *i;
	    temp.m_center[1]*=-1;
	    temp.m_endpoint_1[1]*=-1;
	    temp.m_endpoint_2[1]*=-1;
	    mirror.push_back(temp);
	  }
	  i->GetIntersection(*i,mirror.back(),pointOfIntersection);
	  i->m_endpoint_1[1]*=-1;
	  mirror.back().m_endpoint_2[1]*=-1;
	  j=pointerToIntersection;
	  j++;
	  m_Boarder.erase(j,i);  //remove portion inside of mirror
	  m_Boarder.splice(pointerToIntersection, mirror); //insert mirror into m_Boarder
	}
    }
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class DirectedReachableVolume : public BaseReachableVolume{
  //represents DRV
  //equivalent to DRV_yx rotated about xy plane
  //should be equivalent to Reachable volumes class with same interface

public:
  DRV_xy m_drv_xy;
  string type;
  int m_jID;  //id of joint this is DRV of
  int m_baseID;  //id of root of drv
  int m_nLinks;  //number of links between jid and base
  static const bool m_debug = false;

  DirectedReachableVolume(){}

  DirectedReachableVolume(DRV_xy _drv_xy, int _nLinks=1){
    m_drv_xy=_drv_xy;
    m_nLinks=_nLinks;
  }

  //sets to be drv of one link with specified length and offset angle
  DirectedReachableVolume(double _LinkLength, double _OffsetAngle){
    m_nLinks=1;
  }


  //inputs the drv of a child joint along with the length and offset angle connecting joint to child
  //sets to be drv of parent
  DirectedReachableVolume(DirectedReachableVolume _drv, double _LinkLength, double _OffsetAngle){
    m_nLinks=1;
  }

  /*
  //Removed because drv method does not suppot composing drvs in this manner
  DirectedReachableVolume(const DirectedReachableVolume &_drv1, const DirectedReachableVolume &_drv2){
    m_nLinks=_r1.m_nLinks+_r2.m_nLinks;
    m_rmax = _r1.m_rmax + _r2.m_rmax;
    if(_r1.m_rmin>_r2.m_rmin){
      m_rmin = max((double)0,_r1.m_rmin - _r2.m_rmax);
    }else{
      m_rmin = max((double)0,_r2.m_rmin - _r1.m_rmax);
    }
    m_base=_r1.m_base+_r2.m_base;
  }
  */

  /*removed
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



  //Returns true if _point is in the reachable volume stored in this class
  bool IsInReachableVolume(const Vector3d& _point){
    double x = sqrt(_point[0]*_point[0] + _point[2]*_point[2]);
    return m_drv_xy.isInDRV_xy(x, _point[1]);
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
  */

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

  /*removed
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
    cout<<m_jID<<", "<<m_baseID<<", "<<m_nLinks<<""<<endl;
    m_drv_xy.print();
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
  */
};

#endif
