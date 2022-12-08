#ifndef Reachable_Voleme_Joint_h
#define Reachable_Voleme_Joint_h
#include "ReachableVolume.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class ReachableVolumeJoint{
 private:
  static const bool m_debug = false;

 public:
  int m_jointID;
  int m_nLinks;
  shared_ptr<vector<shared_ptr<ReachableVolume> > > m_reachableVolumesJoint;  //the reachable volumes whoes intersection the joint may be located in


  ReachableVolumeJoint(){
    m_reachableVolumesJoint=shared_ptr<vector<shared_ptr<ReachableVolume> > >(new vector<shared_ptr<ReachableVolume> >);
  }



  ReachableVolumeJoint(shared_ptr<vector<shared_ptr<ReachableVolume> > > &_reachableVolumesJoint){
    m_reachableVolumesJoint=_reachableVolumesJoint;
    if(m_reachableVolumesJoint->size()>0)
      m_nLinks = (*m_reachableVolumesJoint)[0]->m_nLinks;
    m_reachableVolumesJoint=shared_ptr<vector<shared_ptr<ReachableVolume> > >(new vector<shared_ptr<ReachableVolume> >);
    m_reachableVolumesJoint->clear();
    m_reachableVolumesJoint->resize(0);
  }

  ReachableVolumeJoint(double _rmax, double _rmin, int _nLinks=1){
    m_nLinks = _nLinks;
    m_reachableVolumesJoint=shared_ptr<vector<shared_ptr<ReachableVolume> > >(new vector<shared_ptr<ReachableVolume> >);
    ReachableVolume *tmp = new ReachableVolume(_rmax, _rmin, _nLinks);
    shared_ptr<ReachableVolume> ptr = shared_ptr<ReachableVolume>(tmp);
    m_reachableVolumesJoint->push_back(ptr);
  }


  ReachableVolumeJoint(ReachableVolumeJoint* _r1, ReachableVolumeJoint* _r2){
    cout<<"adding constraints "<<_r1->m_reachableVolumesJoint->size();
    cout<<"adding constraints 2 "<<_r2->m_reachableVolumesJoint->size();

    m_reachableVolumesJoint = _r1->m_reachableVolumesJoint;
    m_nLinks=_r1->m_nLinks+_r2->m_nLinks;
    addConstraint(_r2->m_reachableVolumesJoint);
    cout<<"added constraints "<<m_reachableVolumesJoint->size();
  }

  ReachableVolumeJoint* mkSum(double _min, double _max){
    ReachableVolumeJoint *rvj = new ReachableVolumeJoint();
    for(vector<shared_ptr<ReachableVolume> >::iterator i = m_reachableVolumesJoint->begin(); i<m_reachableVolumesJoint->end(); i++){
      double maxForTmp = _max + (*i)->m_rmax;
      double minForTmp;
      if(_min>(*i)->m_rmin){
	minForTmp = max((double)0,_min - (*i)->m_rmax);
      }else{
	minForTmp = max((double)0,(*i)->m_rmin - _max);
      }
      //cout<<"rmin, rmax"<<minForTmp<<", "<<maxForTmp<<endl;
      rvj->m_reachableVolumesJoint->push_back(shared_ptr<ReachableVolume>(new ReachableVolume(minForTmp, maxForTmp)));
      rvj->m_reachableVolumesJoint->back()->m_base=(*i)->m_base;
    }
    return rvj;
  }

  void print(){
    for(unsigned int i=0; i<m_reachableVolumesJoint->size();i++)
      (*m_reachableVolumesJoint)[i]->print();
  }

  bool addConstraint(shared_ptr<ReachableVolume> _constraint){
    if(m_debug)cout<<"adding constraint"<<endl;
    bool addConstraint=true;
    bool changed = true;
    vector<shared_ptr<ReachableVolume> > *reachableVolumesJointNew = new vector<shared_ptr<ReachableVolume> >;
    for(vector<shared_ptr<ReachableVolume> >::iterator i = m_reachableVolumesJoint->begin(); i!=m_reachableVolumesJoint->end(); i++){
      if((*i)->m_base[0]==_constraint->m_base[0] && (*i)->m_base[1]==_constraint->m_base[1] && (*i)->m_base[2]==_constraint->m_base[2]){
	changed = (*i)->m_rmin < _constraint->m_rmin || (*i)->m_rmax > _constraint->m_rmax;
	_constraint->m_rmin=max(_constraint->m_rmin,(*i)->m_rmin);
        _constraint->m_rmax=min(_constraint->m_rmax,(*i)->m_rmax);
        continue;
      }

      if(_constraint->isEqual(**i)){
	reachableVolumesJointNew->push_back(*i);
	addConstraint = false;
	continue;
      }
      if(!(*i)->isContained(*_constraint))
	reachableVolumesJointNew->push_back(*i);
      if(_constraint->isContained(**i)){
	addConstraint = false;
      }
    }
    if(m_debug) cout<<"after add constraints"<<endl;

    m_reachableVolumesJoint=shared_ptr<vector<shared_ptr<ReachableVolume> > >(reachableVolumesJointNew);
    if(addConstraint){
      if(m_debug) cout<<"New Constraint Added"<<endl;
      m_reachableVolumesJoint->push_back(_constraint);
    }
    if(m_debug) cout<<"return add constraints"<<endl;
    return addConstraint && changed;
  }

  bool addConstraint(vector<shared_ptr<ReachableVolume> > &_constraint){
    if(m_debug)cout<<"adding constraint vector"<<endl;
    bool changed = false;
    for(vector<shared_ptr<ReachableVolume> >::iterator i = _constraint.begin(); i!= _constraint.end(); i++){
      changed = changed || addConstraint(*i);
    }
    return changed;
  }

  bool addConstraint(shared_ptr<ReachableVolumeJoint> &_rvj){
    return addConstraint(_rvj->m_reachableVolumesJoint);
  }

  bool addConstraint(shared_ptr<vector<shared_ptr<ReachableVolume> > > &_constraint){
    return addConstraint(*_constraint);
  }

  //returns cosine angle at p2
  static inline double CosineAngle(Vector3d &p1, Vector3d &p2, Vector3d &p3){
    return CosineAngle(ReachableVolume::distance(p1,p2),ReachableVolume::distance(p2,p3),ReachableVolume::distance(p1,p3));
  }

  //returns angle at point p2 of triangle p1,p2,p3 in radians
  static inline double CosineAngle(double dist_p1_p2, double dist_p2_p3, double dist_p1_p3){
    return acos((pow(dist_p1_p2,2)+pow(dist_p2_p3,2)-pow(dist_p1_p3,2))/(2*dist_p1_p2*dist_p2_p3));
  }

  inline bool IsInReachableVolume(const Vector3d &_p){
    for(vector<shared_ptr<ReachableVolume> >::iterator i = m_reachableVolumesJoint->begin(); i!=m_reachableVolumesJoint->end(); i++){
      if(!(*i)->IsInReachableVolumeBase(_p))
	return false;
    }
    return true;
  }

  //returns true if point is within eps of being in all reachable volume objects
  //Warning, this is not the same as saying it is in epsilon of the reachable volume represented by the intersection of these objects
  inline bool IsInEpsOfReachableVolume(const Vector3d &_p, double _eps){
    for(vector<shared_ptr<ReachableVolume> >::iterator i = m_reachableVolumesJoint->begin(); i!=m_reachableVolumesJoint->end(); i++){
      if(!(*i)->IsInEpsOfReachableVolumeBase(_p, _eps))
	return false;
    }
    return true;
  }

 //(x,y)=point in plane
 //rotate the plane by rotateAngle
 static inline Vector2d rotateCoordinatePlane(double x, double y, double rotateAngle){
   double r = sqrt(pow(x,2)+pow(y,2));
   double theta = atan2(y,x);
   Vector2d rotatedPoint;
   rotatedPoint[0]=r*cos(theta+rotateAngle);
   rotatedPoint[1]=r*sin(theta+rotateAngle);
   return rotatedPoint;
 }

  //rotates coordinate frame about origin by given inclination and rotation
  //takes a point p in old coordinate frame
  //returns the point p in the new coordinate frame
  static inline Vector3d rotateCoordinateFrame(Vector3d _p, double _inclination, double _rotation){
    Vector2d tmp = rotateCoordinatePlane(_p[0], _p[2], _inclination);
    _p[0] = tmp[0];
    _p[2] = tmp[1];
    tmp = rotateCoordinatePlane(_p[0], _p[1], _rotation);
    _p[0] = tmp[0];
    _p[1] = tmp[1];
    return _p;
  }


  //******************************************************************
  //Methods for obtaining point in intersection of reachable volume
  //finds a point on the intersection of rmax of the 2 samples
  //used when rmin=rmax for both points
  Vector3d intersection_rmax(shared_ptr<ReachableVolume> &_leftNode, shared_ptr<ReachableVolume> &_rightNode, double _nAttempts){
    if(m_debug){
      cout<<"ljoint ="<<_leftNode->m_base<<endl;
      cout<<"rjoint ="<<_rightNode->m_base<<endl;
    }
    double l1 = _leftNode->m_rmax;
    double l2 = _rightNode->m_rmax;
    double l3 = _leftNode->distance(_leftNode->m_base, _rightNode->m_base);
    double theta = CosineAngle(l1,l3,l2);
    double psi = (float)rand()/(float)RAND_MAX * 2 * PI;
    if(m_debug)cout<<"intersection_rmax paremeters"<<l1<<", "<<l2<<", "<<l3<<", "<<theta<<", "<<psi<<endl;
    Vector3d pointOnIntersection;

    double inclination = asin((_rightNode->m_base[2]-_leftNode->m_base[2])/l3);
    double rotation = atan2(_rightNode->m_base[1]-_leftNode->m_base[1],_rightNode->m_base[0]-_leftNode->m_base[0]);


    pointOnIntersection[0]=l1*cos(theta);
    pointOnIntersection[1]=l1*sin(theta)*sin(psi);
    pointOnIntersection[2]=l1*sin(theta)*cos(psi);
    Vector3d pointToAdd;
    Vector2d tmp=rotateCoordinatePlane(pointOnIntersection[0],pointOnIntersection[2],inclination);
    pointOnIntersection[2]=tmp[1];
    pointOnIntersection[0]=tmp[0];
    tmp=rotateCoordinatePlane(pointOnIntersection[0],pointOnIntersection[1],rotation);
    pointOnIntersection[1]=tmp[1];
    pointOnIntersection[0]=tmp[0];
    return pointOnIntersection+_leftNode->m_base;
  }

  //generates a samples in one nodes reachable volume then tests if it is in the reachable volume of the other node
  //useful if overlap between reachable volumes is large
  Vector3d brute_force(shared_ptr<ReachableVolume> &_leftNode,  shared_ptr<ReachableVolume> &_rightNode, double _nAttempts){
    for(int k = 0;  k < _nAttempts; k++){
      Vector3d jointPos = _leftNode->GetPointInReachableVolume()+_leftNode->m_base;
      Vector3d tmp = jointPos - _rightNode->m_base;
      if(_rightNode->IsInReachableVolume(tmp))
	return jointPos;

      //try the other way
      jointPos = _rightNode->GetPointInReachableVolume()+_rightNode->m_base;
      tmp = jointPos - _leftNode->m_base;
      if(_leftNode->IsInReachableVolume(tmp))
        return jointPos;
    }
    if(m_debug) cout<<"failed to find cfg(1)"<<endl;
    Vector3d revurnValue;
    revurnValue[0]=-1;
    revurnValue[1]=-1;
    revurnValue[2]=-1;
    return revurnValue;
  }


  //computes bounding cube of intersection of reachable volume
  //generates points in bounding cube until it finds a point in the reachable volume of both samples
  Vector3d bounding_cube(shared_ptr<ReachableVolume> &_leftNode, shared_ptr<ReachableVolume> &_rightNode, double _nAttempts){
    //compute bounding cube of intersections of reachable volumes
    double d = _leftNode->distance(_leftNode->m_base, _rightNode->m_base);
    if(d==0)
      return brute_force(_leftNode, _rightNode, _nAttempts);
    double h;
    if(_leftNode->m_rmax >= d +_rightNode->m_rmax)
      h=_rightNode->m_rmax;
    else if(_rightNode->m_rmax >= d +_leftNode->m_rmax)
      h=_leftNode->m_rmax;
    else
      h = _leftNode->m_rmax * sin(CosineAngle(_leftNode->m_rmax, d, _rightNode->m_rmax));
    double left_bound = max(-_leftNode->m_rmax, d - _rightNode->m_rmax);
    double right_bound = min(_leftNode->m_rmax, d + _rightNode->m_rmax);

    for(int k = 0;  k < _nAttempts; k++){
      //generate sample in bounding cube (right_bound to left_bound X 2h X 2h)
      Vector3d pointInBoundingCube;
      pointInBoundingCube[0]=left_bound+(right_bound-left_bound)*(float)rand()/(float)RAND_MAX;
      pointInBoundingCube[1]=(2*h)*(float)rand()/(float)RAND_MAX - h;
      pointInBoundingCube[2]=(2*h)*(float)rand()/(float)RAND_MAX - h;

      double inclination = asin((_rightNode->m_base[2]-_leftNode->m_base[2])/d);
      double rotation = atan2(_rightNode->m_base[1]-_leftNode->m_base[1],_rightNode->m_base[0]-_leftNode->m_base[0]);
      Vector3d rotatedPointInBoundingCube = rotateCoordinateFrame(pointInBoundingCube, inclination, rotation);

      Vector3d jointPos = rotatedPointInBoundingCube + _leftNode->m_base;
      Vector3d jointLeftPointCoordFrame = rotatedPointInBoundingCube;
      Vector3d jointRightPointCoordFrame = jointPos-_rightNode->m_base;

      if(_leftNode->IsInReachableVolume(jointLeftPointCoordFrame) && _rightNode->IsInReachableVolume(jointRightPointCoordFrame))
        return jointPos;
    }
    if(m_debug)cout<<"failed to find cfg(2)"<<endl;
    Vector3d revurnValue;
    revurnValue[0]=-1;
    revurnValue[1]=-1;
    revurnValue[2]=-1;
    //return NULL;
    return revurnValue;
  }

  Vector3d bounding_patch(shared_ptr<ReachableVolume> &_leftChild, shared_ptr<ReachableVolume> &_rightChild, const Vector3d &_leftJoint, const Vector3d &_rightJoint, double _nAttempts){
    Vector3d v;
    double d = _leftChild->distance(_leftJoint, _rightJoint);
    double inclination = asin((_rightJoint[2]-_leftJoint[2])/d);
    double rotation = atan2(_rightJoint[1]-_leftJoint[1], _rightJoint[0]-_leftJoint[0]);
    double boundingAngle;
    if(_rightChild->m_rmax>=d + _leftChild->m_rmax)
      return _leftChild->GetPointInReachableVolume();
    else
      boundingAngle = CosineAngle(_leftChild->m_rmax, d, _rightChild->m_rmax);
    double r = _leftChild->m_rmax;
    for(int k = 0;  k < _nAttempts; k++){
      double psi = (float)rand()/(float)RAND_MAX * 2 * boundingAngle - boundingAngle + rotation;
      double theta= (float)rand()/(float)RAND_MAX * 2 * boundingAngle - boundingAngle + inclination;
      Vector3d point = _leftChild->convertRadialToEuclidean(r,psi,theta);
      Vector3d jointPos = point + _leftJoint;
      Vector3d jointRightPointCoordFrame = jointPos-_rightJoint;
      if(_rightChild->IsInReachableVolume(jointRightPointCoordFrame))
        return jointPos;
    }
    if(m_debug) cout<<"failed to find cfg(3)"<<endl;
    Vector3d revurnValue;
    revurnValue[0]=-1;
    revurnValue[1]=-1;
    revurnValue[2]=-1;
    //return NULL;
    return revurnValue;

  }

  //constructs an axis aligned bb around the intersection of all constraints
  //generates joint position by randomly sampling in this bb
  Vector3d sampleAABoundingBox(int m_nAttempts){
    Vector3d urCorner;
    Vector3d llCorner;
    vector<shared_ptr<ReachableVolume> >::iterator i=m_reachableVolumesJoint->begin();
    urCorner[0]=(*i)->m_base[0]+(*i)->m_rmax;
    urCorner[1]=(*i)->m_base[1]+(*i)->m_rmax;
    urCorner[2]=(*i)->m_base[2]+(*i)->m_rmax;
    llCorner[0]=(*i)->m_base[0]-(*i)->m_rmax;
    llCorner[1]=(*i)->m_base[1]-(*i)->m_rmax;
    llCorner[2]=(*i)->m_base[2]-(*i)->m_rmax;
    i++;
    while(i!= m_reachableVolumesJoint->end()){
      urCorner[0]=min((*i)->m_base[0]+(*i)->m_rmax,urCorner[0]);
      urCorner[1]=min((*i)->m_base[1]+(*i)->m_rmax,urCorner[1]);
      urCorner[2]=min((*i)->m_base[2]+(*i)->m_rmax,urCorner[2]);
      llCorner[0]=max((*i)->m_base[0]-(*i)->m_rmax,llCorner[0]);
      llCorner[1]=max((*i)->m_base[1]-(*i)->m_rmax,llCorner[1]);
      llCorner[2]=max((*i)->m_base[2]-(*i)->m_rmax,llCorner[2]);
      i++;
    }
    if(urCorner[0]<llCorner[0] || urCorner[1]<llCorner[1]||urCorner[2]<llCorner[2]){
      Vector3d revurnValue;
      revurnValue[0]=-1;
      revurnValue[1]=-1;
      revurnValue[2]=-1;
      //return NULL;
      return revurnValue;
    }
    for(int i=0; i<m_nAttempts; i++){
      Vector3d v;
      v[0]=llCorner[0]+(float)rand()/(float)RAND_MAX*(urCorner[0]-llCorner[0]);
      v[1]=llCorner[1]+(float)rand()/(float)RAND_MAX*(urCorner[1]-llCorner[1]);
      v[2]=llCorner[2]+(float)rand()/(float)RAND_MAX*(urCorner[2]-llCorner[2]);
      bool inReachableVolumes=true;
      for(vector<shared_ptr<ReachableVolume> >::iterator iter=m_reachableVolumesJoint->begin(); iter!=m_reachableVolumesJoint->end(); iter++){
	if(!(*iter)->IsInReachableVolume(v)){
	  inReachableVolumes=false;
	  break;
	}
      }
      if(inReachableVolumes)
	return v;
    }
    Vector3d revurnValue;
    revurnValue[0]=-1;
    revurnValue[1]=-1;
    revurnValue[2]=-1;
    //return NULL;
    return revurnValue;
  }

  //gets a node that is in the reachable volume of the joint
  Vector3d getJointPos(int m_nAttempts){
    Vector3d jointPos;
    if(m_reachableVolumesJoint->size()==0){
      //return NULL;
      Vector3d revurnValue;
      revurnValue[0]=-1;
      revurnValue[1]=-1;
      revurnValue[2]=-1;
      //return NULL;
      return revurnValue;
    }

    //check for single point constraints
    for(vector<shared_ptr<ReachableVolume> >::iterator i = m_reachableVolumesJoint->begin(); i!=m_reachableVolumesJoint->end();i++){
      if((*i)->m_rmax==0){
	for(vector<shared_ptr<ReachableVolume> >::iterator j = m_reachableVolumesJoint->begin(); j!=m_reachableVolumesJoint->end();j++){
	  if(!(*j)->IsInReachableVolumeBase((*i)->m_base)){
	    //return NULL;
	    Vector3d revurnValue;
	    revurnValue[0]=-1;
	    revurnValue[1]=-1;
	    revurnValue[2]=-1;
	    //return NULL;
	    return revurnValue;
	  }
	}
	return (*i)->m_base;
      }
    }
    if(m_reachableVolumesJoint->size()==1){
      if(m_debug) cout<<"Encountered joint with 1 constraint"<<endl;
      return (*m_reachableVolumesJoint)[0]->GetPointInReachableVolume()+(*m_reachableVolumesJoint)[0]->m_base;
    }
    if(m_reachableVolumesJoint->size()==2){
      if(m_debug) cout<<"Encountered joint with 2 constraints"<<endl;
      if((*m_reachableVolumesJoint)[0]->m_rmin ==(*m_reachableVolumesJoint)[0]->m_rmax && (*m_reachableVolumesJoint)[1]->m_rmin == (*m_reachableVolumesJoint)[1]->m_rmax){
	jointPos = intersection_rmax((*m_reachableVolumesJoint)[1], (*m_reachableVolumesJoint)[0], m_nAttempts);
      }else if((*m_reachableVolumesJoint)[0]->m_rmin ==(*m_reachableVolumesJoint)[0]->m_rmax){
	jointPos = bounding_patch((*m_reachableVolumesJoint)[0], (*m_reachableVolumesJoint)[1], (*m_reachableVolumesJoint)[0]->m_base, (*m_reachableVolumesJoint)[1]->m_base, m_nAttempts);
      }else if((*m_reachableVolumesJoint)[1]->m_rmin ==(*m_reachableVolumesJoint)[1]->m_rmax){
	jointPos = bounding_patch((*m_reachableVolumesJoint)[1], (*m_reachableVolumesJoint)[0], (*m_reachableVolumesJoint)[1]->m_base, (*m_reachableVolumesJoint)[0]->m_base, m_nAttempts);
      }else{
	jointPos = bounding_cube((*m_reachableVolumesJoint)[0], (*m_reachableVolumesJoint)[1], m_nAttempts);
      }
      return jointPos;
    }
    if(m_debug) cout<<"Encountered joint with 3 or more constraints"<<endl;
    //More than 2 constraints
    //Construct a bounding cube around the constraints
    //Sample within bounding box
    vector<shared_ptr<ReachableVolume> > constsMinEqMax;
    for(vector<shared_ptr<ReachableVolume> >::iterator i = m_reachableVolumesJoint->begin(); i!=m_reachableVolumesJoint->end();i++){
      if((*i)->m_rmin==(*i)->m_rmin)
	constsMinEqMax.push_back(*i);
    }
    if(constsMinEqMax.size()==0){
      return sampleAABoundingBox(m_nAttempts);
    }else if(constsMinEqMax.size()==1){
      for(int j=0; j<m_nAttempts;j++){
	int rNum=(int)rand()%m_reachableVolumesJoint->size();
	jointPos = bounding_patch(constsMinEqMax[0], (*m_reachableVolumesJoint)[rNum], constsMinEqMax[0]->m_base, (*m_reachableVolumesJoint)[rNum]->m_base, m_nAttempts);
	bool success=true;
	for(vector<shared_ptr<ReachableVolume> >::iterator i = m_reachableVolumesJoint->begin(); i!=m_reachableVolumesJoint->end();i++){
	  Vector3d tmp = jointPos-(*i)->m_base;
	  if(!(*i)->IsInReachableVolume(tmp)){
	    success=false;
	    break;
	  }
	}
	if(success)
	  return jointPos;
      }
      Vector3d revurnValue;
      revurnValue[0]=-1;
      revurnValue[1]=-1;
      revurnValue[2]=-1;
      //return NULL;
      return revurnValue;
    }else{
      for(int j=0; j<m_nAttempts;j++){
	jointPos = bounding_patch(constsMinEqMax[0], constsMinEqMax[1], constsMinEqMax[0]->m_base, constsMinEqMax[1]->m_base, m_nAttempts);
	bool success=true;
        for(vector<shared_ptr<ReachableVolume> >::iterator i = m_reachableVolumesJoint->begin(); i!=m_reachableVolumesJoint->end();i++){
	  Vector3d tmp = jointPos-(*i)->m_base;
          if(!(*i)->IsInReachableVolume(tmp)){
            success=false;
            break;
          }
        }
        if(success)
          return jointPos;
      }
    }
    Vector3d revurnValue;
    revurnValue[0]=-1;
    revurnValue[1]=-1;
    revurnValue[2]=-1;
    //return NULL;
    return revurnValue;
  }

  inline Vector3d getNearestPointInRV(Vector3d _p, int _nAttempts=100){
    bool success=false;
    Vector3d nearest;
    double d_min;
    for(vector<shared_ptr<ReachableVolume> >::iterator i = m_reachableVolumesJoint->begin(); i<m_reachableVolumesJoint->end(); i++){
      Vector3d tmp = (*i)->getNearestPointInRV(_p);
      double d = ReachableVolume::distance(_p,tmp);
      if(!success || d<d_min){
	bool inRv=true;
	for(vector<shared_ptr<ReachableVolume> >::iterator j = m_reachableVolumesJoint->begin(); j<m_reachableVolumesJoint->end(); j++){
	  if(i!=j){
	    if(!(*j)->IsInReachableVolume(tmp)){
	      inRv=false;
	    }
	  }
	}
	if(inRv){
	  d_min=d;
	  nearest=tmp;
	  success=true;
	}
      }
    }
    if(success)
      return nearest;
    return getJointPos(_nAttempts);
  }

};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class ReachableVolumeJointTreeNode : public ReachableVolumeJoint {
 public:
  ReachableVolumeJointTreeNode *m_parent;
  vector<shared_ptr<ReachableVolumeJointTreeNode> > m_children;
  shared_ptr<ReachableVolume> m_ReachableVolumeToParent;


  ReachableVolumeJointTreeNode()
    : ReachableVolumeJoint(){
  }

  ReachableVolumeJointTreeNode(double _rmin, double _rmax, int _nLinks=1)
    : ReachableVolumeJoint(_rmax, _rmin, _nLinks){
    m_parent=NULL;
  }

  ReachableVolumeJointTreeNode(double _linkLength, int _nLinks=1)
    : ReachableVolumeJoint(_linkLength,_linkLength,_nLinks){
  }

  ReachableVolumeJointTreeNode(ReachableVolumeJointTreeNode &_r1, ReachableVolumeJointTreeNode &_r2){
    ReachableVolumeJointTreeNode(&_r1, &_r2);
  }

  ReachableVolumeJointTreeNode(shared_ptr<ReachableVolumeJointTreeNode> _r1, shared_ptr<ReachableVolumeJointTreeNode> _r2){
    ReachableVolumeJointTreeNode(_r1.get(), _r2.get());
  }


  //Computes Minkowski sum of reachable volumes r1 and r2
  //if r1 and r2 are the reachable volumes of 2 chains then this will be the reachable volume of the merged chain
  ReachableVolumeJointTreeNode(ReachableVolumeJointTreeNode *_r1, ReachableVolumeJointTreeNode *_r2)
    : ReachableVolumeJoint(_r1, _r2){
    //everything done in ReachableVolumeJoint consructor
  }

  ReachableVolumeJointTreeNode* mkSum(double _min, double _max){
    ReachableVolumeJointTreeNode *rvj = new ReachableVolumeJointTreeNode();
    for(vector<shared_ptr<ReachableVolume> >::iterator i = this->m_reachableVolumesJoint->begin(); i<this->m_reachableVolumesJoint->end(); i++){
      double maxForTmp = _max + (*i)->m_rmax;
      double minForTmp;
      if(_min > (*i)->m_rmin){
	minForTmp = max((double)0,_min - (*i)->m_rmax);
      }else{
	minForTmp = max((double)0,(*i)->m_rmin - _max);
      }
      rvj->m_reachableVolumesJoint->push_back(shared_ptr<ReachableVolume>(new ReachableVolume(minForTmp, maxForTmp)));
      rvj->m_reachableVolumesJoint->back()->m_base=(*i)->m_base;
    }
    rvj->m_nLinks=this->m_nLinks;
    return rvj;
  }

};

#endif
