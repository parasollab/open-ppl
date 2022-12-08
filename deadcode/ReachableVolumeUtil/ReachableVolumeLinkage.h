//This class represents the reachable volume of an entire chain.
#ifndef Reachable_Volume_Linkage_h
#define Reachable_Volume_Linkage_h

#include "ReachableVolumeJoint.h"
#include "deque"
#include "cmath"
/*needed by version on brazos
double abs(double n)
{
  return (n < 0 )? -n : n;
}
*/

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class ReachableVolumeLinkage{
 private:
  Vector3d m_linkageRoot;  //root of linkage
  int m_nAttempts;

 public:
  map<int,shared_ptr<ReachableVolumeJoint> > *m_RVSpaceJointConstraints;
  shared_ptr<ReachableVolumeJointTreeNode> m_root;  //root of tree of reahable volumes (which will contain the reachable volume of the entire chain
  int m_baseJointID;
  bool m_loop;
  shared_ptr<vector<pair<double, double> > > m_linkLengths;  //first = min linkage length, second = max link length
  static const bool m_debug = false;

  //_nAttempts is the number of samples attempted before it fails
  ReachableVolumeLinkage(const char* filename, map<int,shared_ptr<ReachableVolumeJoint> > *_RVSpaceJointConstraints, int _nAttempts = 10000, bool _loop=false){
    m_loop=_loop;
    m_nAttempts=_nAttempts;
    m_RVSpaceJointConstraints=_RVSpaceJointConstraints;
  }

  ReachableVolumeLinkage(vector<pair<double, double> >* _linkLengths, map<int,shared_ptr<ReachableVolumeJoint> > *_RVSpaceJointConstraints, int _nAttempts = 10000, bool _loop=false){
    m_loop=_loop;
    m_nAttempts=_nAttempts;
    m_linkLengths=shared_ptr<vector<pair<double, double> > >(_linkLengths);  //-------could be costly to copy, consider using prt
    m_RVSpaceJointConstraints=_RVSpaceJointConstraints;
  }




  //assumes that the first element of m_reachableVolumes has been set
  //computes tree of reachable volumes until we have one reachable volume that is the reachable volume of the entire linkage
  //stores tree of reachable volumes in m_reachableVolumes (where each element of m_reachableVolumes is one lever of the tree)
  void setReachableVolumes(shared_ptr<vector<pair<double, double> > > _linkLengths, vector<int> &_earParentLinks, vector<shared_ptr<ReachableVolumeLinkage> > &_RVLinkages, string _treeStructure = "EndEffectorFirst"){
    m_linkLengths = _linkLengths;
    setReachableVolumes(_earParentLinks, _RVLinkages, _treeStructure);
  }


  //returns all of the linkages that are rooted at the passed node
  static inline vector<shared_ptr<ReachableVolumeLinkage> > *getLinkagesRootedAt(vector<int> &_earParentLinks, vector<shared_ptr<ReachableVolumeLinkage> > &_RVLinkages, int _id){
    //cout<<"get linkages rooted at "<<_id<<endl;
    vector<shared_ptr<ReachableVolumeLinkage> > *linksRootedAt = new vector<shared_ptr<ReachableVolumeLinkage> >;
    for(unsigned int i=0; i<_earParentLinks.size(); i++){
      //cout<<"before if"<<endl;
      if(_earParentLinks[i]+1==_id){  //+1 because linkage branches from second joint of parent
	//cout<<"found linkage rooted at id "<<i+1<<endl;
        linksRootedAt->push_back(_RVLinkages[i+1]);//+1 because no parent for first linkage
      }
    }
    return linksRootedAt;
  }


  void setReachableVolumesBalanced(vector<int> &_earParentLinks, vector<shared_ptr<ReachableVolumeLinkage> > &_RVLinkages){
    vector<shared_ptr<ReachableVolumeJointTreeNode> > *treeLevel = new vector<shared_ptr<ReachableVolumeJointTreeNode> >;
    //set tree level to be top level of tree
    for(unsigned int i=0; i<m_linkLengths->size();i++){
      double first = (*m_linkLengths)[i].first;
      double second = (*m_linkLengths)[i].first;
      shared_ptr<ReachableVolumeJointTreeNode> rv = shared_ptr<ReachableVolumeJointTreeNode>(new ReachableVolumeJointTreeNode(first,second,1));
      rv->m_jointID=i+1;
      rv->m_parent=NULL;
      treeLevel->push_back(rv);
    }

    //merge nodes until we get reachable volume of entire tree
    while(treeLevel->size()>1){
      vector<shared_ptr<ReachableVolumeJointTreeNode> > *nextLevel = new vector<shared_ptr<ReachableVolumeJointTreeNode> >;
      for (unsigned int i = 0; i<treeLevel->size()-1; i+=2){
        ReachableVolumeJointTreeNode *rvj=new ReachableVolumeJointTreeNode(*((*treeLevel)[i]), *((*treeLevel)[i+1]));
	ReachableVolume *rv = new ReachableVolume((*treeLevel)[i]->m_reachableVolumesJoint->front().get(), (*treeLevel)[i+1]->m_reachableVolumesJoint->front().get());
        rvj->m_reachableVolumesJoint->clear();
        rvj->m_reachableVolumesJoint->push_back(shared_ptr<ReachableVolume>(rv));
	rvj->m_jointID=((*treeLevel)[i+1])->m_jointID;
	rvj->m_children.push_back((*treeLevel)[i]);
        rvj->m_children.push_back((*treeLevel)[i+1]);
	(*treeLevel)[i]->m_parent=rvj;
        (*treeLevel)[i+1]->m_parent=rvj;
	(*treeLevel)[i+1]->m_ReachableVolumeToParent=shared_ptr<ReachableVolume>(new ReachableVolume(0,0,0));
        (*treeLevel)[i]->m_ReachableVolumeToParent=(*treeLevel)[i]->m_reachableVolumesJoint->front();
	rvj->m_jointID=(*treeLevel)[i+1]->m_jointID;
	(*treeLevel)[i+1]->m_jointID=-1;

        nextLevel->push_back(shared_ptr<ReachableVolumeJointTreeNode>(rvj));
        //still need to address linkages rooted at joint
        //current setup won't work for trees
      }
      treeLevel=nextLevel;
    }
    m_root=treeLevel->front();
    m_root->m_parent=NULL;
  }


  void setReachableVolumes(vector<int> &_earParentLinks, vector<shared_ptr<ReachableVolumeLinkage> > &_RVLinkages, string _treeStructure = "EndEffectorFirst"){
    if(_treeStructure.compare("EndEffectorFirst")==0){
      if(m_debug)cout<<"using tree structure EndEffectorFirst"<<endl;
      setReachableVolumesLinear(_earParentLinks, _RVLinkages, true);
    }else if(_treeStructure.compare("RootFirst")==0){
      if(m_debug)cout<<"using tree structure RootFirst"<<endl;
      setReachableVolumesLinear(_earParentLinks, _RVLinkages, false);
    }else if(_treeStructure.compare("Balanced")==0){
      if(m_debug)cout<<"using tree structure balanced"<<endl;
      setReachableVolumesBalanced(_earParentLinks, _RVLinkages);
    }else{
      cout<<"Tree sturcture = "<<_treeStructure<<" which is not a specified option.  Defaulting to option EndEffectorFirst"<<endl;
      setReachableVolumesLinear(_earParentLinks, _RVLinkages, true);
    }
  }



  //since the root of the rv tree is the last node forward results in an rv tree that is rooted at the end effector while backwards results in an rv tree rooted and root of linkage
  //in forward variation the rvj for contains the volume of space that this node can reach while conforming to the constraints on that joint and any joint before it in the chain
  //the constraints from joints after that node do not need to be consitered because these joints will be sampled first
  //once the node after has been sampled we add the constraint introduced by that sample
  //all reachable volumes are in the rv space of the linkage with root of linkage at (0,0,0)
  void setReachableVolumesLinear(vector<int> &_earParentLinks, vector<shared_ptr<ReachableVolumeLinkage> > &_RVLinkages, bool _forward){

    int size=m_linkLengths->size();
    if(m_loop)
      size--;

   //create top level of tree by creating a reachable volume for each link in m_linkageLengths

    ReachableVolumeJointTreeNode* current;
    int nIter;
    if(_forward){
      current = new ReachableVolumeJointTreeNode();
      current->m_ReachableVolumeToParent=shared_ptr<ReachableVolume>(new ReachableVolume(0, 0,1));
      current->m_reachableVolumesJoint->push_back(shared_ptr<ReachableVolume>(new ReachableVolume(0, 0,1)));
      current->m_jointID=m_baseJointID-1;
      nIter=size;
    }else{
      current=new ReachableVolumeJointTreeNode;
      current->m_ReachableVolumeToParent=shared_ptr<ReachableVolume>(new ReachableVolume((*m_linkLengths)[size-1].first, (*m_linkLengths)[size-1].second,1));
      current->m_jointID=size+m_baseJointID-1;
      map<int, shared_ptr<ReachableVolumeJoint> >::iterator iter=m_RVSpaceJointConstraints->find(current->m_jointID);
      if(iter!=m_RVSpaceJointConstraints->end())
	current->m_reachableVolumesJoint=iter->second->m_reachableVolumesJoint;
      if(m_loop){
        current->addConstraint(shared_ptr<ReachableVolume>(new ReachableVolume(m_linkLengths->back().first, m_linkLengths->back().second)));
      }
      nIter=size-1;
    }

    //find any links rooted and current and set their reachable volumes
    vector<shared_ptr<ReachableVolumeLinkage> > *childLinks = getLinkagesRootedAt(_earParentLinks, _RVLinkages, current->m_jointID);
    for(vector<shared_ptr<ReachableVolumeLinkage> >::iterator iter = childLinks->begin(); iter!=childLinks->end(); iter++){
      (*iter)->setReachableVolumesLinear(_earParentLinks, _RVLinkages,false);
      (*iter)->m_root->m_parent=current;
      current->addConstraint((*iter)->m_root->mkSum((*iter)->m_linkLengths->front().first, (*iter)->m_linkLengths->front().second)->m_reachableVolumesJoint);
      current->m_children.push_back((*iter)->m_root);
    }
    delete childLinks;

    //create reachable volumes for linkage
      for(int i=1; i<=nIter; i++){
	int jointID,linkID;
	if(_forward){
	  jointID=i;
	  linkID=i-1;
	}else{
	  jointID=size-i;
	  linkID=size-i;
	}

	ReachableVolumeJointTreeNode *tmp = new ReachableVolumeJointTreeNode();

	//add any problems specified joint constraints for joint (constraints set in .tree file)
	map<int, shared_ptr<ReachableVolumeJoint> >::iterator iter=m_RVSpaceJointConstraints->find(jointID);
	if(iter!=m_RVSpaceJointConstraints->end()){
	  tmp->m_reachableVolumesJoint=iter->second->m_reachableVolumesJoint;
	}

	//add constraint from parent (i.e. mk sum of reachable volume of parent and reachable volume of link connecting joint to parent)
	tmp->addConstraint(current->mkSum((*m_linkLengths)[linkID].first, (*m_linkLengths)[linkID].second)->m_reachableVolumesJoint);  				    current->m_parent=tmp;    //parent = joint from last itteration

	//find any links rooted and current and set their reachable volumes
	vector<shared_ptr<ReachableVolumeLinkage> > *childLinks = getLinkagesRootedAt(_earParentLinks, _RVLinkages, jointID+m_baseJointID-1);
	for(vector<shared_ptr<ReachableVolumeLinkage> >::iterator iter = childLinks->begin(); iter!=childLinks->end(); iter++){
	  (*iter)->setReachableVolumesLinear(_earParentLinks, _RVLinkages,false);
          (*iter)->m_root->m_parent=tmp;
	  tmp->addConstraint((*iter)->m_root->mkSum((*iter)->m_linkLengths->front().first, (*iter)->m_linkLengths->front().second)->m_reachableVolumesJoint);
	  tmp->m_children.push_back((*iter)->m_root);
	}
	delete childLinks;
	tmp->m_ReachableVolumeToParent=shared_ptr<ReachableVolume>(new ReachableVolume((*m_linkLengths)[linkID].first, (*m_linkLengths)[linkID].second,1));
	tmp->m_jointID=jointID+m_baseJointID-1;
	//cout<<"Created rv for "<<tmp->m_jointID<<endl;
	shared_ptr<ReachableVolumeJointTreeNode> tmp_ptr =shared_ptr<ReachableVolumeJointTreeNode>(current);
        tmp->m_children.push_back(tmp_ptr);
	current=tmp;
	current->m_nLinks++;
      }
      m_root=shared_ptr<ReachableVolumeJointTreeNode>(current);
      if(m_loop&&_forward){
	m_root->addConstraint(shared_ptr<ReachableVolume>(new ReachableVolume(m_linkLengths->back().first, m_linkLengths->back().second)));
      }
  }








 //returns angle offset between line(p1,p2) and line(p3,p2) in radians
 //angles range from 0 to 2*pi
 inline static double getAngleSeperation(const Vector3d &p1, const Vector3d &p2, const Vector3d &p3, const Vector3d &positiveDir){
   double dotPro = dotProduct(p1-p2,p3-p2)/(ReachableVolume::distance(p1,p2)*ReachableVolume::distance(p3,p2));
   dotPro = min(max(dotPro,(double)-1),(double)1);
   double angle = acos(dotPro);
   if(ReachableVolumeLinkage::dotProduct(positiveDir, crossProduct(p1-p2,p3-p2)) < 0){
     angle = 2*PI - angle;
   }
   return angle;
 }

 //returns a unit vector in the same direction as v
 inline static Vector3d unitVector(const Vector3d &_v){
   Vector3d origin; //(0,0,0)
   return _v/ReachableVolume::distance(_v,origin);
 }

 inline static double dotProduct(const Vector3d &_v1, const Vector3d &_v2){
   return _v1[0]*_v2[0]+_v1[1]*_v2[1]+_v1[2]*_v2[2];
 }


 static inline Vector3d crossProduct(const Vector3d &_v1, const Vector3d &_v2){
   Vector3d cross_product;
   cross_product[0]=_v1[1]*_v2[2]-_v1[2]*_v2[1];
   cross_product[1]=_v1[2]*_v2[0]-_v1[0]*_v2[2];
   cross_product[2]=_v1[0]*_v2[1]-_v1[1]*_v2[0];
   return cross_product;
 }


 //rotates vector v about axis by angle theta
 static inline Vector3d rotateVectorAboutAxis(const Vector3d &_v, const Vector3d &_axis, double _theta){
  Vector3d r;
  Vector3d unit_axis = unitVector(_axis);
  r[0] = (cos(_theta)+pow(unit_axis[0],2)*(1-cos(_theta))) * _v[0]
    + (unit_axis[0]*unit_axis[1] * (1-cos(_theta)) - unit_axis[2] * sin (_theta)) * _v[1]
    + (unit_axis[0]*unit_axis[2] * (1-cos(_theta)) + unit_axis[1] * sin (_theta)) * _v[2];

  r[1] = (unit_axis[1]*unit_axis[0] * (1-cos(_theta)) + unit_axis[2] * sin (_theta)) * _v[0]
    + (cos(_theta)+pow(unit_axis[1],2)*(1-cos(_theta))) * _v[1]
    + (unit_axis[1]*unit_axis[2] * (1-cos(_theta)) - unit_axis[0] * sin (_theta)) * _v[2];

  r[2] = (unit_axis[2]*unit_axis[0] * (1-cos(_theta)) - unit_axis[1] * sin (_theta)) * _v[0]
    + (unit_axis[2]*unit_axis[1] * (1-cos(_theta)) + unit_axis[0] * sin (_theta)) * _v[1]
    + (cos(_theta)+pow(unit_axis[2],2)*(1-cos(_theta))) * _v[2];

  return r;
}


 //rotates _v by theta in plane contaning _v and _planeVector
 Vector3d rotateInPlaneOf(Vector3d _v, Vector3d _planeVector, double _theta){

  //find normal to _planeVector and _v, then rotate _v about that vector
  Vector3d normal = crossProduct(_v,_planeVector);
  //if(dotProduct((0,1,0),normal)>0)
  Vector3d y_vector; //(0,1,0)
  y_vector[1]=1;
  if(dotProduct(y_vector,normal)>0)
    _theta=-_theta;
  return rotateVectorAboutAxis(_v, normal,_theta);
 }


 static inline double getPsiAngleWRTUp_new(const Vector3d &_p1, const Vector3d &_p2, const Vector3d &_p3){
   //set origin to be at _p2
   Vector3d p1Trans=_p1-_p2;
   Vector3d p3Trans=_p3-_p2;

   //rotate so that _p1Trans is in plane y,z with positive z value
   double x=p1Trans[0];
   double z=p1Trans[2];
   double angleP1 = atan2(x,z);
   p1Trans[2]=sqrt(x*x+z*z);
   p1Trans[0]=0;
   double d_p3=sqrt(p3Trans[0]*p3Trans[0]+p3Trans[2]*p3Trans[2]);
   double angleP3 = atan2(p3Trans[0],p1Trans[2])-angleP1;
   p3Trans[2]=d_p3*cos(angleP3);
   p3Trans[0]=d_p3*sin(angleP3);

   //rotate so that _p1Trans is along z axis
   double y=p1Trans[1];
   z=p1Trans[2];
   angleP1 = atan2(y,z);
   d_p3=sqrt(p3Trans[1]*p3Trans[1]+p3Trans[2]*p3Trans[2]);
   angleP3 = atan2(p3Trans[1],p1Trans[2])-angleP1;
   p3Trans[2]=d_p3*cos(angleP3);
   p3Trans[1]=d_p3*sin(angleP3);
   double angleReturn=atan2(p3Trans[1],p1Trans[0]);
   while(angleReturn>2*PI)
     angleReturn-=2*PI;
   while(angleReturn<0)
     angleReturn+=2*PI;
   return angleReturn;
 }

 //Given three points
 //returns the psi angle at p2 with respect to up dir
 static inline double getPsiAngleWRTUp(const Vector3d &p1, const Vector3d &p2, const Vector3d &p3, double theta){
   double d = ReachableVolume::distance(p3,p2);
   Vector3d u = (p2-p1)/ReachableVolume::distance(p2,p1);
   Vector3d pCircle = u*d*cos(theta)+p2;  //Center of circle containing joint positions for p3 with articulated angle = theta
   Vector3d pTop; //point on circle in upward direction
   double dXY=sqrt(pow(p1[0]-p2[0],2)+pow(p1[1]-p2[1],2));
   if(dXY==0){
     pTop[2]=0;
     pTop[0]=1;
     pTop[1]=0;
   }else{
     pTop[2]=dXY/ReachableVolume::distance(p1,p2);
     double pTopXYPlane=abs(p1[2]-p2[2])/ReachableVolume::distance(p1,p2);
     pTop[0]=pTopXYPlane*abs(p1[0]-p2[0])/dXY;
     pTop[1]=pTopXYPlane*abs(p1[1]-p2[1])/dXY;
     if(p1[0]-p2[0]>0){
       pTop[0]=-pTop[0];
     }
     if(p1[1]-p2[1]>0){
       pTop[1]=-pTop[1];
     }
     if((p1[2]-p2[2])<0){
       pTop[0]=-pTop[0];
       pTop[1]=-pTop[1];
     }
   }
   pTop[0]+=pCircle[0];
   pTop[1]+=pCircle[1];
   pTop[2]+=pCircle[2];
   return getAngleSeperation(pTop,pCircle,p3,p2-p1);
 }


 //to be gotten rid of and replaced with above function
 shared_ptr<vector<double> > getInternalCFGCoordinates(shared_ptr<deque<Vector3d> > &rvSample, Vector3d &_referenceVector){
   rvSample->push_front(_referenceVector);
   return getInternalCFGCoordinatesHelper(rvSample);
 }

 //gets the joint angle coordinates of the cfg
 vector<double> *getInternalCFGCoordinatesHelper(const vector<Vector3d> &_rvSample){
   vector<double> *cfg_data = new vector<double>;
   double psi_not=0;
   for(vector<Vector3d>::const_iterator i = _rvSample.begin();  i+2 != _rvSample.end(); i++){
     //compute psi  + push back
     double seperationAngle = getAngleSeperation(*i, *(i+1), *(i+2),*(i+1)-*i);
     while(seperationAngle<0){
       seperationAngle+=2*PI;
     }
     if(seperationAngle>PI)
       seperationAngle=2*PI-seperationAngle;
     double theta = PI - seperationAngle;
     while(theta<0){
       theta+=2*PI;
     }
     while(theta>=2*PI){
       theta-=2*PI;
     }
     double angleWRToUp = getPsiAngleWRTUp_new(*i,*(i+1),*(i+2));
     double psi=angleWRToUp-psi_not;
     psi_not+=psi;
     while(psi<0){
       psi+=2*PI;
     }
     while(psi>=2*PI){
       psi-=2*PI;
     }
     cfg_data->push_back(theta/(2*PI));  //set theta angle for joint
     cfg_data->push_back(psi/(2*PI));
   }
   return cfg_data;
 }



 //to be gotten rid of and replaced with above function
 //gets the joint angle coordinates of the cfg
 shared_ptr<vector<double> > getInternalCFGCoordinatesHelper(shared_ptr<deque<Vector3d> > &rvSample){
   shared_ptr<vector<double> > cfg_data = shared_ptr<vector<double> >(new vector<double>);
   for(deque<Vector3d>::iterator i = rvSample->begin();  i+2 != rvSample->end(); i++){
     double seperationAngle = getAngleSeperation(*i, *(i+1), *(i+2),*(i+1)-*i);
     while(seperationAngle<0){
       seperationAngle+=2*PI;
     }
     if(seperationAngle>PI)
       seperationAngle=2*PI-seperationAngle;
     double theta = PI - seperationAngle;
     while(theta<0){
       theta+=2*PI;
     }
     while(theta>=2*PI){
       theta-=2*PI;
     }

     double angleWRToUp = getPsiAngleWRTUp(*i,*(i+1),*(i+2),theta);
     double psi;
     if(i==rvSample->begin())
       psi=angleWRToUp;
     else{
       Vector3d v1 = crossProduct(*(i-1) - *i, *(i+1) - *i);
       Vector3d v2 = crossProduct(*i - *(i+1), *(i+2) - *(i+1));
       Vector3d normal = *(i+1) - *i;;
       //psi=getAngleSeperation(v1,(0,0,0),v2,normal);
       Vector3d origin;//(0,0,0)
       psi=getAngleSeperation(v1,origin,v2,normal);
     }
     while(psi<0){
       psi+=2*PI;
     }
     while(psi>=2*PI){
       psi-=2*PI;
     }
     cfg_data->push_back(theta/(2*PI));  //set theta angle for joint
     cfg_data->push_back(psi/(2*PI));
   }
   return cfg_data;
 }


 //gets pitch of first link in linkage
 //note that root of linkage is at origin
 //value returned is in radians
 static double getPitch(const shared_ptr<deque<Vector3d> > &rvSample){
   Vector3d firstNode = (*rvSample)[1];
   double pitch = atan2(firstNode[2],
	                sqrt(pow(firstNode[0],2)+pow(firstNode[1],2)));
   while(pitch<0)
     pitch+=2*PI;
   return pitch;
 }

 //gets pitch of first link in linkage
 //note that root of linkage is at origin
 //value returned is in radians
 static double getPitch(const vector<Vector3d> &_rvSample){
   Vector3d firstNode = _rvSample[1];
   double pitch = atan2(firstNode[2],
	                sqrt(pow(firstNode[0],2)+pow(firstNode[1],2)));
   while(pitch<0)
     pitch+=2*PI;
   return pitch;
 }


//gets yaw of first link in robot
 //value retuned is in radians
 static double getYaw(const vector<Vector3d> &_rvSample){
   Vector3d firstNode =_rvSample[1];
   double yaw = atan2(firstNode[1],firstNode[0]);
   while(yaw<0)
     yaw +=2*PI;
   return yaw;
 }


 //gets yaw of first link in robot
 //value retuned is in radians
 static double getYaw(const shared_ptr<deque<Vector3d> > &rvSample){
   Vector3d firstNode =(*rvSample)[1];
   double yaw = atan2(firstNode[1],firstNode[0]);
   while(yaw<0)
     yaw +=2*PI;
   return yaw;
 }
};

#endif
