#include "VClipCollisionDetection.h"
#include "Metrics.h"

#ifdef USE_VCLIP
VClip::
VClip() : CollisionDetectionMethod() {
  m_name = "VCLIP";
  m_type = Exact;
  m_cdtype = VCLIP;
}

VClip::
~VClip() {
}

ClosestFeaturesHT closestFeaturesHT(3000);

bool
VClip::
IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
	      StatClass& _stats, CDInfo& _cdInfo, std::string *_callName, int _ignoreIAdjacentMultibodies) {
  _stats.IncNumCollDetCalls(GetName(), _callName);

  Real dist;
  VclipPose X12;
  Vect3 cp1, cp2;   // closest points between bodies, in local frame
  // we're throwing this info away for now
  
  if (_cdInfo.m_retAllInfo == true) {
    bool ret_val;
    ret_val = IsInColl_AllInfo_vclip(_robot, _obstacle, _cdInfo, _ignoreIAdjacentMultibodies);
    return ret_val;
  }
  
  for(int i=0 ; i<_robot->GetFreeBodyCount(); i++) {
    
    shared_ptr<PolyTree> rob = _robot->GetFreeBody(i)->GetVClipBody();
    
    for(int j=0; j<_obstacle->GetBodyCount(); j++) {
      
      // if robot check self collision, skip adjacent links.
      if(_robot == _obstacle &&
	 _robot->GetFreeBody(i)->isWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) ) {
	continue;
      }
      
      shared_ptr<PolyTree> obst = _obstacle->GetBody(j)->GetVClipBody();
      X12 = GetVClipPose(_robot->GetFreeBody(i)->WorldTransformation(),
			 _obstacle->GetBody(j)->WorldTransformation());
      dist = PolyTree::vclip(rob.get(),obst.get(),X12,closestFeaturesHT, cp1, cp2);
      
      if(dist <= 0.0) { // once was < 0.001 ????
	return true;
      }
    } // end for j
  } // end for i
  
  return false;
} // end IsInCollision_vclip()


//////////////////////////////////////////////////////////////////////////
// IsInColl_AllInfo_vclip
// written by Brent, June 2000
//
// This function will fill in as much of _cdInfo as possible
// w.r.t. the robot and obstacle sent
// Notice each obstacle could change the results in _cdInfo
// Trace back to general IsInCollision call to see how it all
// gets updated correctly.
//////////////////////////////////////////////////////////////////////////
bool
VClip::
IsInColl_AllInfo_vclip(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
		       CDInfo& _cdInfo, int _ignoreIAdjacentMultibodies) {
  Real dist, m_minDist_so_far;
  VclipPose X12;
  Vect3 cp1, cp2;   // closest points between bodies, in local frame
  Vector3D robot_pt, obs_pt;
  bool ret_val;
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;
  
  ret_val = false;
  m_minDist_so_far = MaxDist;  // =  1e10 by CollisionDetection.h
  
  for(int i=0; i<_robot->GetFreeBodyCount(); i++) {
    shared_ptr<PolyTree> rob = _robot->GetFreeBody(i)->GetVClipBody();
    
    for(int j=0; j<_obstacle->GetBodyCount(); j++) {
      
      // if robot check self collision, skip adjacent links.
      if(_robot == _obstacle &&
	 _robot->GetFreeBody(i)->isWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) ) {   
	continue;
      }
      
      shared_ptr<PolyTree> obst = _obstacle->GetBody(j)->GetVClipBody();
      X12 = GetVClipPose(_robot->GetFreeBody(i)->WorldTransformation(),
			 _obstacle->GetBody(j)->WorldTransformation());
      dist = PolyTree::vclip(rob.get(),obst.get(),X12,closestFeaturesHT, cp1, cp2);

      if ( dist <= 0.0 ) {
	if (dist < m_minDist_so_far)
	  _cdInfo.m_collidingObstIndex = j;
	ret_val = true;
      }
      
      if (dist < m_minDist_so_far) {
	m_minDist_so_far = dist;
        _cdInfo.m_nearestObstIndex = j;
	_cdInfo.m_minDist = dist;
	
	// change a 3 elmt array to Vector3D class
	robot_pt[0] = cp1[0];
	robot_pt[1] = cp1[1];
	robot_pt[2] = cp1[2];
	
	obs_pt[0] = cp2[0];
	obs_pt[1] = cp2[1];
	obs_pt[2] = cp2[2];
	
	//cout << "CD method, robot pt = " << robot_pt << endl;
	//cout << "CD method, obs_pt = " << obs_pt << endl;
	
	// transform points to world coords
	// using *_pt vars in case overloaded * was not done well.
	_cdInfo.m_robotPoint = _robot->GetFreeBody(i)->WorldTransformation() * robot_pt;
	_cdInfo.m_objectPoint = _obstacle->GetBody(j)->WorldTransformation() * obs_pt;
	
      }
    } // end for j
  } // end for i
  
  return ret_val;
} // end IsInColl_AllInfo_vclip()


VclipPose 
VClip::
GetVClipPose(const Transformation &myT, const Transformation &obstT) {	
  Transformation diff = Transformation(obstT).Inverse() * myT;
  
  diff.m_orientation.ConvertType(Orientation::EulerXYZ);
  
  //------------------------------------------------
  // here's where it really starts.
  //------------------------------------------------
  
  Vect3 XYZ(diff.m_position.getX(),diff.m_position.getY(),diff.m_position.getZ());
  
  Quat RPY         (diff.m_orientation.alpha,Vect3::I);
  RPY.postmult(Quat(diff.m_orientation.beta ,Vect3::J));
  RPY.postmult(Quat(diff.m_orientation.gamma,Vect3::K));
  
  // the above is for EulerXYZ.
  // For EulerZYX, or FixedXYZ, we should have the following instead,
  // i.e. Rotation = Rz(alpha) * Ry(beta) * Rx(gamma)
  // Quat RPY         (diff.m_orientation.alpha,Vect3::K);
  // RPY.postmult(Quat(diff.m_orientation.beta ,Vect3::J));
  // RPY.postmult(Quat(diff.m_orientation.gamma,Vect3::I));
  
  return VclipPose(RPY,XYZ);
}
#endif

