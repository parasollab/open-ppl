#ifdef USE_VCLIP

#include "VClipCollisionDetection.h"
#include "Utilities/MetricUtils.h"
#include "MPProblem/Geometry/MultiBody.h"

VClip::VClip() : CollisionDetectionMethod("VCLIP", Exact, VCLIP) {}

VClip::~VClip() {}

ClosestFeaturesHT closestFeaturesHT(3000);

bool
VClip::IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
    StatClass& _stats, CDInfo& _cdInfo, string* _callName, int _ignoreIAdjacentMultibodies) {
  _stats.IncNumCollDetCalls(m_name, _callName);

  if (_cdInfo.m_retAllInfo == true)
    return FillCdInfo(_robot, _obstacle, _cdInfo, _ignoreIAdjacentMultibodies);

  Real dist;
  VClipPose x12; //VClip position
  Vect3 cp1, cp2;   // closest points between bodies, in local frame

  for(int i=0 ; i < _robot->GetFreeBodyCount(); i++) {

    shared_ptr<PolyTree> rob = _robot->GetFreeBody(i)->GetVClipBody();

    for(int j=0; j < _obstacle->GetBodyCount(); j++) {

      // if robot check self collision, skip adjacent links.
      if(_robot == _obstacle &&
          _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) ) {
        continue;
      }

      shared_ptr<PolyTree> obst = _obstacle->GetBody(j)->GetVClipBody();
      x12 = GetVClipPose(_robot->GetFreeBody(i)->WorldTransformation(),
          _obstacle->GetBody(j)->WorldTransformation());
      dist = PolyTree::vclip(rob.get(),obst.get(),x12,closestFeaturesHT, cp1, cp2);

      if(dist <= 0.0)
        return true;
    }
  }

  return false;
}


/* Get all collsion information for given MultiBody.
 * Collision is checked in Body level between two MultiBodys,
 * if any of Body from Robot collides with any of Body from obstacle,
 * true will be returned.
 *
 * More information about collision between two object, such as the closet points between
 * two object, closest distance... all of these information are stored in _cdInfo.
 *
 * each obstacle could change the results in _cdInfo
 * Trace back to general IsInCollision call to see how it all
 * gets updated correctly.
 */
bool
VClip::FillCdInfo(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
    CDInfo& _cdInfo, int _ignoreIAdjacentMultibodies) {
  Real dist, m_minCurrentDist;
  VClipPose x12;  
  Vect3 cp1, cp2; //closest points between bodies, in local frame
  
  bool isInCollision = false;
  
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  m_minCurrentDist = MaxDist;  // =  1e10 by CollisionDetection.h

  for(int i=0; i<_robot->GetFreeBodyCount(); i++) {
    shared_ptr<PolyTree> rob = _robot->GetFreeBody(i)->GetVClipBody();

    for(int j=0; j<_obstacle->GetBodyCount(); j++) {
      // if robot check self collision, skip adjacent links.
      if(_robot == _obstacle &&
          _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) ) {   
        continue;
      }

      shared_ptr<PolyTree> obst = _obstacle->GetBody(j)->GetVClipBody();
      x12 = GetVClipPose(_robot->GetFreeBody(i)->WorldTransformation(),
          _obstacle->GetBody(j)->WorldTransformation());
      dist = PolyTree::vclip(rob.get(),obst.get(),x12,closestFeaturesHT, cp1, cp2);

      if ( dist <= 0.0 ) {
        if (dist < m_minCurrentDist)
          _cdInfo.m_collidingObstIndex = j;
        isInCollision = true;
      }

      if (dist < m_minCurrentDist) {
        m_minCurrentDist = dist;
        _cdInfo.m_nearestObstIndex = j;
        _cdInfo.m_minDist = dist;

        // change a 3 elmt array to Vector3D class
        Vector3D robotPt(cp1[0], cp1[1], cp1[2]);
        Vector3D obstPt(cp2[0], cp2[1], cp2[2]);

        // transform points to world coords
        _cdInfo.m_robotPoint = _robot->GetFreeBody(i)->WorldTransformation() * robotPt;
        _cdInfo.m_objectPoint = _obstacle->GetBody(j)->WorldTransformation() * obstPt;
      }
    }
  }

  return isInCollision;
}

VClipPose
VClip::GetVClipPose(const Transformation& myT, const Transformation& obstT) {	
  Transformation diff = Transformation(obstT).Inverse() * myT;
  diff.m_orientation.ConvertType(Orientation::EulerXYZ);

  Vect3 XYZ(diff.m_position.getX(),diff.m_position.getY(),diff.m_position.getZ());

  Quat RPY(diff.m_orientation.alpha, Vect3::I);
  RPY.postmult(Quat(diff.m_orientation.beta, Vect3::J));
  RPY.postmult(Quat(diff.m_orientation.gamma, Vect3::K));

  return VClipPose(RPY, XYZ);
}

#endif
