#include "SpheresCollisionDetection.h"
#include "Utilities/MetricUtils.h"
#include "MPProblem/Geometry/MultiBody.h"

BoundingSpheres::BoundingSpheres() : CollisionDetectionMethod() {
  m_name = "boundingSpheres";
  m_type = Out;
  m_cdType = BOUNDING_SPHERES;
}


BoundingSpheres::~BoundingSpheres() {
}

bool
BoundingSpheres::IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle,
    StatClass& _stats, CDInfo& _cdInfo, const string& _callName, int _ignoreIAdjacentMultibodies) {
  _stats.IncNumCollDetCalls(GetName(), _callName );

  Vector3d robotCom = _robot->GetCenterOfMass();
  Vector3d obstCom  = _obstacle->GetCenterOfMass();

  if(_robot->GetFreeBodyCount())
    robotCom = _robot->GetFreeBody(0)->GetWorldTransformation() * robotCom;
  if(_obstacle->GetFreeBodyCount())
    obstCom  = _obstacle->GetFreeBody(0)->GetWorldTransformation() * obstCom;

  double robotRadius = _robot->GetBoundingSphereRadius();
  double obstRadius  = _obstacle->GetBoundingSphereRadius();

  double dist = sqrt(sqr(robotCom[0] - obstCom[0]) +
      sqr(robotCom[1] - obstCom[1]) +
      sqr(robotCom[2] - obstCom[2]));

  return dist <= robotRadius + obstRadius;
}


//////////


InsideSpheres::InsideSpheres() : CollisionDetectionMethod() {
  m_name = "insideSpheres";
  m_type = In;
  m_cdType = INSIDE_SPHERES;
}


InsideSpheres::~InsideSpheres() {
}

bool
InsideSpheres::IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle,
    StatClass& _stats, CDInfo& _cdInfo, const string& _callName, int _ignoreIAdjacentMultibodies) {
  _stats.IncNumCollDetCalls(GetName(),_callName );

  Vector3d robotCom = _robot->GetCenterOfMass();
  Vector3d obstCom  = _obstacle->GetCenterOfMass();

  if(_robot->GetFreeBodyCount())
    robotCom = _robot->GetFreeBody(0)->GetWorldTransformation() * robotCom;
  if(_obstacle->GetFreeBodyCount())
    obstCom  = _obstacle->GetFreeBody(0)->GetWorldTransformation() * obstCom;

  double robotRadius = _robot->GetInsideSphereRadius();
  double obstRadius  = _obstacle->GetInsideSphereRadius();

  double dist = sqrt(sqr(robotCom[0] - obstCom[0]) +
      sqr(robotCom[1] - obstCom[1]) +
      sqr(robotCom[2] - obstCom[2]));

  return dist <= robotRadius + obstRadius;
}

