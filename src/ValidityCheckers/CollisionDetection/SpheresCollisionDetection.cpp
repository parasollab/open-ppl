#include "SpheresCollisionDetection.h"

#include "MPProblem/Geometry/ActiveMultiBody.h"
#include "MPProblem/Geometry/FreeBody.h"
#include "Utilities/MetricUtils.h"

BoundingSpheres::
BoundingSpheres() : CollisionDetectionMethod("BoundingSpheres", CDType::Out) {
  }

bool
BoundingSpheres::
IsInCollision(shared_ptr<ActiveMultiBody> _robot,
    shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
    size_t _ignoreIAdjacentMultibodies) {

  Vector3d robotCom = _robot->GetCenterOfMass();
  Vector3d obstCom  = _obstacle->GetCenterOfMass();

  if(_robot->GetFreeBodyCount())
    robotCom = _robot->GetFreeBody(0)->GetWorldTransformation() * robotCom;
  if(_obstacle->GetBodyCount())
    obstCom  = _obstacle->GetBody(0)->GetWorldTransformation() * obstCom;

  double robotRadius = _robot->GetBoundingSphereRadius();
  double obstRadius  = _obstacle->GetBoundingSphereRadius();

  double dist = sqrt(sqr(robotCom[0] - obstCom[0]) +
      sqr(robotCom[1] - obstCom[1]) +
      sqr(robotCom[2] - obstCom[2]));

  return dist <= robotRadius + obstRadius;
}


//////////


InsideSpheres::
InsideSpheres() : CollisionDetectionMethod("InsideSpheres", CDType::In) {
  }

bool
InsideSpheres::
IsInCollision(shared_ptr<ActiveMultiBody> _robot,
    shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
    size_t _ignoreIAdjacentMultibodies) {

  Vector3d robotCom = _robot->GetCenterOfMass();
  Vector3d obstCom  = _obstacle->GetCenterOfMass();

  if(_robot->GetFreeBodyCount())
    robotCom = _robot->GetFreeBody(0)->GetWorldTransformation() * robotCom;
  if(_obstacle->GetBodyCount())
    obstCom  = _obstacle->GetBody(0)->GetWorldTransformation() * obstCom;

  double robotRadius = _robot->GetInsideSphereRadius();
  double obstRadius  = _obstacle->GetInsideSphereRadius();

  double dist = sqrt(sqr(robotCom[0] - obstCom[0]) +
      sqr(robotCom[1] - obstCom[1]) +
      sqr(robotCom[2] - obstCom[2]));

  return dist <= robotRadius + obstRadius;
}

