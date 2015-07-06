#include "SpheresCollisionDetection.h"

#include "Environment/Body.h"

BoundingSpheres::
BoundingSpheres() : CollisionDetectionMethod("BoundingSpheres", CDType::Out) {
}

bool
BoundingSpheres::
IsInCollision(shared_ptr<Body> _body1, shared_ptr<Body> _body2,
    CDInfo& _cdInfo) {

  Vector3d body1Com = _body1->GetWorldTransformation() * _body1->GetCenterOfMass();
  Vector3d body2Com = _body2->GetWorldTransformation() * _body2->GetCenterOfMass();

  double body1Radius = _body1->GetBoundingSphereRadius();
  double body2Radius = _body2->GetBoundingSphereRadius();

  double dist = (body1Com - body2Com).norm();

  return dist <= body1Radius + body2Radius;
}

InsideSpheres::
InsideSpheres() : CollisionDetectionMethod("InsideSpheres", CDType::In) {
}

bool
InsideSpheres::
IsInCollision(shared_ptr<Body> _body1, shared_ptr<Body> _body2,
    CDInfo& _cdInfo) {

  Vector3d body1Com = _body1->GetWorldTransformation() * _body1->GetCenterOfMass();
  Vector3d body2Com = _body2->GetWorldTransformation() * _body2->GetCenterOfMass();

  double body1Radius = _body1->GetInsideSphereRadius();
  double body2Radius = _body2->GetInsideSphereRadius();

  double dist = (body1Com - body2Com).norm();

  return dist <= body1Radius + body2Radius;
}
