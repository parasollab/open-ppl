#include "SpheresCollisionDetection.h"

#include "Environment/Body.h"


BoundingSpheres::
BoundingSpheres() : CollisionDetectionMethod("BoundingSpheres") { }


bool
BoundingSpheres::
IsInCollision(shared_ptr<Body> _body1, shared_ptr<Body> _body2,
    CDInfo& _cdInfo) {
  // Get distance between the bodies.
  const Vector3d body1Com = _body1->GetWorldTransformation() *
      _body1->GetCenterOfMass();
  const Vector3d body2Com = _body2->GetWorldTransformation() *
      _body2->GetCenterOfMass();
  const double dist = (body1Com - body2Com).norm();

  const double body1Radius = _body1->GetBoundingSphereRadius();
  const double body2Radius = _body2->GetBoundingSphereRadius();

  return dist <= body1Radius + body2Radius;
}


InsideSpheres::
InsideSpheres() : CollisionDetectionMethod("InsideSpheres") { }


bool
InsideSpheres::
IsInCollision(shared_ptr<Body> _body1, shared_ptr<Body> _body2,
    CDInfo& _cdInfo) {
  // Get distance between the bodies.
  const Vector3d body1Com = _body1->GetWorldTransformation() *
      _body1->GetCenterOfMass();
  const Vector3d body2Com = _body2->GetWorldTransformation() *
      _body2->GetCenterOfMass();
  const double dist = (body1Com - body2Com).norm();

  const double body1Radius = _body1->GetInsideSphereRadius();
  const double body2Radius = _body2->GetInsideSphereRadius();

  return dist <= body1Radius + body2Radius;
}
