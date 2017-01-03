#include "SpheresCollisionDetection.h"

#include "Geometry/Bodies/Body.h"


BoundingSpheres::
BoundingSpheres() : CollisionDetectionMethod("BoundingSpheres") { }


bool
BoundingSpheres::
IsInCollision(const Body* const _body1, const Body* const _body2,
    CDInfo& _cdInfo) {
  // Get distance between the bodies.
  const Vector3d& body1Com = _body1->GetWorldPolyhedron().GetCentroid();
  const Vector3d& body2Com = _body2->GetWorldPolyhedron().GetCentroid();
  const double dist = (body1Com - body2Com).norm();

  const double body1Radius = _body1->GetBoundingSphereRadius();
  const double body2Radius = _body2->GetBoundingSphereRadius();

  return dist <= body1Radius + body2Radius;
}


InsideSpheres::
InsideSpheres() : CollisionDetectionMethod("InsideSpheres") { }


bool
InsideSpheres::
IsInCollision(const Body* const _body1, const Body* const _body2,
    CDInfo& _cdInfo) {
  // Get distance between the bodies.
  const Vector3d& body1Com = _body1->GetWorldPolyhedron().GetCentroid();
  const Vector3d& body2Com = _body2->GetWorldPolyhedron().GetCentroid();
  const double dist = (body1Com - body2Com).norm();

  const double body1Radius = _body1->GetInsideSphereRadius();
  const double body2Radius = _body2->GetInsideSphereRadius();

  return dist <= body1Radius + body2Radius;
}
