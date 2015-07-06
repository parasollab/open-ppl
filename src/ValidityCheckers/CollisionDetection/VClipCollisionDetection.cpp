#include "VClipCollisionDetection.h"

#ifdef USE_VCLIP

#include "CDInfo.h"
#include "MPProblem/Geometry/Body.h"

ClosestFeaturesHT closestFeaturesHT(3000);

VClip::
VClip() : CollisionDetectionMethod("VCLIP", CDType::Exact) {}

void
VClip::
Build(Body* _body) {
  GMSPolyhedron& poly = _body->GetPolyhedron();
  Polyhedron* vpoly = new Polyhedron;
  for(size_t v = 0 ; v < poly.m_vertexList.size() ; v++){
    vpoly->addVertex("",
        Vect3(poly.m_vertexList[v][0],
          poly.m_vertexList[v][1],
          poly.m_vertexList[v][2]
          ));
  }
  vpoly->buildHull();
  shared_ptr<PolyTree> vclipBody(new PolyTree);
  vclipBody->setPoly(vpoly);
  _body->SetVClipBody(vclipBody);
}

bool
VClip::
IsInCollision(shared_ptr<Body> _body1, shared_ptr<Body> _body2,
    CDInfo& _cdInfo) {

  shared_ptr<PolyTree> body1 = _body1->GetVClipBody();
  shared_ptr<PolyTree> body2 = _body2->GetVClipBody();
  Transformation& t1 = _body1->WorldTransformation();
  Transformation& t2 = _body2->WorldTransformation();

  VClipPose x12 = GetVClipPose(t1, t2);

  Vect3 cp1, cp2; // closest points between bodies, in local frame
  Real dist = PolyTree::vclip(body1.get(), body2.get(), x12,
      closestFeaturesHT, cp1, cp2);

  _cdInfo.m_minDist = dist;

  Vector3d pt1(cp1[0], cp1[1], cp1[2]);
  Vector3d pt2(cp2[0], cp2[1], cp2[2]);

  _cdInfo.m_robotPoint = t1 * pt1;
  _cdInfo.m_objectPoint = t2 * pt2;

  return dist <= 0.0;
}

VClipPose
VClip::
GetVClipPose(const Transformation& _myT, const Transformation& _obstT) {
  Transformation diff = (-_obstT) * _myT;
  Quaternion q;
  convertFromMatrix(q, diff.rotation().matrix());

  Vect3 pos(diff.translation()[0], diff.translation()[1], diff.translation()[2]);

  Quat rot(q.real(), q.imaginary()[0], q.imaginary()[1], q.imaginary()[2]);

  return VClipPose(rot, pos);
}

#endif
