#include "PQPCollisionDetection.h"

#ifndef NO_PQP

#include "CDInfo.h"
#include "Environment/Body.h"
#include <set>

/*------------------------------- Construction -------------------------------*/

PQP::
PQP() : CollisionDetectionMethod("PQP") { }

/*------------------------------- CD Interface -------------------------------*/

void
PQP::
Build(Body* _body) {
  GMSPolyhedron& polyhedron = _body->GetPolyhedron();
  shared_ptr<PQP_Model> pqpBody(new PQP_Model);
  pqpBody->BeginModel();
  size_t q = 0;
  for(const auto& poly : polyhedron.m_polygonList) {
    double point[3][3];
    for(size_t i = 0; i < 3; ++i)
      for(size_t j = 0; j < 3; ++j)
        point[i][j] = poly.GetPoint(i)[j];
    pqpBody->AddTri(point[0], point[1], point[2], q++);
  }
  pqpBody->EndModel();
  _body->SetPQPBody(pqpBody);
}


bool
PQP::
IsInCollision(shared_ptr<Body> _body1, shared_ptr<Body> _body2,
    CDInfo& _cdInfo) {

  shared_ptr<PQP_Model> body1 = _body1->GetPQPBody();
  shared_ptr<PQP_Model> body2 = _body2->GetPQPBody();
  Transformation& t1 = _body1->WorldTransformation();
  Transformation& t2 = _body2->WorldTransformation();

  if(_cdInfo.m_retAllInfo) {
    PQP_DistanceResult result;
    if(PQP_Distance(&result,
          t1.rotation().matrix(), t1.translation(), body1.get(),
          t2.rotation().matrix(), t2.translation(), body2.get(), 0.0, 0.0))
      throw RunTimeException(WHERE, "PQP_ERR_COLLIDE_OUT_OF_MEMORY");

    _cdInfo.m_minDist = result.Distance();

    _cdInfo.m_robotPoint = t1 * Vector3d(result.P1());
    _cdInfo.m_objectPoint = t2 * Vector3d(result.P2());

    return result.Distance() <= 0.0;
  }
  else {
    PQP_CollideResult result;
    if(PQP_Collide(&result,
          t1.rotation().matrix(), t1.translation(), body1.get(),
          t2.rotation().matrix(), t2.translation(), body2.get(),
          PQP_FIRST_CONTACT))
      throw RunTimeException(WHERE, "PQP_ERR_COLLIDE_OUT_OF_MEMORY");

    return result.Colliding();
  }
}


bool
PQPSolid::
IsInCollision(shared_ptr<Body> _body1, shared_ptr<Body> _body2,
    CDInfo& _cdInfo) {
  bool collision = PQP::IsInCollision(_body1, _body2, _cdInfo);
  if(!collision)
    collision = IsInsideObstacle(_body1->GetWorldPolyhedron().m_vertexList[0],
        _body2);
  return collision;
}


bool
PQPSolid::
IsInsideObstacle(const Vector3d& _pt, shared_ptr<Body> _body) {
  // Set up a pseudo-ray for a ray-shooting test.
  static PQP_Model* ray = BuildPseudoRay();
  static PQP_REAL rotation[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL translation[3] = {_pt[0], _pt[1], _pt[2]};

  // Get obstacle info.
  shared_ptr<PQP_Model> body = _body->GetPQPBody();
  Transformation& t2 = _body->WorldTransformation();

  // Perform ray-shooting collision test.
  PQP_CollideResult result;
  PQP_Collide(&result, rotation, translation, ray,
      t2.rotation().matrix(), t2.translation(), body.get());

  // Sort collisions by relative X-value.

  static const double tolerance = 10 * numeric_limits<double>::epsilon();
  static const Vector3d r(10e6, 0, 0); // Vector-representation of the ray.
  const auto& vertices = _body->GetWorldPolyhedron().GetVertexList();
  const auto& polygons = _body->GetWorldPolyhedron().GetPolygonList();

  enum TransitionType {Entering = 0, Exiting = 1};
  typedef pair<double, TransitionType> Transition;

  // We will store the processed collisions in a set to sort them and remove
  // duplicate transitions of the same type and x-value. Duplicate removal guards
  // against the case where the psuedo-ray passes through an edge or vertex that
  // is shared by multiple triangles with the same facing.
  static auto compare = [](const Transition& _t1, const Transition& _t2) -> bool {
    if(abs(_t1.first - _t2.first) > tolerance)
      return _t1.first < _t2.first;
    else
      return _t1.second < _t2.second;
  };
  static set<Transition, decltype(compare)> collisions(compare);

  // Process each collision.
  collisions.clear();
  for(int i = 0; i < result.NumPairs(); ++i) {
    const auto& triangle = polygons[result.Id2(i)];
    const auto& v = vertices[triangle[0]];
    const auto& n = triangle.GetNormal();

    // Skip collisions against triangles whose normals are perpendicular to the
    // ray: these are scrapes and don't affect inside/outside-ness.
    if(abs(n[0]) < tolerance) continue;

    // The collision occurs at some fraction of r. This fraction is the ratio of
    // |pt to the triangle plane| over |r's projection along n|.
    double alpha = ((v - _pt) * n) / (r * n);

    // We are exiting the triangle if the normal has positive x value and
    // entering it otherwise (zero x values handled above).
    collisions.emplace(alpha * r[0], TransitionType(n[0] > 0));
  }

  // Check the ordered collisions to see what happened. Skip over any points
  // where we enter and exit at the same point.
  while(!collisions.empty()) {
    if(collisions.size() == 1 ||
        abs(collisions.begin()->first - (++collisions.begin())->first) >
        tolerance)
      return collisions.begin()->second;
    else
      collisions.erase(collisions.begin(), ++++collisions.begin());
  }

  // If we're still here, there are no valid collisions.
  return false;
}

/*--------------------------------- Helpers ----------------------------------*/

PQP_Model*
PQPSolid::
BuildPseudoRay() const {
  PQP_Model* ray{new PQP_Model()};

  PQP_REAL zero[3] = {0, 0, 0};
  PQP_REAL x[3] = {1e10, 0, 0};

  ray->BeginModel();
  ray->AddTri(zero, zero, x, 0);
  ray->EndModel();

  return ray;
}

/*----------------------------------------------------------------------------*/

#endif
