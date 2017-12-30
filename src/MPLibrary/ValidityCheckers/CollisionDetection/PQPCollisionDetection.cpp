#include "PQPCollisionDetection.h"

#include "CDInfo.h"
#include "Geometry/Bodies/Body.h"

#include <set>

#include <PQP.h>


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PQP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

PQP::
PQP() : CollisionDetectionMethod("PQP") { }

/*------------------------------- CD Interface -------------------------------*/

void
PQP::
Build(Body* const _body) {
  const GMSPolyhedron& poly = _body->GetPolyhedron();
  std::unique_ptr<PQP_Model> pqpBody(new PQP_Model);
  pqpBody->BeginModel();
  for(size_t q = 0; q < poly.m_polygonList.size(); q++) {
    double point[3][3];
    for(int i = 0; i < 3; i++) {
      const Vector3d& tmp = poly.m_polygonList[q].GetPoint(i);
      for(int j = 0; j < 3; j++)
        point[i][j] = tmp[j];
    }
    pqpBody->AddTri(point[0], point[1], point[2], q);
  }
  pqpBody->EndModel();
  _body->SetPQPBody(std::move(pqpBody));
}


bool
PQP::
IsInCollision(const Body* const _body1, const Body* const _body2,
    CDInfo& _cdInfo) {
  auto body1 = _body1->GetPQPBody();
  auto body2 = _body2->GetPQPBody();
  /// @TODO See if we can modify PQP_Distance to take const double arrays
  ///       instead of just double arrays so we don't have to copy.
  //const Transformation& t1 = _body1->GetWorldTransformation();
  //const Transformation& t2 = _body2->GetWorldTransformation();
  Transformation t1 = _body1->GetWorldTransformation();
  Transformation t2 = _body2->GetWorldTransformation();

  if(_cdInfo.m_retAllInfo) {
    PQP_DistanceResult result;
    if(PQP_Distance(&result,
          t1.rotation().matrix(), t1.translation(), body1,
          t2.rotation().matrix(), t2.translation(), body2, 0.0, 0.0))
      throw RunTimeException(WHERE, "PQP_ERR_COLLIDE_OUT_OF_MEMORY");

    _cdInfo.m_minDist = result.Distance();

    _cdInfo.m_robotPoint = t1 * result.P1();
    _cdInfo.m_objectPoint = t2 * result.P2();

    const bool inCollision = result.Distance() <= 0.0;

    if(inCollision) {
      // Now do a collision check to get all colliding triangle pairs.
      PQP_CollideResult result;
      if(PQP_Collide(&result,
            t1.rotation().matrix(), t1.translation(), body1,
            t2.rotation().matrix(), t2.translation(), body2,
            PQP_ALL_CONTACTS))
        throw RunTimeException(WHERE, "PQP_ERR_COLLIDE_OUT_OF_MEMORY");

      for(int i = 0; i < result.NumPairs(); ++i)
        _cdInfo.m_trianglePairs.emplace_back(result.Id1(i), result.Id2(i));
    }

    return inCollision;
  }
  else {

    PQP_CollideResult result;
    if(PQP_Collide(&result,
          t1.rotation().matrix(), t1.translation(), body1,
          t2.rotation().matrix(), t2.translation(), body2,
          PQP_FIRST_CONTACT))
      throw RunTimeException(WHERE, "PQP_ERR_COLLIDE_OUT_OF_MEMORY");

    if(result.Colliding())
      _cdInfo.m_trianglePairs.emplace_back(result.Id1(0), result.Id2(0));

    return result.Colliding();
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PQPSolid ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

PQPSolid::
PQPSolid() : PQP() {
  m_name = "PQP_SOLID";
}

/*------------------------------- CD Interface -------------------------------*/

bool
PQPSolid::
IsInCollision(const Body* const _body1, const Body* const _body2,
    CDInfo& _cdInfo) {
  bool collision = PQP::IsInCollision(_body1, _body2, _cdInfo);
  if(!collision)
  {
    const auto& modelPoint = _body1->GetPolyhedron().m_vertexList[0];
    const auto  worldPoint = _body1->GetWorldTransformation() * modelPoint;

    collision = IsInsideObstacle(worldPoint, _body2);
  }

  return collision;
}


bool
PQPSolid::
IsInsideObstacle(const Vector3d& _pt, const Body* const _body) {
  // Set up a pseudo-ray for a ray-shooting test.
  static PQP_Model* ray = BuildPseudoRay();
  static PQP_REAL rotation[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL translation[3] = {_pt[0], _pt[1], _pt[2]};

  // Get obstacle info.
  auto body = _body->GetPQPBody();
  /// @TODO See if we can modify PQP_Collide to take const double arrays
  ///       instead of just double arrays so we don't have to copy.
  Transformation t2 = _body->GetWorldTransformation();

  // Perform ray-shooting collision test.
  PQP_CollideResult result;
  PQP_Collide(&result, rotation, translation, ray,
      t2.rotation().matrix(), t2.translation(), body);

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
    const double alpha = ((v - _pt) * n) / (r * n);

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
