#include "PQPCollisionDetection.h"

#ifndef NO_PQP

#include "CDInfo.h"
#include "Environment/Body.h"

PQP::
PQP() : CollisionDetectionMethod("PQP", CDType::Exact) {
}

PQP::
~PQP() {
}

void
PQP::
Build(Body* _body, bool _isConvex) {
  GMSPolyhedron obstacle;
  if(_isConvex)
    obstacle = _body->GetConvexPolyhedron();
  else
    obstacle = _body->GetPolyhedron();
  GMSPolyhedron& poly = obstacle;

  shared_ptr<PQP_Model> pqpBody(new PQP_Model);
  pqpBody->BeginModel();
  for(size_t q = 0; q < poly.m_polygonList.size(); q++) {
    int vertexNum[3];
    double point[3][3];
    for(int i = 0; i < 3; i++) {
      vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
      Vector3d& tmp = poly.m_vertexList[vertexNum[i]];
      for(int j = 0; j < 3; j++)
        point[i][j] = tmp[j];
    }
    pqpBody->AddTri(point[0], point[1], point[2], q);
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

    _cdInfo.m_robotPoint = t1 * result.P1();
    _cdInfo.m_objectPoint = t2 * result.P2();

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
    collision = IsInsideObstacle(_body1->GetWorldPolyhedron().m_vertexList[0], _body2);
  return collision;
}

bool
PQPSolid::
IsInsideObstacle(const Vector3d& _pt, shared_ptr<Body> _body) {
  static PQP_Model* ray = BuildPQPSegment(1e10, 0, 0);

  PQP_REAL t[3] = {_pt[0], _pt[1], _pt[2]};
  static PQP_REAL r[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

  shared_ptr<PQP_Model> body = _body->GetPQPBody();
  Transformation& t2 = _body->WorldTransformation();

  PQP_CollideResult result;
  PQP_Collide(&result, r, t, ray,
      t2.rotation().matrix(), t2.translation(), body.get());

  return result.NumPairs() % 2 == 1;
}

PQP_Model*
PQPSolid::BuildPQPSegment(PQP_REAL _x, PQP_REAL _y, PQP_REAL _z) const {
  //build a narrow triangle.
  PQP_Model* ray = new PQP_Model();

  if(_x == 0 && _y == 0 && _z == 0)
     cerr << "! CollisionDetection::BuildPQPRay Warning : All are [0]" << endl;

  static PQP_REAL tinyV = 1e-20/numeric_limits<long>::max();
  static PQP_REAL picoV = tinyV/2;
  static PQP_REAL p1[3] = { tinyV, tinyV, tinyV };
  static PQP_REAL p2[3] = { picoV, picoV, picoV };
  PQP_REAL p3[3] = { _x, _y, _z};

  ray->BeginModel();
  ray->AddTri(p1, p2, p3, 0);
  ray->EndModel();

  return ray;
}

#endif
