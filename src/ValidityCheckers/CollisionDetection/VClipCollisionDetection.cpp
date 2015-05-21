#ifdef USE_VCLIP

#include "VClipCollisionDetection.h"

#include "CDInfo.h"
#include "MPProblem/Geometry/ActiveMultiBody.h"
#include "MPProblem/Geometry/FreeBody.h"

VClip::
VClip() : CollisionDetectionMethod("VCLIP", CDType::Exact) {}

ClosestFeaturesHT closestFeaturesHT(3000);

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
IsInCollision(shared_ptr<ActiveMultiBody> _robot,
    shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
    size_t _ignoreIAdjacentMultibodies) {

  if (_cdInfo.m_retAllInfo == true)
    return FillCdInfo(_robot, _obstacle, _cdInfo, _ignoreIAdjacentMultibodies);

  Real dist;
  VClipPose x12; //VClip position
  Vect3 cp1, cp2;   // closest points between bodies, in local frame

  for(size_t i = 0; i < _robot->GetFreeBodyCount(); ++i) {

    shared_ptr<PolyTree> rob = _robot->GetFreeBody(i)->GetVClipBody();

    for(size_t j = 0; j < _obstacle->GetBodyCount(); ++j) {

      // if robot check self collision, skip adjacent links.
      if(_robot == _obstacle &&
          _robot->GetFreeBody(i)->IsWithinI(
            _obstacle->GetBody(j), _ignoreIAdjacentMultibodies))
        continue;


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
VClip::
FillCdInfo(shared_ptr<ActiveMultiBody> _robot,
    shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
    size_t _ignoreIAdjacentMultibodies) {
  Real dist, minCurrentDist;
  VClipPose x12;
  Vect3 cp1, cp2; //closest points between bodies, in local frame

  bool isInCollision = false;

  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  minCurrentDist = MAX_DBL;

  for(size_t i = 0; i < _robot->GetFreeBodyCount(); ++i) {
    shared_ptr<PolyTree> rob = _robot->GetFreeBody(i)->GetVClipBody();

    for(size_t j = 0; j < _obstacle->GetBodyCount(); ++j) {
      // if robot check self collision, skip adjacent links.
      if(_robot == _obstacle &&
          _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) )
        continue;


      shared_ptr<PolyTree> obst = _obstacle->GetBody(j)->GetVClipBody();
      x12 = GetVClipPose(_robot->GetFreeBody(i)->WorldTransformation(),
          _obstacle->GetBody(j)->WorldTransformation());
      dist = PolyTree::vclip(rob.get(),obst.get(),x12,closestFeaturesHT, cp1, cp2);

      if ( dist <= 0.0 ) {
        if (dist < minCurrentDist)
          _cdInfo.m_collidingObstIndex = j;
        isInCollision = true;
      }

      if (dist < minCurrentDist) {
        minCurrentDist = dist;
        _cdInfo.m_nearestObstIndex = j;
        _cdInfo.m_minDist = dist;

        // change a 3 elmt array to Vector3d class
        Vector3d robotPt(cp1[0], cp1[1], cp1[2]);
        Vector3d obstPt(cp2[0], cp2[1], cp2[2]);

        // transform points to world coords
        _cdInfo.m_robotPoint = _robot->GetFreeBody(i)->WorldTransformation() * robotPt;
        _cdInfo.m_objectPoint = _obstacle->GetBody(j)->WorldTransformation() * obstPt;
      }
    }
  }

  return isInCollision;
}

VClipPose
VClip::GetVClipPose(const Transformation& _myT, const Transformation& _obstT) {
  Transformation diff = (-_obstT) * _myT;
  Quaternion q;
  convertFromMatrix(q, diff.rotation().matrix());

  Vect3 pos(diff.translation()[0], diff.translation()[1], diff.translation()[2]);

  Quat rot(q.real(), q.imaginary()[0], q.imaginary()[1], q.imaginary()[2]);

  return VClipPose(rot, pos);
}

#endif
