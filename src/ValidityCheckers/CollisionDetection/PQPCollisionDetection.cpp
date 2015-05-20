#include "PQPCollisionDetection.h"

#include "CDInfo.h"
#include "Cfg/Cfg.h"
#include "MPProblem/Environment.h"
#include "MPProblem/Geometry/ActiveMultiBody.h"
#include "MPProblem/Geometry/FreeBody.h"
#include "Utilities/MetricUtils.h"

#ifdef USE_PQP

PQP::
PQP() : CollisionDetectionMethod("PQP", CDType::Exact, PROXIMITYQUERYPACKAGE) {
}

PQP::~PQP() {
}

bool
PQP::
IsInCollision(shared_ptr<ActiveMultiBody> _robot,
    shared_ptr<MultiBody> _obstacle, StatClass& _stats, CDInfo& _cdInfo,
    const string& _callName, size_t _ignoreIAdjacentMultibodies) {

   _stats.IncNumCollDetCalls(GetName(), _callName);

  if(_cdInfo.m_retAllInfo) {
    PQP_DistanceResult res;
    double minDistSoFar = MAX_DBL;
    _cdInfo.ResetVars();
    _cdInfo.m_retAllInfo = true;
    Vector3d robotPt, obsPt;
    bool retVal = false;

    //for each part of robot
    for(size_t i = 0; i < _robot->GetFreeBodyCount(); ++i) {
      shared_ptr<PQP_Model> rob = _robot->GetFreeBody(i)->GetPQPBody();
      Transformation& t1 = _robot->GetFreeBody(i)->WorldTransformation();

      //for each part of obstacle
      for(size_t j = 0; j < _obstacle->GetBodyCount(); ++j) {
        // if robot check self collision, skip adjacent links.
        if(_robot == _obstacle &&
           _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies))
          continue;

        shared_ptr<PQP_Model> obst = _obstacle->GetBody(j)->GetPQPBody();

        Transformation& t2 = _obstacle->GetBody(j)->WorldTransformation();

        if(PQP_Distance(&res,
              t1.rotation().matrix(), t1.translation(), rob.get(),
              t2.rotation().matrix(), t2.translation(), obst.get(), 0.0, 0.0))
          throw RunTimeException(WHERE, "PQP_ERR_COLLIDE_OUT_OF_MEMORY");

        if(res.Distance() <= 0.0){
	  if(res.Distance() < minDistSoFar)
	    _cdInfo.m_collidingObstIndex = j;
          retVal = true;
	}

        if(res.Distance() < minDistSoFar){
          _cdInfo.m_nearestObstIndex = j;
          // which called this function - look there for more info
          minDistSoFar=res.Distance();
          _cdInfo.m_minDist = minDistSoFar;

          // change a 3 elmt array to Vector3d class
          for(int k=0; k < 3; k++) {
            robotPt[k] = res.P1()[k];
            obsPt[k] = res.P2()[k];
          }
          // transform points to world coords
          // using *_pt vars in case overloaded * was not done well.
          _cdInfo.m_robotPoint = _robot->GetFreeBody(i)->WorldTransformation() * robotPt;
          _cdInfo.m_objectPoint = _obstacle->GetBody(j)->WorldTransformation() * obsPt;
        }
      }//end of each part of obs
    }//end of each part of robot

    return retVal;
  }
  else {
    for(size_t i = 0 ; i < _robot->GetFreeBodyCount(); ++i) {
      shared_ptr<PQP_Model> rob = _robot->GetFreeBody(i)->GetPQPBody();

      for(size_t j=0; j < _obstacle->GetBodyCount(); ++j) {
        // if robot check self collision, skip adjacent links.
        if(_robot == _obstacle &&
           _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies))
          continue;

        shared_ptr<PQP_Model> obst = _obstacle->GetBody(j)->GetPQPBody();
        Transformation& t1 = _robot->GetFreeBody(i)->WorldTransformation();
        Transformation& t2 = _obstacle->GetBody(j)->WorldTransformation();

        PQP_CollideResult result;
        if(PQP_Collide(&result,
              t1.rotation().matrix(), t1.translation(), rob.get(),
              t2.rotation().matrix(), t2.translation(), obst.get(),
              PQP_FIRST_CONTACT))
          throw RunTimeException(WHERE, "PQP_ERR_COLLIDE_OUT_OF_MEMORY");

        if(result.Colliding())
          return true;
      }
    }
    return false;
  }
}

bool
PQPSolid::IsInsideObstacle(const Cfg& _cfg, Environment* _env){
  size_t nMulti = _env->GetUsableMultiBodyCount();
  size_t robot = _cfg.GetRobotIndex();

  Vector3d robotPt(_cfg.GetData()[0], _cfg.GetData()[1], _cfg.GetData()[2]);

  for(size_t i=0; i < nMulti; i++ )
    if(i != robot && IsInsideObstacle(robotPt, _env->GetMultiBody(i)))
      return true;
  return false;
}


PQP_Model*
PQPSolid::BuildPQPSegment(PQP_REAL _dX, PQP_REAL _dY, PQP_REAL _dZ) const{
  //build a narrow triangle.
  PQP_Model* pRay = new PQP_Model();
  if( pRay==NULL )
    return NULL;

  if( _dY==0 && _dZ==0 && _dX==0 )
     cerr << "! CollisionDetection::BuildPQPRay Warning : All are [0]" << endl;

  static PQP_REAL tinyV = ((double)1e-20)/numeric_limits<long>::max();
  static PQP_REAL picoV = tinyV/2;
  PQP_REAL p1[3] = { tinyV, tinyV, tinyV };
  PQP_REAL p2[3] = { picoV, picoV, picoV };
  PQP_REAL p3[3] = { _dX, _dY, _dZ};

  pRay->BeginModel();
  pRay->AddTri(p1, p2, p3, 0);
  pRay->EndModel();

  return pRay;
}


bool
PQPSolid::IsInsideObstacle(Vector3d _robotPt, shared_ptr<MultiBody> _obstacle){
  static PQP_Model* mPRay = BuildPQPSegment(1e10,0,0);
  assert(mPRay != NULL);

  PQP_REAL t[3]={_robotPt[0], _robotPt[1], _robotPt[2]};
  static PQP_REAL r[3][3]={{1,0,0}, {0,1,0}, {0,0,1}};

  for(size_t j = 0; j < _obstacle->GetBodyCount(); ++j) {
    shared_ptr<PQP_Model> obst = _obstacle->GetBody(j)->GetPQPBody();
    //GMSPolyhedron& poly=_obstacle->GetBody(j)->GetPolyhedron();
    Transformation& t2 = _obstacle->GetBody(j)->WorldTransformation();

    PQP_CollideResult result;
    PQP_Collide(&result, r, t, mPRay, t2.rotation().matrix(), t2.translation(), obst.get());

    if(result.NumPairs() % 2 == 1)
      return true;
  }//end of each part of obs

  return false;
}


bool
PQPSolid::
IsInCollision(shared_ptr<ActiveMultiBody> _robot,
    shared_ptr<MultiBody> _obstacle, StatClass& _stats, CDInfo& _cdInfo,
    const string& _callName, size_t _ignoreIAdjacentMultibodies) {

  _stats.IncNumCollDetCalls(GetName(), _callName);

  PQP_CollideResult result;

  if (_cdInfo.m_retAllInfo == true){
    PQP_DistanceResult res;
    double minDistSoFar = MAX_DBL;
    Vector3d robotPt, obsPt;
    bool retVal=false;

    //for each part of robot
    for(size_t i = 0; i < _robot->GetFreeBodyCount(); ++i) {
      shared_ptr<PQP_Model> rob = _robot->GetFreeBody(i)->GetPQPBody();
      Transformation& t1 = _robot->GetFreeBody(i)->WorldTransformation();

      //for each part of obstacle
      for(size_t j = 0; j < _obstacle->GetBodyCount(); ++j) {
        // if robot check self collision, skip adjacent links.
	//replace with finction that checks is is in i of link
        if(_robot == _obstacle &&
           _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies))
          continue;

        shared_ptr<PQP_Model> obst = _obstacle->GetBody(j)->GetPQPBody();

        Transformation& t2 = _obstacle->GetBody(j)->WorldTransformation();

        if(PQP_Distance(&res,
              t1.rotation().matrix(), t1.translation(), rob.get(),
              t2.rotation().matrix(), t2.translation(), obst.get(), 0.0, 0.0))
          throw RunTimeException(WHERE, "PQP_ERR_COLLIDE_OUT_OF_MEMORY");

        if(res.Distance() <= 0.0)
          retVal = true;

        if(res.Distance() < minDistSoFar){
          // _cdInfo.m_nearestObstIndex =  is set by IsInCollision()
          // which called this function - look there for more info
          minDistSoFar=res.Distance();
          _cdInfo.m_minDist = minDistSoFar;

          // change a 3 elmt array to Vector3d class
          for(int k=0;k < 3;k++){
            robotPt[k] = res.P1()[k];
            obsPt[k] = res.P2()[k];
          }

          // transform points to world coords
          // using *_pt vars in case overloaded * was not done well.
          _cdInfo.m_robotPoint = _robot->GetFreeBody(i)->WorldTransformation() * robotPt;
          _cdInfo.m_objectPoint = _obstacle->GetBody(j)->WorldTransformation() * obsPt;
        }
      }//end of each part of obs

      if(retVal == false && _robot != _obstacle && IsInsideObstacle(_robot->GetFreeBody(i)->GetWorldPolyhedron().m_vertexList[0], _obstacle))
        retVal = true;
    }//end of each part of robot
    return retVal;
  }
  else{
    for(size_t i = 0; i < _robot->GetFreeBodyCount(); ++i) {
      shared_ptr<PQP_Model> rob = _robot->GetFreeBody(i)->GetPQPBody();

      for(size_t j = 0; j < _obstacle->GetBodyCount(); ++j) {
        // if robot check self collision, skip adjacent links.
        if(_robot == _obstacle &&
           _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies))
          continue;

        shared_ptr<PQP_Model> obst = _obstacle->GetBody(j)->GetPQPBody();
        Transformation& t1 = _robot->GetFreeBody(i)->WorldTransformation();
        Transformation& t2 = _obstacle->GetBody(j)->WorldTransformation();

        if(PQP_Collide(&result,
              t1.rotation().matrix(), t1.translation(), rob.get(),
              t2.rotation().matrix(), t2.translation(), obst.get(),
              PQP_FIRST_CONTACT))
          throw RunTimeException(WHERE, "PQP_ERR_COLLIDE_OUT_OF_MEMORY");

        if(result.Colliding())
          return true;
      }

      if(_robot != _obstacle && IsInsideObstacle(_robot->GetFreeBody(i)->GetWorldPolyhedron().m_vertexList[0], _obstacle))
        return true;
    }
    return false;
  }
}

#endif

