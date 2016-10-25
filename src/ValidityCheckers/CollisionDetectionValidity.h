#ifndef COLLISION_DETECTION_VALIDITY_H_
#define COLLISION_DETECTION_VALIDITY_H_

#include "ValidityCheckerMethod.h"

#include "Environment/ActiveMultiBody.h"
#include "Environment/Environment.h"
#include "Environment/FixedBody.h"
#include "Environment/StaticMultiBody.h"

#include "Utilities/MetricUtils.h"

#include "ValidityCheckers/CollisionDetection/CDInfo.h"
#include "ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/RapidCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief Collision detection validation
/// @tparam MPTraits Motion planning universe
///
/// Validation here means collision-free. This class interfaces with external CD
/// libraries to determing collision information -- sometimes including
/// clearance and penetration information.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class CollisionDetectionValidity : public ValidityCheckerMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ///@name Construction
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _cdMethod Collision detection library
    /// @param _ignoreSelfCollision Compute robot self-collisions?
    /// @param _ignoreIAdjacentLinks For self-collision adjacent links to ignore
    CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod = NULL,
        bool _ignoreSelfCollision = false, int _ignoreIAdjacentLinks = 1);

    ////////////////////////////////////////////////////////////////////////////
    /// @param _problem MPProblem
    /// @param _node XML input node
    CollisionDetectionValidity(MPProblemType* _problem, XMLNode& _node);

    virtual ~CollisionDetectionValidity();

    ///@}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Collision Detection library
    CollisionDetectionMethod* GetCDMethod() const {return m_cdMethod;}

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const string& _callName);
    virtual bool IsInsideObstacle(const CfgType& _cfg);

  private:

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Orchestrate collision computation between robot and environment
    ///        multibodies
    /// @param[out] _cdInfo CDInfo
    /// @param _robotIndex ActiveBody index of robot
    /// @param _callName Function calling validity checker
    /// @return Collision
    bool IsInCollision(CDInfo& _cdInfo, size_t _robotIndex,
        const string& _callName);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Check self-collision with robot
    /// @param[out] _cdInfo CDInfo
    /// @param _rob ActiveMultiBody of robot
    /// @param _callName Function calling validity checker
    /// @return Collision
    bool IsInSelfCollision(CDInfo& _cdInfo, shared_ptr<ActiveMultiBody> _rob,
        const string& _callName);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Check inter-robot collision
    /// @param[out] _cdInfo CDInfo
    /// @param _rob ActiveMultiBody of robot
    /// @param _otherRobot ActiveMultiBody of the other robot to check
    /// @param _callName Function calling validity checker
    /// @return Collision
    bool IsInterRobotCollision(CDInfo& _cdInfo, shared_ptr<ActiveMultiBody> _rob,
        shared_ptr<ActiveMultiBody> _otherRobot, const string& _callName);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Check collision between robot and one obstacle
    /// @param[out] _cdInfo CDInfo
    /// @param _rob ActiveMultiBody of robot
    /// @param _obst StaticMultiBody of obstacle
    /// @param _callName Function calling validity checker
    /// @return Collision
    bool IsInObstCollision(CDInfo& _cdInfo, shared_ptr<ActiveMultiBody> _rob,
        shared_ptr<StaticMultiBody> _obst, const string& _callName);

    CollisionDetectionMethod* m_cdMethod; ///< Collision Detection library
    bool m_ignoreSelfCollision;           ///< Check self collisions
    size_t m_ignoreIAdjacentLinks;        ///< With self collisions how many
                                          ///< adjacent links to ignore
};


template<class MPTraits>
CollisionDetectionValidity<MPTraits>::
CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod,
    bool _ignoreSelfCollision, int _ignoreIAdjacentLinks) :
    ValidityCheckerMethod<MPTraits>(), m_cdMethod(_cdMethod),
    m_ignoreSelfCollision(_ignoreSelfCollision),
    m_ignoreIAdjacentLinks(_ignoreIAdjacentLinks) {
  this->m_name = "CollisionDetection";
}


template<class MPTraits>
CollisionDetectionValidity<MPTraits>::
CollisionDetectionValidity(MPProblemType* _problem, XMLNode& _node) :
    ValidityCheckerMethod<MPTraits>(_problem, _node) {
  this->m_name = "CollisionDetection";

  m_ignoreSelfCollision = _node.Read("ignoreSelfCollision", false, false,
      "Check for self collision");
  m_ignoreIAdjacentLinks = _node.Read("ignoreIAdjacentLinks", false, 1, 0,
      MAX_INT, "number of links to ignore for linkages");

  string cdLabel = _node.Read("method", true, "", "method");

  if(cdLabel == "BoundingSpheres")
    m_cdMethod = new BoundingSpheres();
  else if(cdLabel == "InsideSpheres")
    m_cdMethod = new InsideSpheres();
#ifndef NO_RAPID
  else if(cdLabel == "RAPID")
    m_cdMethod = new Rapid();
#endif
#ifndef NO_PQP
  else if(cdLabel == "PQP")
    m_cdMethod = new PQP();
  else if(cdLabel == "PQP_SOLID")
    m_cdMethod = new PQPSolid();
#endif
  else
    throw ParseException(_node.Where(),
        "Unknown collision detection library '" + cdLabel + "' requested.");
}


template<class MPTraits>
CollisionDetectionValidity<MPTraits>::
~CollisionDetectionValidity() {
  delete m_cdMethod;
}


template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) {
  this->GetStatClass()->IncCfgIsColl(_callName);

  //position environment
  _cfg.ConfigureRobot();

  //check collision
#ifdef PMPCfgMultiRobot
  bool answerFromEnvironment = false;
  for(size_t i = 0; i < CfgType::m_numRobot; ++i)
    answerFromEnvironment |= IsInCollision(_cdInfo, i, _callName);
#else
  bool answerFromEnvironment = IsInCollision(_cdInfo, _cfg.GetRobotIndex(),
      _callName);
#endif

  _cfg.SetLabel("VALID", !answerFromEnvironment);

  return !answerFromEnvironment;
}


template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInCollision(CDInfo& _cdInfo, size_t _robotIndex, const string& _callName) {
  _cdInfo.ResetVars(_cdInfo.m_retAllInfo);
  bool retVal = false;

  //get robot
  Environment* env = this->GetEnvironment();
  shared_ptr<ActiveMultiBody> robot = env->GetRobot(_robotIndex);

  //check self collision
  if(!m_ignoreSelfCollision && robot->NumFreeBody() > 1 &&
      IsInSelfCollision(_cdInfo, robot, _callName)) {
    _cdInfo.m_collidingObstIndex = -1;
    return true;
  }

#ifdef PMPCfgMultiRobot
  //check inter-robot collision for multiple robot case
  //for performance issue, check only robots that have higher robotIndex
  size_t numRobot = env->NumRobots();
  for (size_t n = _robotIndex+1; n < numRobot; n++) {
    shared_ptr<ActiveMultiBody> otherRobot = env->GetRobot(n);
    bool collisionFound = IsInterRobotCollision(_cdInfo, robot, otherRobot,
        _callName);
    if(collisionFound) {
      if(!_cdInfo.m_retAllInfo)
        return true;
      _cdInfo.m_collidingRobtIndex.push_back(make_pair(_robotIndex, n));
    }
  }
  if(_cdInfo.m_collidingRobtIndex.size() > 0)
    return true;
#endif

  //check obstacle collisions
  size_t numObst = env->NumObstacles();
  for(size_t i = 0; i < numObst; ++i) {
    CDInfo cdInfo(_cdInfo.m_retAllInfo);
    bool coll = IsInObstCollision(
        cdInfo, robot, env->GetObstacle(i), _callName);

    //make sure to store closest obstacle information
    if(cdInfo < _cdInfo) {
      _cdInfo = cdInfo;
      _cdInfo.m_nearestObstIndex = i;
    }

    //store first collision in colliding index
    if(coll && !retVal) {
      _cdInfo.m_collidingObstIndex = i;
      retVal = true;
    }

    //early quit if we don't care about distance information
    if(coll && !_cdInfo.m_retAllInfo)
      return true;
  }

  return retVal;
}


template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInSelfCollision(CDInfo& _cdInfo, shared_ptr<ActiveMultiBody> _rob,
    const string& _callName) {
  this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _callName);

  size_t numBody = _rob->NumFreeBody();
  for(size_t i = 0; i < numBody - 1; ++i) {
    for(size_t j = i+1; j < numBody; ++j) {
      shared_ptr<FreeBody> body1 = _rob->GetFreeBody(i);
      shared_ptr<FreeBody> body2 = _rob->GetFreeBody(j);

      if(body1->IsWithinI(body2, m_ignoreIAdjacentLinks))
        continue;

      if(m_cdMethod->IsInCollision(body1, body2, _cdInfo))
        return true;
    }
  }

  return false;
}


template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInterRobotCollision(CDInfo& _cdInfo,
    shared_ptr<ActiveMultiBody> _rob,
    shared_ptr<ActiveMultiBody> _otherRobot, const string& _callName) {
  this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _callName);

  size_t numBody = _rob->NumFreeBody();
  size_t numOtherBody = _otherRobot->NumFreeBody();

  for(size_t i = 0; i < numBody; ++i) {
    for(size_t j = 0; j < numOtherBody; ++j) {
      bool collisionFound =
        m_cdMethod->IsInCollision(_rob->GetFreeBody(i),
            _otherRobot->GetFreeBody(j), _cdInfo);

      if(collisionFound)
        return true;
    }
  }
  return false;
}


template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInObstCollision(CDInfo& _cdInfo, shared_ptr<ActiveMultiBody> _rob,
    shared_ptr<StaticMultiBody> _obst, const string& _callName) {
  this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _callName);

  bool collision = false;
  size_t numBody = _rob->NumFreeBody();
  shared_ptr<Body> obst = _obst->GetFixedBody(0);

  for(size_t i = 0; i < numBody; ++i) {
    CDInfo cdInfo(_cdInfo.m_retAllInfo);
    bool coll = m_cdMethod->IsInCollision(_rob->GetFreeBody(i), obst, cdInfo);

    //retain minimum distance information
    if(cdInfo < _cdInfo)
      _cdInfo = cdInfo;

    //Early quit if we do not care for distance information
    if(coll && !_cdInfo.m_retAllInfo)
      return true;
    else if(coll)
      collision = true;
  }

  return collision;
}


template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInsideObstacle(const CfgType& _cfg) {
  Environment* env = this->GetEnvironment();
  size_t nMulti = env->NumObstacles();

  Vector3d robotPt(_cfg.GetData()[0], _cfg.GetData()[1], _cfg.GetData()[2]);

  for(size_t i = 0; i < nMulti; ++i)
    if(m_cdMethod->IsInsideObstacle(
          robotPt, env->GetObstacle(i)->GetFixedBody(0)))
      return true;
  return false;
}

#endif
