#ifndef COLLISION_DETECTION_VALIDITY_H_
#define COLLISION_DETECTION_VALIDITY_H_

#include "ValidityCheckerMethod.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "Geometry/Bodies/FixedBody.h"
#include "Geometry/Bodies/StaticMultiBody.h"

#include "Utilities/MetricUtils.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/RapidCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// Validation here means collision-free. This class interfaces with external CD
/// libraries to determine collision information -- sometimes including
/// clearance and penetration information.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class CollisionDetectionValidity : public ValidityCheckerMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    /// @param _cdMethod Collision detection library
    /// @param _ignoreSelfCollision Compute robot self-collisions?
    /// @param _ignoreIAdjacentLinks For self-collision adjacent links to ignore
    CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod = nullptr,
        bool _ignoreSelfCollision = false, int _ignoreIAdjacentLinks = 1);

    CollisionDetectionValidity(XMLNode& _node);

    virtual ~CollisionDetectionValidity();

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const string& _callName);

    ///@}
    ///@name CollisionDetection Interface
    ///@{

    /// @return Collision Detection library
    CollisionDetectionMethod* GetCDMethod() const noexcept {return m_cdMethod;}

    virtual bool IsInsideObstacle(const CfgType& _cfg);

    /// Check if two workspace points are mutually visible.
    /// @param _a The first point.
    /// @param _b The second point.
    /// @return True if _a is visible from _b and vice versa.
    virtual bool WorkspaceVisibility(const Point3d& _a, const Point3d& _b);

    ///@}

  private:

    /// Orchestrate collision computation between robot and environment
    /// multibodies
    /// @param[out] _cdInfo CDInfo
    /// @param _cfg Configuration of interest.
    /// @param _callName Function calling validity checker
    /// @return Collision
    bool IsInCollision(CDInfo& _cdInfo, const CfgType& _cfg,
        const string& _callName);

    /// Check self-collision with robot
    /// @param[out] _cdInfo CDInfo
    /// @param _rob ActiveMultiBody of robot
    /// @param _callName Function calling validity checker
    /// @return Collision
    bool IsInSelfCollision(CDInfo& _cdInfo, ActiveMultiBody* _rob,
        const string& _callName);

    /// Check inter-robot collision
    /// @param[out] _cdInfo CDInfo
    /// @param _rob ActiveMultiBody of robot
    /// @param _otherRobot ActiveMultiBody of the other robot to check
    /// @param _callName Function calling validity checker
    /// @return Collision
    bool IsInterRobotCollision(CDInfo& _cdInfo, ActiveMultiBody* _rob,
        ActiveMultiBody* _otherRobot, const string& _callName);

    /// Check collision between robot and one obstacle
    /// @param[out] _cdInfo CDInfo
    /// @param _rob ActiveMultiBody of robot
    /// @param _obst StaticMultiBody of obstacle
    /// @param _callName Function calling validity checker
    /// @return Collision
    bool IsInObstCollision(CDInfo& _cdInfo, ActiveMultiBody* _rob,
        StaticMultiBody* _obst, const string& _callName);

    CollisionDetectionMethod* m_cdMethod; ///< Collision Detection library
    bool m_ignoreSelfCollision;           ///< Check self collisions
    bool m_interRobotCollision{false};          ///< Check inter robot collisions
    size_t m_ignoreIAdjacentLinks;        ///< With self collisions how many
                                          ///< adjacent links to ignore
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
CollisionDetectionValidity<MPTraits>::
CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod,
    bool _ignoreSelfCollision, int _ignoreIAdjacentLinks) :
    ValidityCheckerMethod<MPTraits>(), m_cdMethod(_cdMethod),
    m_ignoreSelfCollision(_ignoreSelfCollision),
    m_ignoreIAdjacentLinks(_ignoreIAdjacentLinks) {
  this->SetName("CollisionDetection");
}


template <typename MPTraits>
CollisionDetectionValidity<MPTraits>::
CollisionDetectionValidity(XMLNode& _node) :
    ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("CollisionDetection");

  m_ignoreSelfCollision = _node.Read("ignoreSelfCollision", false, false,
      "Check for self collision");
  m_interRobotCollision = _node.Read("interRobotCollision", false, false,
      "Check for intern robot collision");
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


template <typename MPTraits>
CollisionDetectionValidity<MPTraits>::
~CollisionDetectionValidity() {
  delete m_cdMethod;
}

/*------------------------- ValidityChecker Interface ------------------------*/

template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) {
  this->GetStatClass()->IncCfgIsColl(_callName);

  //position environment
  _cfg.ConfigureRobot();

  //check collision
  bool inCollision = IsInCollision(_cdInfo, _cfg, _callName);

  _cfg.SetLabel("VALID", !inCollision);

  return !inCollision;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInCollision(CDInfo& _cdInfo, const CfgType& _cfg, const string& _callName) {
  _cdInfo.ResetVars(_cdInfo.m_retAllInfo);
  bool retVal = false;

  //get multiBody
  Environment* env = this->GetEnvironment();
  auto multiBody = _cfg.GetMultiBody();

  //check self collision
  if(!m_ignoreSelfCollision && multiBody->NumFreeBody() > 1 &&
      IsInSelfCollision(_cdInfo, multiBody, _callName)) {
    _cdInfo.m_collidingObstIndex = -1;
    return true;
  }

  // Check boundary collision.
  const bool inBounds = _cfg.InBounds(this->GetEnvironment());
  if(!inBounds) {
    _cdInfo.m_collidingObstIndex = -1;
    return true;
  }


  //check obstacle collisions
  size_t numObst = env->NumObstacles();
  for(size_t i = 0; i < numObst; ++i) {
    CDInfo cdInfo(_cdInfo.m_retAllInfo);
    bool coll = IsInObstCollision(
        cdInfo, multiBody, env->GetObstacle(i), _callName);

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
  if(m_interRobotCollision) {
    //cout << "Checking for inter robot collision " << endl;
    auto allRobots = this->GetMPProblem()->GetRobots();
    for(auto robot : allRobots) {
      if(_cfg.GetRobot() == robot or robot->IsVirtual())
        continue;

      CDInfo cdInfo(_cdInfo.m_retAllInfo);
      cout << "Checking for robot collision " << endl;
      cout << "Label: " << robot->GetLabel() << endl;
      cout << "Simulated position: " << robot->GetDynamicsModel()->GetSimulatedState() << endl;
      cout << "Multibody pos: ";
      for(auto dof : robot->GetMultiBody()->GetCurrentDOFs())
        cout << " " << dof;
      cout << endl;
      bool coll = IsInterRobotCollision(cdInfo, _cfg.GetRobot()->GetMultiBody(), robot->GetMultiBody(), _callName);
      if(coll) {
        cout << " Cfg in collision: " << _cfg << endl;
        return true;
      }
    }
  }
  return retVal;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInSelfCollision(CDInfo& _cdInfo, ActiveMultiBody* _rob,
    const string& _callName) {
  this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _callName);

  size_t numBody = _rob->NumFreeBody();
  for(size_t i = 0; i < numBody - 1; ++i) {
    for(size_t j = i+1; j < numBody; ++j) {
      auto body1 = _rob->GetFreeBody(i);
      auto body2 = _rob->GetFreeBody(j);

      if(body1->IsWithinI(body2, m_ignoreIAdjacentLinks))
        continue;

      if(m_cdMethod->IsInCollision(body1, body2, _cdInfo))
        return true;
    }
  }

  return false;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInterRobotCollision(CDInfo& _cdInfo, ActiveMultiBody* _rob,
    ActiveMultiBody* _otherRobot, const string& _callName) {
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


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInObstCollision(CDInfo& _cdInfo, ActiveMultiBody* _rob,
    StaticMultiBody* _obst, const string& _callName) {
  this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _callName);

  bool collision = false;
  size_t numBody = _rob->NumFreeBody();
  auto obst = _obst->GetFixedBody(0);

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


template <typename MPTraits>
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


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
WorkspaceVisibility(const Point3d& _a, const Point3d& _b) {
  // Generate the third point for the triangle with _a, _b as side
  Vector3d p;
  auto direction = (_b -_a).normalize();

  // If line segment is not parallel to z-axis
  if(abs(direction*Vector3d(0,0,1))< 0.9)
    p = _a + Vector3d(0,0,1e-8);
  else
    p = _a + Vector3d(0,1e-8,0);

  // Create a new free body with a nullptr as owning multibody
  FixedBody* lineBody = new FixedBody(nullptr);
  // Create body geometry with a single triangle
  GMSPolyhedron poly;
  poly.GetVertexList() = vector<Vector3d>{{_a[0], _a[1], _a[2]},
      {_b[0], _b[1], _b[2]}, p};
  poly.GetPolygonList() = vector<GMSPolygon>{GMSPolygon(0, 1, 2,
      poly.GetVertexList())};
  lineBody->SetPolyhedron(poly);

  // Default behaviour do not store the cd info
  CDInfo cdInfo;
  // Check collision of the triangle against every obstacle in the environment
  Environment* env = this->GetEnvironment();
  const size_t num = env->NumObstacles();

  bool visible = true;

  for(size_t i = 0; i < num; ++i) {
    if(m_cdMethod->IsInCollision(lineBody, env->GetObstacle(i)->GetFixedBody(0),
        cdInfo)) {
      visible = false;
      break;
    }
  }

  delete lineBody;
  return visible;
}

/*----------------------------------------------------------------------------*/

#endif
