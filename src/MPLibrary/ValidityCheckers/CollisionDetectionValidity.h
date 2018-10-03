#ifndef COLLISION_DETECTION_VALIDITY_H_
#define COLLISION_DETECTION_VALIDITY_H_

#include "ValidityCheckerMethod.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/Environment/Environment.h"

#include "Utilities/MetricUtils.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/RapidCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"

#include "nonstd/io.h"


////////////////////////////////////////////////////////////////////////////////
/// Validation here means collision-free. This class interfaces with external CD
/// libraries to determine collision information -- sometimes including
/// clearance and penetration information.
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class CollisionDetectionValidity : public ValidityCheckerMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfg;
    typedef std::vector<size_t>             Formation;

    ///@}
    ///@name Construction
    ///@{

    /// @param _cdMethod Collision detection library
    /// @param _ignoreSelfCollision Compute robot self-collisions?
    /// @param _ignoreAdjacentLinks For self-collision adjacent links to ignore
    CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod = nullptr,
        bool _ignoreSelfCollision = false, int _ignoreAdjacentLinks = 0,
        int _ignoreSiblingCollisions = 0,
        bool measureSelfDist = false);

    CollisionDetectionValidity(XMLNode& _node);

    virtual ~CollisionDetectionValidity();

    ///@}
    ///@name CollisionDetection Interface
    ///@{

    /// @return Collision Detection library
    CollisionDetectionMethod* GetCDMethod() const noexcept {return m_cdMethod;}

    /// Determine whether a workspace point lies inside of an obstacle.
    /// @param _p The workspace point.
    /// @return True if _p is inside an obstacle.
    bool IsInsideObstacle(const Point3d& _p);

    /// Check if two workspace points are mutually visible.
    /// @param _a The first point.
    /// @param _b The second point.
    /// @return True if _a is visible from _b and vice versa.
    virtual bool WorkspaceVisibility(const Point3d& _a, const Point3d& _b);

    /// Check inter-robot collision
    /// @param[out] _cdInfo CDInfo
    /// @param _rob MultiBody of robot
    /// @param _otherRobot MultiBody of the other robot to check
    /// @param _callName Function calling validity checker
    /// @return Collision
    bool IsMultiBodyCollision(CDInfo& _cdInfo, MultiBody* _rob,
                              MultiBody* _otherRobot, const string& _callName);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const string& _callName) override;

    virtual bool IsValidImpl(GroupCfg& _cfg, CDInfo& _cdInfo,
        const string& _callName, const Formation& _robotIndexes = Formation())
        override;

    /// Orchestrate collision computation between robot and environment
    /// multibodies
    /// @param[out] _cdInfo CDInfo
    /// @param _cfg Configuration of interest.
    /// @param _callName Function calling validity checker
    /// @return Collision
    virtual bool IsInCollision(CDInfo& _cdInfo, const CfgType& _cfg,
                               const string& _callName);

    /// Orchestrate collision computation between robots in a group cfg
    /// and environment multibodies
    /// @param[out] _cdInfo CDInfo
    /// @param _cfg Group configuration of interest.
    /// @param _callName Function calling validity checker
    /// @param _robotIndexes The robot indexes in this will be checked against
    ///              each of the indexes NOT in the vector. All-to-all if empty.
    /// @return Collision
    virtual bool IsInCollision(CDInfo& _cdInfo, const GroupCfg& _cfg,
                       const string& _callName, const Formation& _robotIndexes);

    /// Check self-collision with robot
    /// @param[out] _cdInfo CDInfo
    /// @param _rob MultiBody of robot
    /// @param _callName Function calling validity checker
    /// @return Collision
    virtual bool IsInSelfCollision(CDInfo& _cdInfo, MultiBody* _rob,
                                   const string& _callName);

    /// Check collision between robot and one obstacle
    /// @param[out] _cdInfo CDInfo
    /// @param _rob MultiBody of robot
    /// @param _obst MultiBody of obstacle
    /// @param _callName Function calling validity checker
    /// @return Collision
    virtual bool IsInObstCollision(CDInfo& _cdInfo, MultiBody* _rob,
                                   MultiBody* _obst, const string& _callName);

    ///@}
    ///@name Internal State
    ///@{

    CollisionDetectionMethod* m_cdMethod{nullptr}; ///< Collision Detection library

    bool m_ignoreSelfCollision{false};    ///< Check self collisions
    bool m_interRobotCollision{false};    ///< Check inter-robot collisions
    bool m_ignoreAdjacentLinks{false};    ///< Ignore adj links in self collisions
    bool m_ignoreSiblingCollisions{false};    ///< Ignore sibling links self collisions

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
CollisionDetectionValidity<MPTraits>::
CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod,
    bool _ignoreSelfCollision, int _ignoreAdjacentLinks,
    int _ignoreSiblingCollisions,bool measureSelfDist) :
  ValidityCheckerMethod<MPTraits>(), m_cdMethod(_cdMethod),
  m_ignoreSelfCollision(_ignoreSelfCollision),
  m_ignoreAdjacentLinks(_ignoreAdjacentLinks),
  m_ignoreSiblingCollisions(_ignoreSiblingCollisions) {
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
  m_ignoreAdjacentLinks = _node.Read("ignoreAdjacentLinks", false, false,
      "Ignore adjacent links in self-collision checks.");
  m_ignoreSiblingCollisions = _node.Read("ignoreSiblingCollisions", false, false,
      "Ignore links that share a parent in self-collision checks.");

  const std::string cdLabel = _node.Read("method", true, "", "method");

  if(cdLabel == "BoundingSpheres")
    m_cdMethod = new BoundingSpheres();
  else if(cdLabel == "InsideSpheres")
    m_cdMethod = new InsideSpheres();
  else if(cdLabel == "RAPID")
    m_cdMethod = new Rapid();
  else if(cdLabel == "PQP")
    m_cdMethod = new PQP();
  else if(cdLabel == "PQP_SOLID")
    m_cdMethod = new PQPSolid();
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
  const bool inCollision = IsInCollision(_cdInfo, _cfg, _callName);

  _cfg.SetLabel("VALID", !inCollision);

  return !inCollision;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsValidImpl(GroupCfg& _cfg, CDInfo& _cdInfo, const string& _callName,
            const Formation& _robotIndexes) {
  this->GetStatClass()->IncCfgIsColl(_callName);

  //position environment
  _cfg.ConfigureRobot();

  bool inCollision = true;

  //check collision
  if(this->m_debug) {
    std::cout << "Not using desired active robots " << _robotIndexes
              << " and instead checking ALL robots in debug mode." << std::endl;
    // Pass empty vector as that means check all robots exhaustively.
    inCollision = IsInCollision(_cdInfo, _cfg, _callName, {});
  }
  else {
    inCollision = IsInCollision(_cdInfo, _cfg, _callName, _robotIndexes);
  }

  return !inCollision;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInCollision(CDInfo& _cdInfo, const CfgType& _cfg, const string& _callName) {
  _cdInfo.ResetVars(_cdInfo.m_retAllInfo);

  const Environment* const env = this->GetEnvironment();
  auto multiBody = _cfg.GetMultiBody();
  auto robot = _cfg.GetRobot();

  //check self collision
  if(!m_ignoreSelfCollision and multiBody->GetNumBodies() > 1 and
      IsInSelfCollision(_cdInfo, multiBody, _callName)) {
    _cdInfo.m_collidingObstIndex = -1;

    if(this->m_debug)
      std::cout << "Self-collision detected:"
                << "\n\tCfg: " << _cfg.PrettyPrint()
                << "\n\tEnvironment boundary: "
                << *this->GetEnvironment()->GetBoundary()
                << std::endl;

    return true;
  }

  // Check boundary collision.
  const bool inBounds = env->UsingBoundaryObstacle()
                     or _cfg.InBounds(this->GetEnvironment());
  if(!inBounds) {
    _cdInfo.m_collidingObstIndex = -1;

    if(this->m_debug)
      std::cout << "Out-of-bounds detected:"
                << "\n\tCfg: " << _cfg.PrettyPrint()
                << "\n\tEnvironment boundary: "
                << *this->GetEnvironment()->GetBoundary()
                << std::endl;

    return true;
  }


  //check obstacle collisions
  bool inCollision = false;
  size_t numObst = env->NumObstacles();
  for(size_t i = 0; i < numObst; ++i) {
    CDInfo cdInfo(_cdInfo.m_retAllInfo);
    const bool coll = IsInObstCollision(
        cdInfo, multiBody, env->GetObstacle(i), _callName);

    //make sure to store closest obstacle information
    if(cdInfo < _cdInfo) {
      _cdInfo = cdInfo;
      _cdInfo.m_nearestObstIndex = i;
    }

    //store first collision in colliding index
    if(coll && !inCollision) {
      _cdInfo.m_collidingObstIndex = i;
      inCollision = true;
    }

    if(coll and this->m_debug)
      std::cout << "Obstacle collision detected:"
                << "\n\tCfg: " << _cfg.PrettyPrint()
                << "\n\tObstacle index: " << i
                << std::endl;

    //early quit if we don't care about distance information
    if(coll and !_cdInfo.m_retAllInfo)
      return true;
  }

  // Check against other robots if requested.
  if(m_interRobotCollision and !robot->IsVirtual()) {
    if(this->m_debug)
      std::cout << "Checking inter-robot collisions with robot "
                << robot->GetLabel() << std::endl;

    const auto& allRobots = this->GetMPProblem()->GetRobots();
    for(const auto& otherRobot : allRobots) {
      // Skip self-checks and checks against virtual robots.
      if(robot == otherRobot.get() or otherRobot->IsVirtual())
        continue;

      if(this->m_debug)
        std::cout << "Checking against robot " << otherRobot->GetLabel()
                  << std::endl;

      // Perform the collision check.
      CDInfo cdInfo(_cdInfo.m_retAllInfo);
      const bool collision = IsMultiBodyCollision(cdInfo,
          robot->GetMultiBody(), otherRobot->GetMultiBody(), _callName);

      if(collision) {
        if(this->m_debug)
          std::cout << "Inter-robot collision detected:"
                    << "\n\tRobot1: " << _cfg.GetRobot()->GetLabel()
                    << "\n\tCfg1: " << _cfg.PrettyPrint()
                    << "\n\tRobot2: " << otherRobot->GetLabel()
                    << "\n\tCfg2: " << otherRobot->GetMultiBody()->GetCurrentDOFs()
                    << std::endl;
        return true;
      }
    }
  }

  return inCollision;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInCollision(CDInfo& _cdInfo, const GroupCfg& _cfg, const string& _callName,
              const Formation& _robotIndexes) {
  _cdInfo.ResetVars(_cdInfo.m_retAllInfo);
  bool inCollision = false;

  Environment* const env = this->GetEnvironment();

  // Make a list of all other bodies (excluding m_bodyNumbers' entries from it)
  const size_t numBody = _cfg.GetNumRobots();
  Formation movingRobots, otherRobots(numBody);
  for(unsigned int i = 0; i < numBody; ++i)
    otherRobots[i] = i; //Initially populate with all bodies.

  const bool checkEveryBody = _robotIndexes.empty();
  if (checkEveryBody)
    movingRobots = otherRobots; // Do an all-to-all check
  else {
    //To optimize, remove any entry from otherBodies that is in _robotIndexes
    for(auto id : _robotIndexes)
      otherRobots.erase(remove(otherRobots.begin(), otherRobots.end(), id),
                        otherRobots.end());
    movingRobots = _robotIndexes;
  }

  // There are 3 stages to this:
  // 1) Check if any robots in m_activeBodies are in self-collision
  //    (if a robot has multiple bodies)
  // 2) Check if any robots in m_activeBodies are in collision with any
  //    robots NOT in m_activeBodies.
  // 3) Check if any robots in m_activeBodies are in collision with any
  //    obstacles in the environment.

  // 1) Check all active robots for self-collision (in case of linked
  //    group members).
  for(const size_t robotIndex : movingRobots) {
    MultiBody* const multiBody = _cfg.GetRobot(robotIndex)->GetMultiBody();
    // Check if any active robot is in self-collision, use parent's function.
    if(!this->m_ignoreSelfCollision and multiBody->GetNumBodies() > 1) {
      CDInfo tempInfo(_cdInfo.m_retAllInfo);
      if(this->IsInSelfCollision(tempInfo, multiBody, _callName)) {
        _cdInfo.m_collidingObstIndex = -1;

        if(tempInfo < _cdInfo)
          _cdInfo = tempInfo;

        if(!_cdInfo.m_retAllInfo)
          return true;

        inCollision = true;
      }
    }
  }

  // 2) Now check all desired robots against all others:
  if(this->m_debug)
    std::cout << "Going to do an all-to-all CD check between the following 2 "
              "sets of robot bodies:" << std::endl << "Moving bodies: "
              << movingRobots << "Other bodies: " << otherRobots << std::endl;

  // Create a self-clearance vector to track collision info with other robots.
  /// @todo Replace our disjoint collison storage with something that works for
  ///       all collision types.
  std::vector<double> selfClearance(_cfg.GetNumRobots(),
      numeric_limits<double>::max());

  for(const size_t i : movingRobots) {
    MultiBody* const movingMultiBody = _cfg.GetRobot(i)->GetMultiBody();
    for(const size_t j : otherRobots) {
      if(checkEveryBody and i >= j)
        continue; // This prevents redundant CDs if doing all-to-all.

      MultiBody* const otherMultiBody = _cfg.GetRobot(j)->GetMultiBody();
      CDInfo tempInfo(_cdInfo.m_retAllInfo);

      // Use parent's multibody collision check:
      if(this->IsMultiBodyCollision(tempInfo, movingMultiBody, otherMultiBody,
                                    _callName)) {
        _cdInfo.m_collidingObstIndex = -1;
        if(!_cdInfo.m_retAllInfo) {
          selfClearance[j] = tempInfo.m_minDist;
          _cdInfo.m_selfClearance = std::move(selfClearance);
          return true; // Short-circuit if we don't need all (clearance) info.
        }

        inCollision = true;
      }

      // Keep track of same-group distances:
      selfClearance[j] = std::min(selfClearance[j], tempInfo.m_minDist);

      // Whether or not there was a collision, check if clearances need updating
      if(tempInfo < _cdInfo)
        _cdInfo = tempInfo;
    }
  }

  // 3) Check obstacle collisions
  const size_t numObst = env->NumObstacles();
  for(const size_t robotIndex : movingRobots) {
    MultiBody* const multiBody = _cfg.GetRobot(robotIndex)->GetMultiBody();
    for(size_t i = 0; i < numObst; ++i) {
      CDInfo cdInfo(_cdInfo.m_retAllInfo);
      const bool coll = this->IsMultiBodyCollision(cdInfo, multiBody,
                                                env->GetObstacle(i), _callName);

      // Make sure to store closest obstacle information if that was asked for in
      // the obstacle CD check.
      if(_cdInfo.m_retAllInfo and cdInfo < _cdInfo) {
        _cdInfo = cdInfo;
        _cdInfo.m_nearestObstIndex = i;
      }

      if(coll) {
        //store first collision in colliding index
        if(!inCollision) {
          _cdInfo.m_collidingObstIndex = i;
          inCollision = true;
        }

        //early quit if we don't care about distance information
        if(!_cdInfo.m_retAllInfo)
        {
          _cdInfo.m_selfClearance = std::move(selfClearance);
          return true;
      }
    }
  }
  }

  _cdInfo.m_selfClearance = selfClearance;
  return inCollision;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInSelfCollision(CDInfo& _cdInfo, MultiBody* _rob,
                  const string& _callName) {
  this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _callName);

  bool collision = false;
  size_t numBody = _rob->GetNumBodies();
  for(size_t i = 0; i < numBody - 1; ++i) {
    for(size_t j = i + 1; j < numBody; ++j) {
      auto body1 = _rob->GetBody(i);
      auto body2 = _rob->GetBody(j);

      if(m_ignoreAdjacentLinks and body1->IsAdjacent(body2))
        continue;

      if(m_ignoreSiblingCollisions and body1->SameParent(body2))
        continue;

      if (!_cdInfo.m_retAllInfo) {
        if(m_cdMethod->IsInCollision(body1, body2, _cdInfo)) {
          return true;
        }
      }
      else {
        CDInfo cdInfo(_cdInfo.m_retAllInfo);
        bool collisionFound = m_cdMethod->IsInCollision(body1, body2, cdInfo);

        //retain minimum distance information
        if(cdInfo < _cdInfo)
          _cdInfo = cdInfo;

        if(collisionFound)
          collision = true;
      }
    }
  }

  return collision;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsMultiBodyCollision(CDInfo& _cdInfo, MultiBody* _rob,
                     MultiBody* _otherRobot, const string& _callName) {
  this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _callName);

  size_t numBody = _rob->GetNumBodies();
  size_t numOtherBody = _otherRobot->GetNumBodies();

  bool collision = false;
  for(size_t i = 0; i < numBody; ++i) {
    for(size_t j = 0; j < numOtherBody; ++j) {
      CDInfo tempInfo(_cdInfo.m_retAllInfo);
      const bool collisionFound = m_cdMethod->IsInCollision(_rob->GetBody(i),
                                            _otherRobot->GetBody(j), tempInfo);

      if(tempInfo < _cdInfo)
        _cdInfo = tempInfo; // Retain only the minimum clearance info.

      if(collisionFound) {
        if(!_cdInfo.m_retAllInfo)
          return true; // Short-circuit CD calls if all info isn't needed.

        collision = true;
      }
    }
  }
  return collision;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInObstCollision(CDInfo& _cdInfo, MultiBody* _rob,
                  MultiBody* _obst, const string& _callName) {
  this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _callName);

  bool collision = false;
  const size_t numBody = _rob->GetNumBodies();

  for(size_t i = 0; i < numBody; ++i) {
    CDInfo cdInfo(_cdInfo.m_retAllInfo);
    for(unsigned int j = 0; j < _obst->GetNumBodies(); ++j) {
      collision |= m_cdMethod->IsInCollision(_rob->GetBody(i),
                                             _obst->GetBody(j),
                                             cdInfo);

      // Retain minimum distance information.
      if(cdInfo < _cdInfo)
        _cdInfo = cdInfo; // Retain minimum distance information

      // Early quit if we do not care for distance information.
      if(collision and !_cdInfo.m_retAllInfo)
        return true;
    }
  }

  return collision;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInsideObstacle(const Point3d& _p) {
  /// @todo Implement a bounding box check (per multibody and body) before
  ///       calling m_cdMethod.

  auto env = this->GetEnvironment();
  const size_t numObstacles = env->NumObstacles();

  // Check each obstacle.
  for(size_t i = 0; i < numObstacles; ++i) {
    auto obst = env->GetObstacle(i);

    // Check each body.
    for(size_t j = 0; j < obst->GetNumBodies(); ++j)
      if(m_cdMethod->IsInsideObstacle(_p, obst->GetBody(j)))
        return true;
  }

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
  Body lineBody(nullptr);
  // Create body geometry with a single triangle
  GMSPolyhedron poly;
  poly.GetVertexList() = vector<Vector3d>{{_a[0], _a[1], _a[2]},
      {_b[0], _b[1], _b[2]}, p};
  poly.GetPolygonList() = vector<GMSPolygon>{GMSPolygon(0, 1, 2,
      poly.GetVertexList())};
  lineBody.SetPolyhedron(std::move(poly));

  // Default behaviour do not store the cd info
  CDInfo cdInfo;
  // Check collision of the triangle against every obstacle in the environment
  Environment* env = this->GetEnvironment();
  const size_t num = env->NumObstacles();

  bool visible = true;

  for(size_t i = 0; i < num; ++i) {
    auto obst = env->GetObstacle(i);
    for(size_t j = 0; j < obst->GetNumBodies(); ++j) {
      if(m_cdMethod->IsInCollision(&lineBody, obst->GetBody(j), cdInfo)) {
        visible = false;
        break;
      }
    }
  }

  return visible;
}

/*----------------------------------------------------------------------------*/

#endif
