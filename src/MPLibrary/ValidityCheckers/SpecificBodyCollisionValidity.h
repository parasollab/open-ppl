#ifndef SPECIFIC_BODY_COLLISION_VALIDITY_H_
#define SPECIFIC_BODY_COLLISION_VALIDITY_H_

#include "CollisionDetectionValidity.h"
#include "nonstd.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// Validation here means collision-free. This class interfaces with external CD
/// libraries to determing collision information -- sometimes including
/// clearance and penetration information.
/// This is specifically made for (Dis)assembly planning code.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class SpecificBodyCollisionValidity : public CollisionDetectionValidity<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename std::vector<unsigned int> Subassembly;

    ///@}
    ///@name Construction
    ///@{

    /// @param _cdMethod Collision detection library
    /// @param _ignoreSelfCollision Compute robot self-collisions?
    /// @param _ignoreAdjacentLinks For self-collision adjacent links to ignore
    SpecificBodyCollisionValidity(CollisionDetectionMethod* _cdMethod = nullptr,
        bool _ignoreSelfCollision = false, int _ignoreAdjacentLinks = 1,
        bool measureSelfDist = false);

    SpecificBodyCollisionValidity(XMLNode& _node);

    virtual ~SpecificBodyCollisionValidity();

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
            const string& _callName, const vector<unsigned int> &_bodyNumbers);

    void SetBodyNumbers(const std::vector<unsigned int> &_bodyNumbers =
                                                   std::vector<unsigned int>());

    const vector<unsigned int>& GetBodyNumbers() {return m_bodyNumbers;}

    ///@}

  private:

    /// Orchestrate collision computation between robot and environment
    /// multibodies
    /// @param[out] _cdInfo CDInfo
    /// @param _cfg Configuration of interest.
    /// @param _callName Function calling validity checker
    /// @return Collision
    virtual bool IsInCollision(CDInfo& _cdInfo, const CfgType& _cfg,
        const string& _callName) override;

    /// Check self-collision with robot
    /// @param[out] _cdInfo CDInfo
    /// @param _rob MultiBody of robot
    /// @param _callName Function calling validity checker
    /// @return Collision
    virtual bool IsInSelfCollision(CDInfo& _cdInfo, MultiBody* _rob,
        const string& _callName) override;

    /// Check collision between robot and one obstacle
    /// @param[out] _cdInfo CDInfo
    /// @param _rob MultiBody of robot
    /// @param _obst MultiBody of obstacle
    /// @param _callName Function calling validity checker
    /// @return Collision
    virtual bool IsInObstCollision(CDInfo& _cdInfo, MultiBody* _rob,
        MultiBody* _obst, const string& _callName) override;

    /// The body numbers represent which bodies don't need to be checked, with
    /// respect to each other. This means that each body included will be
    /// checked against all those not included and all obstacles.
    Subassembly m_bodyNumbers;
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
SpecificBodyCollisionValidity<MPTraits>::
SpecificBodyCollisionValidity(CollisionDetectionMethod* _cdMethod,
    bool _ignoreSelfCollision, int _ignoreAdjacentLinks, bool measureSelfDist) :
    CollisionDetectionValidity<MPTraits>(_cdMethod, _ignoreSelfCollision,
        _ignoreAdjacentLinks, measureSelfDist) {
  this->SetName("SpecificBodyCollisionValidity");
}


template <typename MPTraits>
SpecificBodyCollisionValidity<MPTraits>::
SpecificBodyCollisionValidity(XMLNode& _node) : CollisionDetectionValidity<MPTraits>(_node) {
  this->SetName("SpecificBodyCollisionValidity");
}


template <typename MPTraits>
SpecificBodyCollisionValidity<MPTraits>::
~SpecificBodyCollisionValidity() {
}

/*------------------------- ValidityChecker Interface ------------------------*/

template <typename MPTraits>
bool
SpecificBodyCollisionValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName,
            const vector<unsigned int>& _bodyNumbers) {
  SetBodyNumbers(_bodyNumbers);
  return IsValidImpl(_cfg, _cdInfo, _callName);
}

template <typename MPTraits>
void
SpecificBodyCollisionValidity<MPTraits>::
SetBodyNumbers(const std::vector<unsigned int>& _bodyNumbers) {
  if(this->m_debug) {
    std::cout << "Debug is true. Clearing m_bodyNumbers in VC." << std::endl;
    m_bodyNumbers.clear();
  }
  else
    m_bodyNumbers = _bodyNumbers; // Standard behavior (to optimize CD checks).
}

template <typename MPTraits>
bool
SpecificBodyCollisionValidity<MPTraits>::
IsInCollision(CDInfo& _cdInfo, const CfgType& _cfg, const string& _callName) {
  _cdInfo.ResetVars(_cdInfo.m_retAllInfo);
  bool inCollision = false;

  //get multiBody
  Environment* const env = this->GetEnvironment();
  auto multiBody = _cfg.GetMultiBody();

  if(this->m_debug) {
    std::cout << "IsInCollision: _cfg = " << _cfg.PrettyPrint() << std::endl;
    for(size_t i = 0; i < multiBody->GetNumBodies(); ++i)
      std::cout << "body" << i << " local centroid: " << multiBody->GetBody(i)->
                GetPolyhedron().GetCentroid() << std::endl
                << "body" << i << " world centroid: " << multiBody->GetBody(i)->
                GetWorldPolyhedron().GetCentroid() << std::endl
                << "body" << i << " first world vertex: " << multiBody->GetBody(i)->
                GetWorldPolyhedron().GetVertexList().at(0) << std::endl << std::endl;
    std::cout << std::endl;
  }

  _cdInfo.m_selfClearance.resize(multiBody->GetNumBodies(), numeric_limits<double>::max());

  //check self collision
  if(!this->m_ignoreSelfCollision && multiBody->GetNumBodies() > 1 &&
      IsInSelfCollision(_cdInfo, multiBody, _callName)) {
    _cdInfo.m_collidingObstIndex = -1;

    //If we are returning all info, then we want to check if an obstacle was hit,
    // for later subassembly identification
    if(!_cdInfo.m_retAllInfo)
      return true;

    inCollision = true;
  }

  //check obstacle collisions
  const size_t numObst = env->NumObstacles();
  for(size_t i = 0; i < numObst; ++i) {
    CDInfo cdInfo(_cdInfo.m_retAllInfo);
    bool coll = IsInObstCollision(cdInfo, multiBody, env->GetObstacle(i), _callName);

    //make sure to store closest obstacle information
    if(cdInfo < _cdInfo) {
      _cdInfo.m_minDist = cdInfo.m_minDist;
      _cdInfo.m_nearestObstIndex = i;
    }

    //store first collision in colliding index
    if(coll && !inCollision) {
      _cdInfo.m_collidingObstIndex = i;
      inCollision = true;
    }

    //early quit if we don't care about distance information
    if(coll && !_cdInfo.m_retAllInfo)
      return true;
  }

  return inCollision;
}


template <typename MPTraits>
bool
SpecificBodyCollisionValidity<MPTraits>::
IsInSelfCollision(CDInfo& _cdInfo, MultiBody* _rob,
                  const string& _callName) {
  this->GetStatClass()->IncNumCollDetCalls(this->m_cdMethod->GetName(),
                                           _callName);
  //Make a list of all other bodies (excluding m_bodyNumbers' entries from it)
  const size_t numBody = _rob->GetNumBodies();
  std::vector<unsigned int> movingBodies, otherBodies(numBody);
  for(unsigned int i = 0; i < numBody; ++i)
    otherBodies[i] = i; //Initially populate with all bodies.

  const bool checkEveryBody = m_bodyNumbers.empty();
  if (checkEveryBody)
    movingBodies = otherBodies; // Do an all-to-all check
  else {
    //Remove any entry from otherBodies that is in m_bodyNumbers:
    for(auto id : m_bodyNumbers)
      otherBodies.erase(remove(otherBodies.begin(), otherBodies.end(), id),
                        otherBodies.end());
    movingBodies = m_bodyNumbers;
  }

  if(this->m_debug)
    std::cout << "Going to do an all-to-all CD check between the following 2 "
              "sets of robot bodies:" << std::endl << "Moving bodies: "
              << movingBodies << "Other bodies: " << otherBodies << std::endl;

  bool collision = false;
  _cdInfo.m_selfClearance.resize(numBody, numeric_limits<double>::max());
  for(const unsigned int i : movingBodies) {
    const Body* const body1 = _rob->GetBody(i);
    for(const unsigned int j : otherBodies) {
      const Body* const body2 = _rob->GetBody(j);

      if((checkEveryBody and i >= j) or
         (this->m_ignoreAdjacentLinks and body1->IsAdjacent(body2)))
        continue; // This prevents redundant and adjacent CDs as needed.

      if (!_cdInfo.m_retAllInfo) {
        if(this->m_cdMethod->IsInCollision(body1, body2, _cdInfo))
          return true;
      }
      else {
        CDInfo cdInfo(_cdInfo.m_retAllInfo);
        const bool collisionFound =
                          this->m_cdMethod->IsInCollision(body1, body2, cdInfo);

        //retain minimum distance information
        if(cdInfo < _cdInfo)
          _cdInfo.m_minDist = cdInfo.m_minDist;

        _cdInfo.m_selfClearance[j] = cdInfo.m_minDist;

        if(collisionFound)
          collision = true;
      }
    }
  }

  return collision;
}

template <typename MPTraits>
bool
SpecificBodyCollisionValidity<MPTraits>::
IsInObstCollision(CDInfo& _cdInfo, MultiBody* _rob,
                  MultiBody* _obst, const string& _callName) {
  this->GetStatClass()->IncNumCollDetCalls(this->m_cdMethod->GetName(),
                                           _callName);
  const size_t numBody = _rob->GetNumBodies();
  std::vector<unsigned int> movingBodies(numBody);
  if (m_bodyNumbers.empty())
    for(unsigned int i = 0; i < numBody; ++i)
      movingBodies[i] = i; //Check all bodies if m_bodyNumbers is empty
  else
    movingBodies = m_bodyNumbers; //Only check bodies specified

  bool collision = false;
  for(const unsigned int i : movingBodies) {
    CDInfo cdInfo(_cdInfo.m_retAllInfo);
    bool coll = false;
    for(unsigned int j = 0; j < _obst->GetNumBodies(); ++j) {
      coll = this->m_cdMethod->IsInCollision(
                           _rob->GetBody(i), _obst->GetBody(j), cdInfo) || coll;

      //retain minimum distance information
      if(cdInfo < _cdInfo)
        _cdInfo.m_minDist = cdInfo.m_minDist;

      //Early quit if we do not care for distance information
      if(coll) {
        if(!_cdInfo.m_retAllInfo)
          return true;
        collision = true;
      }
    }
  }

  return collision;
}

/*----------------------------------------------------------------------------*/

#endif
