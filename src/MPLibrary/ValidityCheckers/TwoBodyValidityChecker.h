#ifndef TWO_BODY_COLLISION_CHECKER_H_
#define TWO_BODY_COLLISION_CHECKER_H_

#include "CollisionDetectionValidity.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// Validation here means collision-free. This class interfaces with external CD
/// libraries to determing collision information -- sometimes including
/// clearance and penetration information.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TwoBodyValidityChecker : public CollisionDetectionValidity<MPTraits> {

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
    TwoBodyValidityChecker(CollisionDetectionMethod* _cdMethod = nullptr,
        bool _ignoreSelfCollision = false, int _ignoreIAdjacentLinks = 1);

    TwoBodyValidityChecker(XMLNode& _node);

    virtual ~TwoBodyValidityChecker();

    void Initialize();

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
            const string& _callName, const unsigned int firstBody,
            const unsigned int secondBody);

    void SetBodyNumbers(const unsigned int _firstBody,
                        const unsigned int _secondBody);

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

    unsigned int m_firstBody = 0;
    unsigned int m_secondBody = 0;
    size_t m_numBodies = 0;
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
TwoBodyValidityChecker<MPTraits>::
TwoBodyValidityChecker(CollisionDetectionMethod* _cdMethod,
    bool _ignoreSelfCollision, int _ignoreIAdjacentLinks) :
    CollisionDetectionValidity<MPTraits>(_cdMethod, _ignoreSelfCollision,
        _ignoreIAdjacentLinks) {
  this->SetName("TwoBodyValidityChecker");
}


template <typename MPTraits>
TwoBodyValidityChecker<MPTraits>::
TwoBodyValidityChecker(XMLNode& _node) : CollisionDetectionValidity<MPTraits>(_node) {
  this->SetName("TwoBodyValidityChecker");
}


template <typename MPTraits>
TwoBodyValidityChecker<MPTraits>::
~TwoBodyValidityChecker() {
}

template <typename MPTraits>
void
TwoBodyValidityChecker<MPTraits>::
Initialize() {
  m_numBodies = this->GetMPProblem()->GetRobot(0)->GetMultiBody()->GetNumBodies();
}

/*------------------------- ValidityChecker Interface ------------------------*/

template <typename MPTraits>
bool
TwoBodyValidityChecker<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName,
    const unsigned int firstBody, const unsigned int secondBody) {
  SetBodyNumbers(firstBody, secondBody);
  return IsValidImpl(_cfg, _cdInfo, _callName);
}

template <typename MPTraits>
void
TwoBodyValidityChecker<MPTraits>::
SetBodyNumbers(const unsigned int _firstBody, const unsigned int _secondBody) {
  if (_firstBody >= m_numBodies || _secondBody >= m_numBodies ||
      _firstBody == _secondBody) {
    if(this->m_debug)
      std::cout << this->GetNameAndLabel() << "A body number is too large, or "
                                        "both are the same body." << std::endl;
    return;
  }
  m_firstBody = _firstBody;
  m_secondBody = _secondBody;
}

template <typename MPTraits>
bool
TwoBodyValidityChecker<MPTraits>::
IsInCollision(CDInfo& _cdInfo, const CfgType& _cfg, const string& _callName) {
  _cdInfo.ResetVars(_cdInfo.m_retAllInfo);
  MultiBody* const mb = _cfg.GetMultiBody();

  //check self collision
  if(IsInSelfCollision(_cdInfo, mb, _callName)) {
    _cdInfo.m_collidingObstIndex = -1;
    return true;
  }

  return false;
}

template <typename MPTraits>
bool
TwoBodyValidityChecker<MPTraits>::
IsInSelfCollision(CDInfo& _cdInfo, MultiBody* _rob,
    const string& _callName) {
  this->GetStatClass()->IncNumCollDetCalls(this->m_cdMethod->GetName(),
                                           _callName);
  // Assume that the bodies are not equal.
  const bool col = this->m_cdMethod->IsInCollision(_rob->GetBody(m_firstBody),
                                        _rob->GetBody(m_secondBody), _cdInfo);
  return col;
}

#endif
