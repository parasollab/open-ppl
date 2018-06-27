#ifndef GROUP_TWO_ROBOT_VALIDITY_CHECKER_H_
#define GROUP_TWO_ROBOT_VALIDITY_CHECKER_H_

#include "CollisionDetectionValidity.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// Validation here means collision-free. This class interfaces with external CD
/// libraries to determing collision information -- sometimes including
/// clearance and penetration information.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupTwoRobotValidityChecker : public CollisionDetectionValidity<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::GroupCfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    /// @param _cdMethod Collision detection library
    /// @param _ignoreSelfCollision Compute robot self-collisions?
    /// @param _ignoreIAdjacentLinks For self-collision adjacent links to ignore
    GroupTwoRobotValidityChecker(CollisionDetectionMethod* _cdMethod = nullptr,
        bool _ignoreSelfCollision = false, int _ignoreIAdjacentLinks = 1);

    GroupTwoRobotValidityChecker(XMLNode& _node);

    virtual ~GroupTwoRobotValidityChecker();

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
                     const string& _callName, const unsigned int firstBody,
                     const unsigned int secondBody);

    void SetRobots(const unsigned int _firstBody,
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
               const string& _callName,
               const std::vector<size_t>& _robotIndexes = std::vector<size_t>())
               override;


    unsigned int m_firstRobot{0};
    unsigned int m_secondRobot{0};
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
GroupTwoRobotValidityChecker<MPTraits>::
GroupTwoRobotValidityChecker(CollisionDetectionMethod* _cdMethod,
    bool _ignoreSelfCollision, int _ignoreIAdjacentLinks) :
    CollisionDetectionValidity<MPTraits>(_cdMethod, _ignoreSelfCollision,
        _ignoreIAdjacentLinks) {
  this->SetName("GroupTwoRobotValidityChecker");
}


template <typename MPTraits>
GroupTwoRobotValidityChecker<MPTraits>::
GroupTwoRobotValidityChecker(XMLNode& _node) : CollisionDetectionValidity<MPTraits>(_node) {
  this->SetName("GroupTwoRobotValidityChecker");
}


template <typename MPTraits>
GroupTwoRobotValidityChecker<MPTraits>::
~GroupTwoRobotValidityChecker() { }

/*------------------------- ValidityChecker Interface ------------------------*/

template <typename MPTraits>
bool
GroupTwoRobotValidityChecker<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName,
            const unsigned int firstBody, const unsigned int secondBody) {
  SetRobots(firstBody, secondBody);
  return IsValidImpl(_cfg, _cdInfo, _callName);
}


template <typename MPTraits>
void
GroupTwoRobotValidityChecker<MPTraits>::
SetRobots(const unsigned int _firstBody, const unsigned int _secondBody) {
  if (_firstBody == _secondBody)
    throw RunTimeException(WHERE, "Both are the same robot.");
  m_firstRobot = _firstBody;
  m_secondRobot = _secondBody;
}


template <typename MPTraits>
bool
GroupTwoRobotValidityChecker<MPTraits>::
IsInCollision(CDInfo& _cdInfo, const CfgType& _cfg, const string& _callName,
              const std::vector<size_t>& _robotIndexes) {
  _cdInfo.ResetVars(_cdInfo.m_retAllInfo);

  // Just use parent's robot-wise function.
  return this->IsMultiBodyCollision(_cdInfo,
                       _cfg.GetRobot(m_firstRobot)->GetMultiBody(),
                       _cfg.GetRobot(m_secondRobot)->GetMultiBody(), _callName);
}

#endif
