#ifndef PMPL_VALIDITY_CHECKER_METHOD_H_
#define PMPL_VALIDITY_CHECKER_METHOD_H_

#include "MPLibrary/MPBaseObject.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MPUtils.h"

#include <string>


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref ValidityCheckers.
///
/// Validity checkers define some notion of 'valid' for a configuration. The
/// IsValid function evaluates a configuration with respect to this definition.
/// The output may be inverted by calling ToggleValidity.
///
/// For collision-detection type checkers, a CDInfo object may carry additional
/// information about the check.
///
/// @usage
/// @code
/// ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
/// CfgType c;
/// CDInfo cdInfo;
/// string callee("SomeFunc");
/// bool valid = vc->IsValid(c, cdInfo, callee);
/// @endcode
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ValidityCheckerMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::GroupCfgType  GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::map<Robot*, std::vector<Robot*>> RobotMap;

    ///@}
    ///@name Construction
    ///@{

    ValidityCheckerMethod() = default;

    ValidityCheckerMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {}

    virtual ~ValidityCheckerMethod() = default;

    ///@}
    ///@name Validity Accessors
    ///@{

    /// Get the current meaning of "valid" (true is default).
    bool GetValidity() const;

    /// Switches the meaning of "valid" to "invalid" and vice versa.
    void ToggleValidity();

    virtual void SetLocalBoundary(Boundary* _boundary) {};

    virtual void SetLocalBoundaries(std::map<Robot*,Boundary*> _boundaries) {};


    ///@}
    ///@name Individual Configuration Validity
    ///@{

    /// Classify a configuration to either cfree or cobst.
    /// @param _cfg The individual configuration.
    /// @param _cdInfo Output for extra computed information such as clearance.
    /// @param _caller Name of the calling function.
    /// @return True iff _cfg is in cfree, false otherwise.
    bool IsValid(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller);
    ///@example ValidityCheckers_UseCase.cpp
    /// This is an example of how to use the validity checker methods.

    /// This version does not return extra information.
    /// @overload
    bool IsValid(CfgType& _cfg, const std::string& _caller);

    ///@}
    ///@name Group Configuration Validity
    ///@{

    /// Classify a group configuration to either cfree or cobst.
    /// @param _cfg The group configuration.
    /// @param _cdInfo Output for extra computed information such as clearance.
    /// @param _caller Name of the calling function.
    /// @return True iff _cfg is in cfree, false otherwise.
    bool IsValid(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller);

    /// This version does not return extra information.
    /// @overload
    bool IsValid(GroupCfgType& _cfg, const std::string& _caller);

    bool IsValid(GroupCfgType& _cfg, const RobotMap& _robotMap,
        CDInfo& _cdInfo, const std::string& _caller);

    /// @overload
    bool IsValid(GroupCfgType& _cfg, const RobotMap& _robotMap, const std::string& _caller);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Implementation of the classification of a configuration to either cfree
    /// or cobst.
    /// @param _cfg The individual configuration.
    /// @param _cdInfo Output for extra computed information such as clearance.
    /// @param _caller Name of the calling function.
    /// @return True if _cfg is in cfree.
    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _caller) = 0;

    /// Implementation of group cfg classification.
    /// @param _cfg The group configuration.
    /// @param _cdInfo Output for extra computed information such as clearance.
    /// @param _caller Name of the calling function.
    /// @return True if _cfg is in cfree.
    virtual bool IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _caller);

    virtual bool IsValidImpl(GroupCfgType& _cfg, const RobotMap& _robotMap,
        CDInfo& _cdInfo, const std::string& _caller);

    virtual bool IsValidImpl(GroupCfgType& _cfg, Robot* _robot, std::vector<Robot*> _robots,
        CDInfo& _cdInfo, const std::string& _caller);

    ///@}
    ///@name Internal State
    ///@{

    bool m_validity{true}; ///< Use standard validity? False indicates negation.

    ///@}

};

/*------------------------ Validity Checker Interface ------------------------*/

template <typename MPTraits>
inline
bool
ValidityCheckerMethod<MPTraits>::
GetValidity() const {
  return m_validity;
}


template <typename MPTraits>
inline
void
ValidityCheckerMethod<MPTraits>::
ToggleValidity() {
  m_validity = !m_validity;
}


template <typename MPTraits>
inline
bool
ValidityCheckerMethod<MPTraits>::
IsValid(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  return m_validity == IsValidImpl(_cfg, _cdInfo, _caller);
}


template <typename MPTraits>
inline
bool
ValidityCheckerMethod<MPTraits>::
IsValid(CfgType& _cfg, const std::string& _caller) {
  CDInfo cdInfo;
  return IsValid(_cfg, cdInfo, _caller);
}


template <typename MPTraits>
inline
bool
ValidityCheckerMethod<MPTraits>::
IsValid(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  return m_validity == IsValidImpl(_cfg, _cdInfo, _caller);
}


template <typename MPTraits>
inline
bool
ValidityCheckerMethod<MPTraits>::
IsValid(GroupCfgType& _cfg, const std::string& _caller) {
  CDInfo cdInfo;
  // cdInfo.m_retAllInfo = true;
  return IsValid(_cfg, cdInfo, _caller);
}


template <typename MPTraits>
inline
bool
ValidityCheckerMethod<MPTraits>::
IsValid(GroupCfgType& _cfg, const RobotMap& _robotMap,
        CDInfo& _cdInfo, const std::string& _caller) {
  return m_validity == IsValidImpl(_cfg, _robotMap, _cdInfo, _caller);
}

template <typename MPTraits>
inline
bool
ValidityCheckerMethod<MPTraits>::
IsValid(GroupCfgType& _cfg, const RobotMap& _robotMap, const std::string& _caller) {
  CDInfo cdInfo;
  return IsValid(_cfg, _robotMap, cdInfo, _caller);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
ValidityCheckerMethod<MPTraits>::
IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  for(auto robot : _cfg.GetRobots()) {
    if(!IsValidImpl(_cfg.GetRobotCfg(robot),_cdInfo,_caller))
      return false;
  }
  return true;
}


template <typename MPTraits>
bool
ValidityCheckerMethod<MPTraits>::
IsValidImpl(GroupCfgType& _cfg, const RobotMap& _robotMap, CDInfo& _cdInfo, 
  const std::string& _caller) {

  for(auto const& iter : _robotMap) {
    if(!IsValidImpl(_cfg, iter.first, iter.second, _cdInfo, _caller))
      return false;
  }
  return true;
}


template <typename MPTraits>
bool
ValidityCheckerMethod<MPTraits>::
IsValidImpl(GroupCfgType& _cfg, Robot* _robot, std::vector<Robot*> _robots,
  CDInfo& _cdInfo, const std::string& _caller) {

  throw NotImplementedException(WHERE) << "Not Implemented";
}

/*----------------------------------------------------------------------------*/

#endif