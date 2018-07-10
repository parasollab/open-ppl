#ifndef VALIDITY_CHECKER_METHOD_H
#define VALIDITY_CHECKER_METHOD_H

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
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ValidityCheckerMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::GroupCfgType  GroupCfgType;
    typedef typename GroupCfgType::Formation Formation;

    ///@}
    ///@name Construction
    ///@{

    ValidityCheckerMethod() = default;

    ValidityCheckerMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {}

    virtual ~ValidityCheckerMethod() = default;

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    /// Get the current meaning of "valid" (true is default).
    bool GetValidity() const;

    /// Switches the meaning of "valid" to "invalid" and vice versa.
    void ToggleValidity();

    /// Classify a configuration to either @cfree or @cobst. No extra collision
    /// information is returned.
    /// @overload
    bool IsValid(CfgType& _cfg, const std::string& _callName);

    /// Classify a GroupCfg in the same manner.
    /// @overload
    bool IsValid(GroupCfgType& _cfg, const std::string& _callName,
                 const Formation& _robotIndexes = Formation());

    /// Classify a configuration to either @cfree or @cobst.
    /// @param _cfg Configuration
    /// @param _cdInfo Extra computed information, e.g., minimum dist to
    ///        obstacle
    /// @param _callName Function caller for statistics tracking
    /// @return boolean valid/invalid. Valid (true) implies @cfree.
    ///
    /// @usage
    /// @code
    /// ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
    /// CfgType c;
    /// CDInfo cdInfo;
    /// string callee("SomeFunc");
    /// bool valid = vc->IsValid(c, cdInfo, callee);
    /// @endcode
    bool IsValid(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName);

    /// Classify a GroupCfg in the same manner.
    /// @overload
    bool IsValid(GroupCfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName,
        const Formation& _robotIndexes = Formation());

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Implementation of the classification of a configuration to either @cfree
    /// or @cobst
    /// @param _cfg Configuration
    /// @param _cdInfo Extra computed information, e.g., minimum dist to
    ///        obstacle
    /// @param _callName Function caller for statistics tracking
    /// @return boolean valid/invalid. Valid (true) implies @cfree.
    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) = 0;

    /// We don't want to require VCs implement group behavior, but want the
    /// default to be an exception thrown.
    virtual bool IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName, const Formation& _robotIndexes = 0);

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
IsValid(CfgType& _cfg, const std::string& _callName) {
  CDInfo cdInfo;
  return IsValid(_cfg, cdInfo, _callName);
}


template <typename MPTraits>
inline
bool
ValidityCheckerMethod<MPTraits>::
IsValid(GroupCfgType& _cfg, const std::string& _callName,
    const Formation& _robotIndexes) {
  CDInfo cdInfo;
  return IsValid(_cfg, cdInfo, _callName, _robotIndexes);
}


template <typename MPTraits>
inline
bool
ValidityCheckerMethod<MPTraits>::
IsValid(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  if(m_validity)
    return IsValidImpl(_cfg, _cdInfo, _callName);
  else
    return !IsValidImpl(_cfg, _cdInfo, _callName);
}


template <typename MPTraits>
inline
bool
ValidityCheckerMethod<MPTraits>::
IsValid(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName,
    const Formation& _robotIndexes) {
  if(m_validity)
    return IsValidImpl(_cfg, _cdInfo, _callName, _robotIndexes);
  else
    return !IsValidImpl(_cfg, _cdInfo, _callName, _robotIndexes);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
ValidityCheckerMethod<MPTraits>::
IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName,
    const Formation& _robotIndexes) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/

#endif
