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
/// Cfg c;
/// CDInfo cdInfo;
/// string callee("SomeFunc");
/// bool valid = vc->IsValid(c, cdInfo, callee);
/// @endcode
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
class ValidityCheckerMethod : public MPBaseObject {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType GroupCfgType;
    typedef typename GroupCfgType::Formation Formation;

    ///@}
    ///@name Construction
    ///@{

    ValidityCheckerMethod() = default;

    ValidityCheckerMethod(XMLNode& _node) : MPBaseObject(_node) {}

    virtual ~ValidityCheckerMethod() = default;

    ///@}
    ///@name Validity Accessors
    ///@{

    /// Get the current meaning of "valid" (true is default).
    bool GetValidity() const;

    /// Switches the meaning of "valid" to "invalid" and vice versa.
    void ToggleValidity();

    ///@}
    ///@name Individual Configuration Validity
    ///@{

    /// Classify a configuration to either cfree or cobst.
    /// @param _cfg The individual configuration.
    /// @param _cdInfo Output for extra computed information such as clearance.
    /// @param _caller Name of the calling function.
    /// @return True iff _cfg is in cfree, false otherwise.
    bool IsValid(Cfg& _cfg, CDInfo& _cdInfo, const std::string& _caller);
    ///@example ValidityCheckers_UseCase.cpp
    /// This is an example of how to use the validity checker methods.

    /// This version does not return extra information.
    /// @overload
    bool IsValid(Cfg& _cfg, const std::string& _caller);

    ///@}
    ///@name Group Configuration Validity
    ///@{

    /// Classify a gropu configuration to either cfree or cobst.
    /// @param _cfg The group configuration.
    /// @param _cdInfo Output for extra computed information such as clearance.
    /// @param _caller Name of the calling function.
    /// @return True iff _cfg is in cfree, false otherwise.
    bool IsValid(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller);

    /// This version does not return extra information.
    /// @overload
    bool IsValid(GroupCfgType& _cfg, const std::string& _caller);

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
    virtual bool IsValidImpl(Cfg& _cfg, CDInfo& _cdInfo,
        const std::string& _caller) = 0;

    /// Implementation of group cfg classification.
    /// @param _cfg The group configuration.
    /// @param _cdInfo Output for extra computed information such as clearance.
    /// @param _caller Name of the calling function.
    /// @return True if _cfg is in cfree.
    virtual bool IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _caller);

    ///@}
    ///@name Internal State
    ///@{

    bool m_validity{true}; ///< Use standard validity? False indicates negation.

    ///@}

};

#endif
