#ifndef PMPL_NEGATE_VALIDITY_H_
#define PMPL_NEGATE_VALIDITY_H_

#include "ValidityCheckerMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Wrapper for another validity checker which reverses its output.
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NegateValidity : public ValidityCheckerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    NegateValidity();

    NegateValidity(XMLNode& _node);

    virtual ~NegateValidity() = default;

    ///@}
    ///@name ValidityCheckerMethod Overrides
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_vcLabel; ///< The wrapped VC method to call (and negate).

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
NegateValidity<MPTraits>::
NegateValidity() {
  this->SetName("NegateValidity");
}


template <typename MPTraits>
NegateValidity<MPTraits>::
NegateValidity(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("NegateValidity");

  m_vcLabel = _node.Read("vcLabel", true, "", "validity checker method");
}

/*--------------------- ValidityCheckerMethod Overrides ----------------------*/

template <typename MPTraits>
bool
NegateValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  auto vc = this->GetValidityChecker(m_vcLabel);
  return !vc->IsValid(_cfg, _cdInfo, _callName);
}

/*----------------------------------------------------------------------------*/

#endif
