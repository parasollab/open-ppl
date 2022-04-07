#ifndef PMPL_ALWAYS_TRUE_VALIDITY_H_
#define PMPL_ALWAYS_TRUE_VALIDITY_H_

#include "ValidityCheckerMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Always report valid and mark the configuration for lazy validation.
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class AlwaysTrueValidity : virtual public ValidityCheckerMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    AlwaysTrueValidity();
    AlwaysTrueValidity(XMLNode& _node);
    virtual ~AlwaysTrueValidity() = default;

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    virtual bool IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _caller) override;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
AlwaysTrueValidity<MPTraits>::
AlwaysTrueValidity() {
  this->SetName("AlwaysTrueValidity");
}


template <typename MPTraits>
AlwaysTrueValidity<MPTraits>::
AlwaysTrueValidity(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("AlwaysTrueValidity");
}

/*------------------------- ValidityChecker Interface ------------------------*/

template <typename MPTraits>
bool
AlwaysTrueValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  _cfg.SetLabel("Lazy", true);
  return true;
}

template <typename MPTraits>
bool
AlwaysTrueValidity<MPTraits>::
IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller) {

  for(size_t i = 0; i < _cfg.GetNumRobots(); i++) {
    auto& cfg = _cfg.GetRobotCfg(i);
    cfg.SetLabel("Lazy",true);
  } 

  return true;
}

/*----------------------------------------------------------------------------*/

#endif
