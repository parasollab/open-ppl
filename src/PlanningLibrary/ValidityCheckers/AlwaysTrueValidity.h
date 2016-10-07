#ifndef ALWAYSTRUEVALIDITY_H
#define ALWAYSTRUEVALIDITY_H

#include "ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief Always return a configuration is valid.
/// @tparam MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class AlwaysTrueValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    AlwaysTrueValidity(){
      this->m_name = "AlwaysTrueValidity";
    }

    AlwaysTrueValidity(typename MPTraits::MPProblemType* _problem, XMLNode& _node)
      : ValidityCheckerMethod<MPTraits>(_problem, _node){
        this->m_name = "AlwaysTrueValidity";
      }

    virtual ~AlwaysTrueValidity(){}

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName){
      _cfg.SetLabel("Lazy", true);
      return true;
    }
};

#endif
