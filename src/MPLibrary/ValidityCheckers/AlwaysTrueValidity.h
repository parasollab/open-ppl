#ifndef PMPL_ALWAYS_TRUE_VALIDITY_H_
#define PMPL_ALWAYS_TRUE_VALIDITY_H_

#include "ValidityCheckerMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Always report valid and mark the configuration for lazy validation.
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
class AlwaysTrueValidity : virtual public ValidityCheckerMethod {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType  GroupCfgType;
    typedef typename GroupCfgType::Formation Formation;

    ///@}
    ///@name Construction
    ///@{

    AlwaysTrueValidity();
    AlwaysTrueValidity(XMLNode& _node);
    virtual ~AlwaysTrueValidity() = default;

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    virtual bool IsValidImpl(Cfg& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    bool IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo, 
        const std::string& _caller) override;
    ///@}
};

#endif
