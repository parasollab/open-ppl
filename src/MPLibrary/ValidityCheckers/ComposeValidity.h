#ifndef PMPL_COMPOSE_VALIDITY_H_
#define PMPL_COMPOSE_VALIDITY_H_

#include "MPLibrary/ValidityCheckers/ValidityCheckerMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Composed validity checker which applies two or more validity conditions.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
class ComposeValidity : public ValidityCheckerMethod {

  public:

    ///@name Local Types
    ///@{

    enum LogicalOperator {AND, OR}; ///< The supported logical operators.

    ///@}
    ///@name Construction
    ///@{

    ComposeValidity();

    ComposeValidity(XMLNode& _node);

    virtual ~ComposeValidity() = default;

    ///@}

  protected:

    ///@name ValidityChecker Overrides
    ///@{

    virtual bool IsValidImpl(Cfg& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    ///@}
    ///@name Internal State
    ///@{

    LogicalOperator m_operator; ///< The logical operator joining VC's.
    std::vector<std::string> m_vcLabels; ///< The VC labels to combine.

    ///@}

};

#endif