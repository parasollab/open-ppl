#ifndef PMPL_COMPOSE_EVALUATION_H_
#define PMPL_COMPOSE_EVALUATION_H_

#include "MapEvaluatorMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Combines two or more MapEvaluators to produce a compound condition.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class ComposeEvaluator : public MapEvaluatorMethod {

  public:

    ///@name Local Types
    ///@{

    enum LogicalOperator {AND, OR}; ///< The supported logical operators.

    ///@}
    ///@name Construction
    ///@{

    ComposeEvaluator();

    ComposeEvaluator(XMLNode& _node);

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

    virtual bool operator()() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    LogicalOperator m_logicalOperator;
    std::vector<std::string> m_evalLabels;

    ///@}
};

#endif
