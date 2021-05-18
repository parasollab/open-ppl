#ifndef _PPL_DEPENDENT_PATHS_H_
#define _PPL_DEPENDENT_PATHS_H_

#include "InteractionStrategyMethod.h"

class DependentPaths : public InteractionStrategyMethod {

  public:

    ///@name Construction
    ///@{

    DependentPaths();

    DependentPaths(XMLNode& _node);

    ~DependentPaths();

    ///@}
    ///@name Interface
    ///@{

    void PlanInteraction(StateGraph* sg, std::shared_ptr<MPProblem> problemCopy);

    ///@}

  private:
    string m_firstSamplerLabel;
    string m_restSamplerLabel;

};
#endif
