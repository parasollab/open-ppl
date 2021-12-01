#ifndef PPL_NEXT_BEST_SEARCH_H_
#define PPL_NEXT_BEST_SEARCH_H_

#include "TMPStrategyMethod.h"

class Decomposition;

class NextBestSearch : public TMPStrategyMethod {

  public:

    ///@name Local Types
    ///@{

    ///@}
    ///@name Construction
    ///@{

    NextBestSearch();

    NextBestSearch(XMLNode& _node);

    virtual ~NextBestSearch();

    ///@}
    ///@name Interface
    ///@{

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    virtual void PlanTasks() override;

    double FindTaskPlan(Decomposition* _decomp);

    double ComputeMotions(double& _bestCost);

    ///@}
    ///@name Internal State
    ///@{

    ///@}

};

#endif
