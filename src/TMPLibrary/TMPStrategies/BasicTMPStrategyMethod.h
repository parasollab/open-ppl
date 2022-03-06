#ifndef PPL_BASIC_TMP_STRATEGY_METHOD_H_
#define PPL_BASIC_TMP_STRATEGY_METHOD_H_

#include "TMPStrategyMethod.h"

class BasicTMPStrategyMethod : public TMPStrategyMethod {

  public:
  
    ///@name Local Types
    ///@{

    ///@}
    ///@name Construction
    ///@{

    BasicTMPStrategyMethod();

    BasicTMPStrategyMethod(XMLNode& _node);

    virtual ~BasicTMPStrategyMethod();

    ///@}

  private:

    ///@name Helper Method Overrides
    ///@{

    virtual void PlanTasks() override;

    virtual void DecomposeTasks() override;

    virtual void AssignTasks() override;

    ///@}
};

#endif
