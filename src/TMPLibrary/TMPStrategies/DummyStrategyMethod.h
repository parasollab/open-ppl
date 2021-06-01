#ifndef _PMPL_DUMMY_STRATEGY_H_
#define _PMPL_DUMMY_STRATEGY_H_

#include "TMPStrategyMethod.h"

class DummyStrategyMethod : public TMPStrategyMethod {
  public:
    ///@name Construction
    ///@{

    DummyStrategyMethod();

    DummyStrategyMethod(XMLNode& _node);

    ~DummyStrategyMethod() = default;

    ///@}
    ///@name Overrides
    ///@{

    /*virtual void PlanTasks() override;

    virtual void AssignTasks() override;

    virtual void DecomposeTasks() override;*/

  ///@}
};

#endif
