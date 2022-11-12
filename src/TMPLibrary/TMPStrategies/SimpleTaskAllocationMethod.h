#ifndef _PPL_TASK_ALLOCATION_METHOD_H_
#define _PPL_TASK_ALLOCATION_METHOD_H_

#include "TMPStrategyMethod.h"

class SimpleTaskAllocationMethod : public TMPStrategyMethod {
  public:

    ///@name Construction
    ///@{

    SimpleTaskAllocationMethod();

    SimpleTaskAllocationMethod(XMLNode& _node);

    ~SimpleTaskAllocationMethod();

    ///@}
    ///@name Accessors
    ///@{

    ///@}
    ///@name Call Method
    ///@{

    /// Assign subtasks to agents.
    virtual void AssignTasks() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    ///@}
};
#endif
