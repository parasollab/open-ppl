#ifndef PMPL_TASK_EVALUATOR_METHOD_H_
#define PMPL_TASK_EVALUATOR_METHOD_H_

#include "TMPLibrary/TMPBaseObject.h"

#include <iostream>

class TaskEvaluatorMethod : public TMPBaseObject {
  public:

    ///@name Construction
    ///@{

    TaskEvaluatorMethod();

    TaskEvaluatorMethod(XMLNode& _node);

    virtual ~TaskEvaluatorMethod();

    ///@}
    ///@name Task Evaluator Interface
    ///@{

    /// Evaluate a stateGraph.
    /// @return True if this stateGraph meets the evaluation criteria.
    bool operator()(Plan* _plan = nullptr);

    virtual void Initialize();

    ///@}
  protected:

    ///@name Helper Functions
    ///@{

    virtual bool Run(Plan* _plan = nullptr);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_sgLabel; ///< StateGraph Label

    ///@}

};

/*----------------------------------------------------------------------------*/

#endif
