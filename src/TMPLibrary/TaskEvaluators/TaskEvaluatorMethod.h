#ifndef PMPL_TASK_EVALUATOR_METHOD_H_
#define PMPL_TASK_EVALUATOR_METHOD_H_

#include "TMPLibrary/TMPBaseObject.h"

#include <iostream>

class WholeTask;

class TaskEvaluatorMethod : public TMPBaseObject {
  public:

    ///@name Construction
    ///@{

    TaskEvaluatorMethod();

    TaskEvaluatorMethod(XMLNode& _node);

    virtual ~TaskEvaluatorMethod();

    ///@}
    ///@name MapEvaluator Interface
    ///@{

    /// Evaluate a stateGraph.
    /// @return True if this stateGraph meets the evaluation criteria.
    bool operator()(std::vector<WholeTask*> _wholeTasks = {}, TaskPlan* _plan = nullptr);

    ///@}
  protected:

    virtual bool Run(std::vector<WholeTask*> _wholeTasks = {}, TaskPlan* _plan = nullptr);

    std::string m_sgLabel; ///< StateGraph Label

};

/*----------------------------------------------------------------------------*/

#endif
