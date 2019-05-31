#ifndef PMPL_TASK_EVALUATOR_METHOD_H_
#define PMPL_TASK_EVALUATOR_METHOD_H_

#include "TMPLibrary/TMPBaseObject.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

#include <iostream>

class TaskEvaluatorMethod : public TMPBaseObject {
  public:

  	///@name Construction
    ///@{

  	TaskEvaluatorMethod() = default;

	TaskEvaluatorMethod(XMLNode& _node);

	virtual ~TaskEvaluatorMethod() = default;  	

    ///@}
    ///@name MapEvaluator Interface
    ///@{

    /// Evaluate a stateGraph.
    /// @return True if this stateGraph meets the evaluation criteria.
    virtual bool operator()() = 0;

    ///@}
}

/*----------------------------------------------------------------------------*/

#endif