#ifndef _PPL_SIMPLE_MOTION_EVALUATOR_H_
#define _PPL_SIMPLE_MOTION_EVALUATOR_H_

#include <unordered_map>

#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"

class SimpleMotionEvaluator : public TaskEvaluatorMethod {
  public:

    ///@name Constructor
    ///@{

    SimpleMotionEvaluator();

    SimpleMotionEvaluator(XMLNode& _node);

    ~SimpleMotionEvaluator();

    ///@}
    ///@name Call method
    ///@{

    ///@}
  private:

    virtual bool Run(std::vector<WholeTask*> _wholeTasks = {}, std::shared_ptr<TaskPlan> _plan = nullptr) override;

};

#endif
