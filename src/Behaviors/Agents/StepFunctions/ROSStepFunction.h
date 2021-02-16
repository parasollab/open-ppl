#ifndef _PPL_ROS_STEP_FUNCTION_H_
#define _PPL_ROS_STEP_FUNCTION_H_

#include "StepFunction.h"

#include <ros/ros.h>

class ROSStepFunction : public StepFunction {

  public:
    ///@name Construction
    ///@{

    ROSStepFunction(Agent* _agent, XMLNode& _node);

    ~ROSStepFunction();

    ///@}
    ///@name Interface
    ///@{

    virtual void StepAgent(double _dt) override;

    ///@}

  protected:
    ///@name Internal State
    ///@{



    ///@}
};

#endif
