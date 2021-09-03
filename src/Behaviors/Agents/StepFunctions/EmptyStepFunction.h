#ifndef PPL_EMPTY_STEP_FUNCTION_H_
#define PPL_EMPTY_STEP_FUNCTION_H_

#include "StepFunction.h"

class EmptyStepFunction : public StepFunction {
  public :
    ///@name Construction
    ///@{ 
    
    EmptyStepFunction(Agent* _agent, XMLNode& _node);

    virtual ~EmptyStepFunction();

    ///@}
    ///@name Interface
    ///@{

    /// Function to call the step function behavior
    virtual void StepAgent(double _dt) override;

    ///@}

};

#endif
