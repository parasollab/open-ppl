#ifndef PPL_GRASP_STRATEGY_H_
#define PPL_GRASP_STRATEGY_H_

#include "InteractionStrategyMethod.h"

class GraspStrategy : public InteractionStrategyMethod {

  public: 

    ///@name Local Types
    ///@{

    typedef Condition::State State;

    ///@}
    ///@name Construction
    ///@{

    GraspStrategy();

    GraspStrategy(XMLNode& _node);

    ~GraspStrategy();

    ///@}
    ///@name Interface 
    ///@{

    virtual bool operator()(Interaction* _interaction, State& _start) override;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    ///@}
    ///@name Internal State
    ///@{

    ///@}

};

#endif
