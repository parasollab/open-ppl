#ifndef PMPL_EVALUATE_MAP_STRATEGY_H_
#define PMPL_EVALUATE_MAP_STRATEGY_H_

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "Utilities/XMLNode.h"

#include <iostream>
#include <string>


////////////////////////////////////////////////////////////////////////////////
/// Evaluate the current roadmap using a designated map evaluator. This is
/// mostly useful in simulations where we need to verify/analyze some property
/// of a roadmap.
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class EvaluateMapStrategy : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    EvaluateMapStrategy();

    EvaluateMapStrategy(XMLNode& _node);

    virtual ~EvaluateMapStrategy() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name MPStrategy Overrides
    ///@{

    virtual void Iterate() override;
    virtual void Finalize() override;

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
EvaluateMapStrategy<MPTraits>::
EvaluateMapStrategy() {
  this->SetName("EvaluateMapStrategy");
}


template <typename MPTraits>
EvaluateMapStrategy<MPTraits>::
EvaluateMapStrategy(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("EvaluateMapStrategy");

  for(auto& child : _node)
    if(child.Name() == "Evaluator") {
      std::string method = child.Read("label", true, "", "Map Evaluation Method");
      this->m_meLabels.push_back(method);
    }
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
EvaluateMapStrategy<MPTraits>::
Print(std::ostream& _os) const {
  _os << "EvaluateMapStrategy::"
      << "\n\tevaluators:";
  for(auto& label : this->m_meLabels)
    std::cout << " " << label;
}

/*----------------------- MPStrategyMethod Overrides -------------------------*/

template <typename MPTraits>
void
EvaluateMapStrategy<MPTraits>::
Iterate() {
  this->EvaluateMap();
}


template <typename MPTraits>
void
EvaluateMapStrategy<MPTraits>::
Finalize() {
  // Don't print roadmaps or stats for this strategy.
}

/*----------------------------------------------------------------------------*/

#endif
