#ifndef MAPEVALUATIONMETHOD_H
#define MAPEVALUATIONMETHOD_H

#include "Utilities/MPUtils.h"

template<class MPTraits>
class MapEvaluatorMethod : public MPBaseObject<MPTraits> {
  public:
    MapEvaluatorMethod() {}
    MapEvaluatorMethod(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node)
      : MPBaseObject<MPTraits>(_problem, _node) {}
    virtual ~MapEvaluatorMethod() {}

    virtual void PrintOptions(ostream& _os) const {
      _os << this->GetNameAndLabel() << endl;
    }

    //HasState is called by strategies that start from an existing roadmap
    //if HasState returns true, then the evaluator is called on the input
    //roadmap to reset the state of the object
    //note that most evaluators do not have state, so this is set to false by default
    virtual bool HasState() const {return false;}

    virtual bool operator()() = 0;
};

#endif
