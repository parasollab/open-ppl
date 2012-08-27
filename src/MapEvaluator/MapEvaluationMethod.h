#ifndef MAPEVALUATIONMETHOD_H
#define MAPEVALUATIONMETHOD_H

#include "MPUtils.h"
#include "MPProblem.h"

class MapEvaluationMethod : public MPBaseObject {
  public:

    MapEvaluationMethod() {}
    MapEvaluationMethod(XMLNodeReader& _node, MPProblem* _problem)
      : MPBaseObject(_node, _problem) {}
    virtual ~MapEvaluationMethod() {}

    virtual void PrintOptions(ostream& _os) = 0;

    //has_state is called by strategies that start from an existing roadmap
    //if has_state returns true, then the evaluator is called on the input
    //roadmap to reset the state of the object
    //note that most evaluators do not have state, so this is set to false by default
    virtual bool HasState() const {return false;}

    virtual bool operator ()() = 0;
};

#endif
