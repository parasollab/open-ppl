#ifndef TRUEEVALUATION_H
#define TRUEEVALUATION_H

#include "MapEvaluationMethod.h"

class TrueEvaluation : public MapEvaluationMethod {
  public:

    TrueEvaluation();
    TrueEvaluation(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~TrueEvaluation();

    virtual void PrintOptions(ostream& _os);

    virtual bool operator()();
    virtual bool operator()(int _regionID);
};

#endif
