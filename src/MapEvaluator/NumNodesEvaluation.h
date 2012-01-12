#ifndef NUMNODESEVALUATION_H
#define NUMNODESEVALUATION_H

#include "MapEvaluationMethod.h"
#include "MPProblem.h"

class NumNodesEvaluation : public MapEvaluationMethod {
  public:

    NumNodesEvaluation(size_t _s);
    NumNodesEvaluation(XMLNodeReader& _node, MPProblem* _problem); 
    virtual ~NumNodesEvaluation(); 

    virtual void PrintOptions(ostream& _os);

    virtual bool operator() () {
      return operator()(GetMPProblem()->CreateMPRegion());
    }

    virtual bool operator() (int _regionID); 

  protected:
    size_t m_size;
};

#endif
