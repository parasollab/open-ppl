#ifndef NUMEDGESEVALUATION_H
#define NUMEDGESEVALUATION_H

#include "MapEvaluationMethod.h"
#include "MPProblem.h"

/* This evaluator bases on the number of edges in the roadmap.
 * NOTE:: The number of edges in a directed graph could possibly 
 * be 1/2 of what is expected. One should caution the size based 
 * upon the graph and expected TOTAL number of edges not pairs of nodes.
 */
class NumEdgesEvaluation : public MapEvaluationMethod {
  public:

    NumEdgesEvaluation(size_t _s);
    NumEdgesEvaluation(XMLNodeReader& _node, MPProblem* _problem); 
    virtual ~NumEdgesEvaluation(); 

    virtual void PrintOptions(ostream& _os);

    virtual bool operator() () {
      return operator()(GetMPProblem()->CreateMPRegion());
    }

    virtual bool operator() (int _regionID); 

  protected:
    size_t m_size;
};

#endif
