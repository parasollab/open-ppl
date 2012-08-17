// A map evaluator that runs a query. Stops running when query returns a valid path.
// If no path is found, returns to sampling again.

#ifndef QueryEvaluation_H_
#define QueryEvaluation_H_

#include "MapEvaluationMethod.h"
#include "Query.h"
#include "LazyPRMQuery.h"
#include "DistanceMetrics.h"

template <class CFG, class WEIGHT>
class QueryEvaluation : public MapEvaluationMethod {

  public:
    QueryEvaluation();
    QueryEvaluation(Query<CFG, WEIGHT> _query, StatClass _stats, bool _isLazy);
    QueryEvaluation(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~QueryEvaluation() {
      if(m_query != NULL)
        delete m_query;
    }

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os); 
    
    virtual bool operator()() {
      return operator()(GetMPProblem()->CreateMPRegion());
    }
    virtual bool operator()(int _regionID); 

  protected:
    Query<CFG, WEIGHT>* m_query;
    StatClass m_stats;
    bool m_deleteNodes; // Delete any added nodes?
    bool m_lazy;        // Is the query lazy?
};

template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::QueryEvaluation() {
  m_query = NULL;
  this->SetName("QueryEvaluation");
}

template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::QueryEvaluation(Query<CFG, WEIGHT> _query, StatClass _stats, bool _isLazy = false)
  : MapEvaluationMethod(), m_query(_query), m_stats(_stats), m_lazy(_isLazy) {
  m_query = NULL;
  this->SetName("QueryEvaluation");
}

template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::QueryEvaluation(XMLNodeReader& _node, MPProblem* _problem)
  : MapEvaluationMethod(_node, _problem) {
  this->SetName("QueryEvaluation");
  ParseXML(_node);
  if(m_lazy)
    m_query = new LazyPRMQuery<CFG, WEIGHT>(_node, _problem);
  else
    m_query = new Query<CFG, WEIGHT>(_node, _problem);
  if(this->m_debug)
    PrintOptions(cout);
}

template <class CFG, class WEIGHT>
void
QueryEvaluation<CFG, WEIGHT>::ParseXML(XMLNodeReader& _node) {
  m_lazy = _node.boolXMLParameter("lazy", false, false, "Whether or not the query is lazy");
  m_deleteNodes = _node.boolXMLParameter("deleteNodes", false, true, "Whether or not to delete start and goal from roadmap");
}

template <class CFG, class WEIGHT>
void
QueryEvaluation<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << this->GetNameAndLabel();
  _os << "\n\tdeleteNodes = " << m_deleteNodes;
  _os << "\n\tlazy = " << m_lazy << endl;
}

// Runs the query
template <class CFG, class WEIGHT>
bool
QueryEvaluation<CFG, WEIGHT>::operator()(int _regionID) {
  
  Roadmap<CFG, WEIGHT>* rdmp = GetMPProblem()->GetMPRegion(_regionID)->GetRoadmap();  

  // Delete added nodes (such as start and goal) if desired
  if(m_deleteNodes)
    for(typename vector<CFG>::iterator it = m_query->GetQuery().begin(); it != m_query->GetQuery().end(); it++)
      if(rdmp->m_pRoadmap->IsVertex(*it))
        rdmp->m_pRoadmap->delete_vertex(rdmp->m_pRoadmap->GetVID(*it));

  // Perform query  
  return m_query->PerformQuery(rdmp, m_stats);
}

#endif
