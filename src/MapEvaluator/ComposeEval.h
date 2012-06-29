#ifndef COMPOSEEVALUATION_H_
#define COMPOSEEVALUATION_H_

#include "MapEvaluationMethod.h"
template <class CFG, class WEIGHT> class MapEvaluator;
template <class CFG, class WEIGHT> struct ComposeEvalFunctor;

template <class CFG, class WEIGHT>
class ComposeEvaluation : public MapEvaluationMethod {
public:

  enum LogicalOperator { AND, OR };
  typedef typename std::vector<typename MapEvaluator<CFG, WEIGHT>::MapEvaluationMethodPtr>::iterator InputIterator;

  ComposeEvaluation();
  ComposeEvaluation(LogicalOperator _logicalOperator,
  		    std::vector<std::string> _evalLabels,
  		    std::vector<typename MapEvaluator<CFG, WEIGHT>::MapEvaluationMethodPtr> _evalMethods);

  ComposeEvaluation(XMLNodeReader& _node, MPProblem* _problem);
  ~ComposeEvaluation() { }

  virtual void PrintOptions(ostream& _os);

  virtual bool operator()() {
    return operator()(GetMPProblem()->CreateMPRegion());
  }
  virtual bool operator()(int _regionID);

private:
  LogicalOperator m_logicalOperator;
  std::vector<std::string> m_evalLabels;
  std::vector<typename MapEvaluator<CFG, WEIGHT>::MapEvaluationMethodPtr> m_evalMethods;
};

template <class CFG, class WEIGHT>
ComposeEvaluation<CFG, WEIGHT>::ComposeEvaluation() {
  this->SetName("ComposeEvaluation");
}

template<class CFG, class WEIGHT>
ComposeEvaluation<CFG, WEIGHT>::ComposeEvaluation(LogicalOperator _logicalOperator,
  		 				  std::vector<std::string> _evalLabels,
				                  std::vector<typename MapEvaluator<CFG, WEIGHT>::MapEvaluationMethodPtr> _evalMethods)
  : MapEvaluationMethod(), m_logicalOperator(_logicalOperator), m_evalLabels(_evalLabels) {
  this->SetName("ComposeEvaluation");
}

template<class CFG, class WEIGHT>
ComposeEvaluation<CFG, WEIGHT>::ComposeEvaluation(XMLNodeReader& _node, MPProblem* _problem) 
  : MapEvaluationMethod(_node, _problem) {
  this->SetName("ComposeEvaluation");

  string logicalOperator = _node.stringXMLParameter("operator",true,"","operator");
  if (logicalOperator == "AND" || logicalOperator == "and") {
    m_logicalOperator = AND;
  } else if (logicalOperator == "OR" || logicalOperator == "or") {
    m_logicalOperator = OR;
  } else {
    std::cout << "unknown logical operator label is read " << std::endl;
    exit(-1);
  }

  XMLNodeReader::childiterator citr;
  for (citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
    if (citr->getName() == "eval_method") {
      std::string methodLabel = citr->stringXMLParameter("method", true, "", "method");
      m_evalLabels.push_back(methodLabel);
    }
    else {
      citr->warnUnknownNode();
    }
  }
  if(m_debug) PrintOptions(cout);
}

template<class CFG, class WEIGHT>
void ComposeEvaluation<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << this->GetName() << "::" ;
  for(vector<std::string>::iterator it = m_evalLabels.begin(); it != m_evalLabels.end(); it++)
    _os << "\n\t evaluation method = \'" << *it << "\'";
  _os << "\n\t operator = " << m_logicalOperator << endl;
}

template<class CFG, class WEIGHT>
bool ComposeEvaluation<CFG, WEIGHT>::operator()(int _regionID) {
  MapEvaluator<CFG, WEIGHT>* eval = this->GetMPProblem()->GetMPStrategy()->GetMapEvaluator();

  if (m_evalMethods.size() != m_evalLabels.size()) {
    for(std::vector<std::string>::iterator it = m_evalLabels.begin(); it != m_evalLabels.end(); ++it) {
      m_evalMethods.push_back(eval->GetMethod(*it));
    }
  }

  ComposeEvalFunctor<CFG, WEIGHT> comFunc(eval, _regionID);

  if (m_logicalOperator == AND) {
    Compose<InputIterator, logical_and<bool>, ComposeEvalFunctor<CFG, WEIGHT> > m_comAnd;
    return m_comAnd(m_evalMethods.begin(), m_evalMethods.end(), logical_and<bool>(), comFunc);
  } else if (m_logicalOperator == OR) {
    Compose<InputIterator, logical_or<bool>, ComposeEvalFunctor<CFG, WEIGHT> > m_comOr;
    return m_comOr(m_evalMethods.begin(), m_evalMethods.end(), logical_or<bool>(), comFunc);
  } else {
    std::cout << "unknown label is read " << std::endl;
    return false;
  }
}

template<class CFG, class WEIGHT>
class ComposeEvalFunctor {
public:
  ComposeEvalFunctor(MapEvaluator<CFG, WEIGHT>* _eval, int _regionID) :
    m_eval(_eval), m_regionID(_regionID) {}

  ~ComposeEvalFunctor() {}

  bool operator()(typename MapEvaluator<CFG, WEIGHT>::MapEvaluationMethodPtr _conditionalType) {
    return _conditionalType->operator()(m_regionID);
  }

private:
  MapEvaluator<CFG, WEIGHT>* m_eval;
  int m_regionID;
};


#endif
