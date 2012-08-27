#ifndef NEGATEEVALUATION_H_
#define NEGATEEVALUATION_H_

#include "MapEvaluationMethod.h"

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

template <class CFG, class WEIGHT> class MapEvaluator;


template <class CFG, class WEIGHT>
class NegateEvaluation : public MapEvaluationMethod {
public:
  
  typedef typename vector<typename MapEvaluator<CFG, WEIGHT>::MapEvaluationMethodPtr>::iterator InputIterator;
  
  NegateEvaluation();
  NegateEvaluation(string _label, vector<typename MapEvaluator<CFG, WEIGHT>::MapEvaluationMethodPtr> _eval, ComposeNegate<InputIterator, ComposeEvalFunctor<CFG, WEIGHT> > _comNeg);
  NegateEvaluation(XMLNodeReader& _node, MPProblem* _problem);   
  ~NegateEvaluation() { }
 
  virtual void PrintOptions(ostream& _os);

  virtual bool operator()();
  
private:
  string m_evalLabel;
  vector<typename MapEvaluator<CFG, WEIGHT>::MapEvaluationMethodPtr> m_evalMethods;
  ComposeNegate<InputIterator, ComposeEvalFunctor<CFG, WEIGHT> > m_comNeg;
};

template <class CFG, class WEIGHT>
NegateEvaluation<CFG, WEIGHT>::NegateEvaluation() {
  this->SetName("NegateEvaluation");
}

template <class CFG, class WEIGHT>
NegateEvaluation<CFG, WEIGHT>::NegateEvaluation(string _label, std::vector<typename MapEvaluator<CFG, WEIGHT>::MapEvaluationMethodPtr> _eval, 
						ComposeNegate<InputIterator, ComposeEvalFunctor<CFG, WEIGHT> > _comNeg) 
  : m_evalLabel(_label), m_evalMethods(_eval), m_comNeg(_comNeg) {
  this->SetName("NegateEvaluation");
}

template <class CFG, class WEIGHT>
NegateEvaluation<CFG, WEIGHT>::NegateEvaluation(XMLNodeReader& _node, MPProblem* _problem)
  : MapEvaluationMethod(_node, _problem) {
  this->SetName("NegateEvaluation");
  m_evalLabel = _node.stringXMLParameter("method",true,"","method");    

  if(m_debug) PrintOptions(cout);
}

template <class CFG, class WEIGHT>
void NegateEvaluation<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << this->GetName() << ":: evaluation method = " << m_evalLabel << endl;
}

template <class CFG, class WEIGHT>
bool NegateEvaluation<CFG, WEIGHT>::operator()() {
  MapEvaluator<CFG, WEIGHT>* eval = this->GetMPProblem()->GetMPStrategy()->GetMapEvaluator();
  
  if (m_evalMethods.size() == 0) 
    m_evalMethods.push_back(eval->GetMethod(m_evalLabel));
  
  ComposeEvalFunctor<CFG, WEIGHT> comFunc(eval);
  
  return m_comNeg(m_evalMethods.begin(), comFunc);
}


#endif
