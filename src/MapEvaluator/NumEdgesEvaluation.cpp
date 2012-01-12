#include "NumEdgesEvaluation.h"
#include "MPRegion.h"

NumEdgesEvaluation::NumEdgesEvaluation (size_t _s) : m_size(_s) {
  this->SetName("NumEdgesEvaluator");
} 

NumEdgesEvaluation::NumEdgesEvaluation(XMLNodeReader& _node, MPProblem* _problem) 
  : MapEvaluationMethod(_node, _problem) {
    this->SetName("NumEdgesEvaluator");
    m_size = _node.numberXMLParameter("size", true, 1, 0, MAX_INT, "number of edges required in the roadmap");
    if(m_debug) PrintOptions(cout);
  }

NumEdgesEvaluation::~NumEdgesEvaluation() {
}

void NumEdgesEvaluation::PrintOptions(ostream& _os) {
  _os << this->GetName() << "::  size = " << m_size << endl;
}

bool NumEdgesEvaluation::operator() (int _regionID){
  return (GetMPProblem()->GetMPRegion(_regionID)->GetRoadmap()->m_pRoadmap->get_num_edges() >= m_size);
}

