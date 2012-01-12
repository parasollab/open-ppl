#include "NumNodesEvaluation.h"
#include "MPRegion.h"

NumNodesEvaluation::NumNodesEvaluation(size_t _s) : m_size(_s) {
  this->SetName("NumNodesEvaluator");
} 

NumNodesEvaluation::NumNodesEvaluation(XMLNodeReader& _node, MPProblem* _problem) 
  : MapEvaluationMethod(_node, _problem) {
    this->SetName("NumNodesEvaluator");
    m_size = _node.numberXMLParameter("size", true, 1, 0, MAX_INT, "number of nodes required in roadmap");
    if(m_debug) PrintOptions(cout);
  }

NumNodesEvaluation::~NumNodesEvaluation() {
}

void NumNodesEvaluation::PrintOptions(ostream& _os) {
  _os << this->GetName() << "::  size = " << m_size << endl;
}

bool NumNodesEvaluation::operator() (int _regionID) {
  return (GetMPProblem()->GetMPRegion(_regionID)->GetRoadmap()->m_pRoadmap->get_num_vertices() >= m_size);
}

