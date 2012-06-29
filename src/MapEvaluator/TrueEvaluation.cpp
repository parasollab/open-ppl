#include "TrueEvaluation.h"

TrueEvaluation::TrueEvaluation() {
  this->SetName("TrueEvaluation");
}

TrueEvaluation::TrueEvaluation(XMLNodeReader& _node, MPProblem* _problem)
  : MapEvaluationMethod(_node, _problem) {
    this->SetName("TrueEvaluation");
    if(m_debug) PrintOptions(cout);
}

TrueEvaluation::~TrueEvaluation() {
}

void TrueEvaluation::PrintOptions(ostream& _os){
  _os << "True Evaluator always returns true, no options present." << endl;
}

bool TrueEvaluation::operator()() {
  return true;
}

bool TrueEvaluation::operator()(int _regionID) {
  return true;
}
