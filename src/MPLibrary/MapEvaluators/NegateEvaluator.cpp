#include "NegateEvaluator.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------- Construction -------------------------------*/

NegateEvaluator::NegateEvaluator() {
  this->SetName("NegateEvaluator");
}

NegateEvaluator::NegateEvaluator(const std::string& _meLabel) {
  this->SetName("NegateEvaluator");

  m_meLabel = _meLabel;
}

NegateEvaluator::NegateEvaluator(XMLNode& _node) : MapEvaluatorMethod(_node) {
  this->SetName("NegateEvaluator");

  m_meLabel = _node.Read("evalLabel", true, "", "Evaluator Label");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void NegateEvaluator::Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel() << "\nEvaluation method: " << m_meLabel
      << std::endl;
}

/*-------------------------- MapEvaluator Overrides --------------------------*/

bool NegateEvaluator::operator()() {
  auto me = this->GetMPLibrary()->GetMapEvaluator(m_meLabel);
  return !(*me)();
}

/*----------------------------------------------------------------------------*/
