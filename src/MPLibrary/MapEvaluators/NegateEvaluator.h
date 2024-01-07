#ifndef PMPL_NEGATE_EVALUATOR_H_
#define PMPL_NEGATE_EVALUATOR_H_

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Negates the result of another map evaluator.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class NegateEvaluator : virtual public MapEvaluatorMethod {
 public:
  ///@name Construction
  ///@{

  NegateEvaluator();

  NegateEvaluator(const std::string& _meLabel);

  NegateEvaluator(XMLNode& _node);

  virtual ~NegateEvaluator() = default;

  ///@}
  ///@name MPBaseObject Overrides
  ///@{

  virtual void Print(std::ostream& _os) const override;

  ///@}
  ///@name MapEvaluatorMethod Overrides
  ///@{

  virtual bool operator()() override;

  ///@}

 protected:
  ///@name Internal State
  ///@{

  std::string m_meLabel;  ///< The evaluator to negate.

  ///@}
};

#endif
