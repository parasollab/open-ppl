#ifndef CONDITIONAL_EVALUATION_H
#define CONDITIONAL_EVALUATION_H

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Evaluates whether a given metric meets a specific numeric condition.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class ConditionalEvaluator : public MapEvaluatorMethod {
  public:

    enum Operator { LT , LEQ, GT, GEQ, MOD }; ///< The supported operators

    ConditionalEvaluator(Operator _operator = LT, string _metric = "",
        double _value = 1.0);
    ConditionalEvaluator(XMLNode& _node);
    virtual ~ConditionalEvaluator() = default;

    virtual void Print(ostream& _os) const;

    virtual bool operator()();

  protected:

    Operator m_operator; ///< The operator to use
    string m_metric; ///< The metric to evaluate
    double m_value; ///< The numeric condition
};

#endif
