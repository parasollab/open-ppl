#ifndef CONDITIONALEVALUATION_H
#define CONDITIONALEVALUATION_H

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ConditionalEvaluator : public MapEvaluatorMethod<MPTraits> {
  public:

    enum Operator { LT , LEQ, GT, GEQ, MOD };

    ConditionalEvaluator(Operator _operator = LT, string _metric = "",
        double _value = 1.0);
    ConditionalEvaluator(typename MPTraits::MPProblemType* _problem,
        XMLNode& _node);
    virtual ~ConditionalEvaluator() = default;

    virtual void Print(ostream& _os) const;

    virtual bool operator()();

  protected:

    Operator m_operator;
    string m_metric;
    double m_value;
};


template<class MPTraits>
ConditionalEvaluator<MPTraits>::
ConditionalEvaluator(Operator _operator, string _metric, double _value) :
    MapEvaluatorMethod<MPTraits>(), m_operator(_operator), m_metric(_metric),
    m_value(_value) {
  this->SetName("ConditionalEvaluator");
}


template<class MPTraits>
ConditionalEvaluator<MPTraits>::
ConditionalEvaluator(typename MPTraits::MPProblemType* _problem, XMLNode& _node) :
    MapEvaluatorMethod<MPTraits>(_problem, _node) {
  this->SetName("ConditionalEvaluator");

  m_metric = _node.Read("metric_method", true, "", "Metric Method");
  m_value = _node.Read("value", true, 1.0, 0.0,
      std::numeric_limits<double>::max(), "the value of the metric");

  string op = _node.Read("operator", true, "", "operator");
  if (op == "<")
    m_operator = LT;
  else if (op == "<=")
    m_operator = LEQ;
  else if (op == ">")
    m_operator = GT;
  else if (op == ">=")
    m_operator = GEQ;
  else if (op == "%")
    m_operator = MOD;
  else
    throw ParseException(WHERE, "Unknown relational operator label '" + op + ".");
}


template<class MPTraits>
void
ConditionalEvaluator<MPTraits>::
Print(ostream& _os) const {
  MapEvaluatorMethod<MPTraits>::Print(_os);
  _os << "\tmetric method: " << m_metric << endl;
  _os << "\tvalue: " << m_value << endl;
  _os << "\toperator: ";
  switch(m_operator){
    case LT: cout << "<"; break;
    case LEQ: cout << "<="; break;
    case GT: cout << ">"; break;
    case GEQ: cout << ">="; break;
    case MOD: cout << "%"; break;
  }
  _os << endl;
}


template<class MPTraits>
bool
ConditionalEvaluator<MPTraits>::
operator()() {
  double metricValue = this->GetMPProblem()->GetMetric(m_metric)->operator()();

  switch(m_operator){
    case LT: return metricValue < m_value;
    case LEQ: return metricValue <= m_value;
    case GT: return metricValue > m_value;
    case GEQ: return metricValue >= m_value;
    case MOD:
      static double prevVal=0.0;
      if(prevVal == 0.0 ||
          (floor(metricValue/m_value) != floor(prevVal/m_value) && m_value > 0)) {
        prevVal = metricValue;
        return true;
      }
      prevVal = metricValue;
      return false;
    default:
      cout << "Unknown label is read" << endl;
      return false;
  }
}

#endif
