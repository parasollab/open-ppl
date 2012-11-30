#ifndef CONDITIONALEVALUATION_H
#define CONDITIONALEVALUATION_H

#include "MapEvaluationMethod.h"

template<class MPTraits>
class ConditionalEvaluation : public MapEvaluationMethod<MPTraits> {
  public:

    enum RelationalOperator { LT , LEQ, GT, GEQ };

    ConditionalEvaluation(RelationalOperator _relationalOperator = LT, string _metric = "", double _value = 1.0);
    ConditionalEvaluation(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~ConditionalEvaluation() {}

    virtual void PrintOptions(ostream& _os);

    virtual bool operator()();

  protected:
    RelationalOperator m_relationalOperator;
    string m_metric;
    double m_value;
};

template<class MPTraits>
ConditionalEvaluation<MPTraits>::ConditionalEvaluation(RelationalOperator _relationalOperator, string _metric, double _value)
  : MapEvaluationMethod<MPTraits>(), m_relationalOperator(_relationalOperator), m_metric(_metric), m_value(_value) {
  this->SetName("ConditionalEvaluation");
}

template<class MPTraits>
ConditionalEvaluation<MPTraits>::ConditionalEvaluation(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node)
  : MapEvaluationMethod<MPTraits>(_problem, _node) {
  this->SetName("ConditionalEvaluation");

  m_metric = _node.stringXMLParameter("metric_method", true, "", "Metric Method");
  m_value = _node.numberXMLParameter("value", true, 1.0, 0.0, std::numeric_limits<double>::max(), "the value of the metric");

  string relationalOperator = _node.stringXMLParameter("operator", true, "", "operator");
  if (relationalOperator == "<")
    m_relationalOperator = LT;
  else if (relationalOperator == "<=")
    m_relationalOperator = LEQ;
  else if (relationalOperator == ">")
    m_relationalOperator = GT;
  else if (relationalOperator == ">=")
    m_relationalOperator = GEQ;
  else {
    cerr << "Error::Unknown relational operator label read in " << this->GetNameAndLabel() << ". Exiting." << endl;
    exit(1);
  }
}

template<class MPTraits>
void ConditionalEvaluation<MPTraits>::PrintOptions(ostream& _os) {
  MapEvaluationMethod<MPTraits>::PrintOptions(_os);
  _os << "\tmetric method: " << m_metric << endl;
  _os << "\tvalue: " << m_value << endl;
  _os << "\toperator: ";
  switch(m_relationalOperator){
    case LT: cout << "<"; break;
    case LEQ: cout << "<="; break;
    case GT: cout << ">"; break;
    case GEQ: cout << ">="; break;
  }
  _os << endl; 
}

template<class MPTraits>
bool ConditionalEvaluation<MPTraits>::operator()() {
  double metric_value = this->GetMPProblem()->GetMetric(m_metric)->operator()();

  switch(m_relationalOperator){
    case LT: return metric_value < m_value;
    case LEQ: return metric_value <= m_value;
    case GT: return metric_value > m_value;
    case GEQ: return metric_value >= m_value;
    default:
      cout << "unknown label is read" << endl;
      return false;
  }
}

#endif
