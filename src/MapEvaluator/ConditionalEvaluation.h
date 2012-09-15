#ifndef CONDITIONALEVALUATION_H
#define CONDITIONALEVALUATION_H

#include "MapEvaluationMethod.h"
#include "Metrics.h"

template <class CFG, class WEIGHT>
class ConditionalEvaluation : public MapEvaluationMethod {
  public:

    enum RelationalOperator { LT , LEQ, GT, GEQ };

    ConditionalEvaluation();
    ConditionalEvaluation(RelationalOperator _relationalOperator, string _metric, double _value);
    ConditionalEvaluation(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~ConditionalEvaluation() {}

    virtual void PrintOptions(ostream& _os);

    virtual bool operator()();

  protected:
    RelationalOperator m_relationalOperator;
    string m_metric;
    double m_value;
};

template <class CFG, class WEIGHT>
ConditionalEvaluation<CFG, WEIGHT>::ConditionalEvaluation() {
  this->SetName("ConditionalEvaluation");
}

template <class CFG, class WEIGHT>
ConditionalEvaluation<CFG, WEIGHT>::ConditionalEvaluation(RelationalOperator _relationalOperator, string _metric, double _value)
  : MapEvaluationMethod(), m_relationalOperator(_relationalOperator), m_metric(_metric), m_value(_value) {
  this->SetName("ConditionalEvaluation");
}

template <class CFG, class WEIGHT>
ConditionalEvaluation<CFG, WEIGHT>::ConditionalEvaluation(XMLNodeReader& _node, MPProblem* _problem)
  : MapEvaluationMethod(_node, _problem) {
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
    cout << "unknown relational operator label read" << endl;
    exit(-1);
  }

  if(m_debug) PrintOptions(cout);
}

template <class CFG, class WEIGHT>
void ConditionalEvaluation<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << this->GetName() << "::";
  _os << "\n\tmetric method = \'" << m_metric << "\'";
  _os << "\n\tvalue = \'" << m_value << "\'";
  _os << "\n\toperator = " << m_relationalOperator << endl; 
}

template <class CFG, class WEIGHT>
bool ConditionalEvaluation<CFG, WEIGHT>::operator()() {
  double metric_value = this->GetMPProblem()->GetMPStrategy()->GetMetric()->GetMethod(m_metric)->operator()();

  if (m_relationalOperator == LT) {
    return metric_value < m_value;
  } else if (m_relationalOperator == LEQ) {
    return metric_value <= m_value;
  } else if (m_relationalOperator == GT) {
    return metric_value > m_value;
  } else if (m_relationalOperator == GEQ) {
    return metric_value >= m_value;
  } else {
    cout << "unknown label is read" << endl;
    return false;
  }
}

#endif
