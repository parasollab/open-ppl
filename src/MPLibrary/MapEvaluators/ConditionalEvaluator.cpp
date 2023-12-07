#include "ConditionalEvaluator.h"

#include "MPLibrary/MPLibrary.h"


ConditionalEvaluator::
ConditionalEvaluator(Operator _operator, string _metric, double _value) :
    MapEvaluatorMethod(), m_operator(_operator), m_metric(_metric),
    m_value(_value) {
  this->SetName("ConditionalEvaluator");
}


ConditionalEvaluator::
ConditionalEvaluator(XMLNode& _node) : MapEvaluatorMethod(_node) {
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


void
ConditionalEvaluator::
Print(ostream& _os) const {
  MapEvaluatorMethod::Print(_os);
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


bool
ConditionalEvaluator::
operator()() {
  double metricValue = this->GetMPLibrary()->GetMetric(m_metric)->operator()();

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
