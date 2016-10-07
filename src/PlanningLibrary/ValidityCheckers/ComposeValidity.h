#ifndef COMPOSE_VALIDITY_H
#define COMPOSE_VALIDITY_H

#include "ValidityCheckerMethod.h"
#include "ValidityCheckerFunctor.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief Apply boolean expressions on results of multiple ValidityChecker s.
/// @tparam MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ComposeValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    enum LogicalOperator {AND, OR};

    ComposeValidity(LogicalOperator _logicalOperator = AND,
        const vector<string>& _vcLabel = vector<string>());
    ComposeValidity(MPProblemType* _problem, XMLNode& _node);
    virtual ~ComposeValidity() {}

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const string& _callName);

  private:
    LogicalOperator m_logicalOperator;
    vector<string> m_label;
};

template<class MPTraits>
ComposeValidity<MPTraits>::
ComposeValidity(LogicalOperator _operator, const vector<string>& _vcLabel) :
  ValidityCheckerMethod<MPTraits>(), m_logicalOperator(_operator), m_label(_vcLabel) {
    this->m_name = "ComposeValidity";
  }

template<class MPTraits>
ComposeValidity<MPTraits>::
ComposeValidity(MPProblemType* _problem, XMLNode& _node) :
  ValidityCheckerMethod<MPTraits>(_problem, _node) {
    this->m_name = "ComposeValidity";

    string logicalOperator = _node.Read("operator", true, "", "operator");
    if(logicalOperator == "AND" || logicalOperator == "and")
      m_logicalOperator = AND;
    else if(logicalOperator == "OR" || logicalOperator == "or")
      m_logicalOperator = OR;
    else
      throw ParseException(_node.Where(),
          "Unknown operator '" + logicalOperator + "'.");

    for(auto& child : _node)
      if(child.Name() == "ValidityChecker")
        m_label.push_back(
            child.Read("label", true, "", "validity checker method"));
  }

template<class MPTraits>
bool
ComposeValidity<MPTraits>::IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) {
  vector<ValidityCheckerPointer> vcMethod;
  typedef typename vector<ValidityCheckerPointer>::iterator VCIterator;
  for(vector<string>::iterator it = m_label.begin(); it != m_label.end(); ++it) {
    vcMethod.push_back(this->GetMPProblem()->GetValidityChecker(*it));
  }

  ValidityCheckerFunctor<MPTraits> comFunc(_cfg, _cdInfo, _callName);

  if (m_logicalOperator == AND) {
    Compose<VCIterator, logical_and<bool>, ValidityCheckerFunctor<MPTraits> > andRelation;
    return andRelation(vcMethod.begin(), vcMethod.end(), logical_and<bool>(), comFunc);
  }
  else if (m_logicalOperator == OR) {
    Compose<VCIterator, logical_or<bool>, ValidityCheckerFunctor<MPTraits> > orRelation;
    return orRelation(vcMethod.begin(), vcMethod.end(), logical_or<bool>(), comFunc);
  }
  else {
    cerr << "Error::Compose Validity read unknown label." << endl;
    return false;
  }
}

#endif
