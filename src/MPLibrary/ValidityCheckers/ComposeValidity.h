#ifndef COMPOSE_VALIDITY_H
#define COMPOSE_VALIDITY_H

#include "ValidityCheckerMethod.h"
#include "ValidityCheckerFunctor.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief Apply boolean expressions on results of multiple ValidityChecker s.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ComposeValidity : public ValidityCheckerMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    enum LogicalOperator {AND, OR};

    ///@}
    ///@name Construction
    ///@{

    ComposeValidity(LogicalOperator _logicalOperator = AND,
        const vector<string>& _vcLabel = vector<string>());
    ComposeValidity(XMLNode& _node);
    virtual ~ComposeValidity() = default;

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const string& _callName);

    ///@}

  private:

    ///@name Internal State
    ///@{

    LogicalOperator m_logicalOperator;
    vector<string> m_labels;

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ComposeValidity<MPTraits>::
ComposeValidity(LogicalOperator _operator, const vector<string>& _vcLabel) :
    ValidityCheckerMethod<MPTraits>(), m_logicalOperator(_operator),
    m_labels(_vcLabel) {
  this->SetName("ComposeValidity");
}


template <typename MPTraits>
ComposeValidity<MPTraits>::
ComposeValidity(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("ComposeValidity");

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
      m_labels.push_back(
          child.Read("label", true, "", "validity checker method"));
}

/*------------------------- ValidityChecker Interface ------------------------*/

template <typename MPTraits>
bool
ComposeValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) {
  typedef typename MPTraits::MPLibraryType::ValidityCheckerPointer
      ValidityCheckerPointer;
  typedef typename vector<ValidityCheckerPointer>::iterator VCIterator;

  vector<ValidityCheckerPointer> vcMethod;
  for(const auto& label : m_labels)
    vcMethod.push_back(this->GetValidityChecker(label));

  ValidityCheckerFunctor<MPTraits> comFunc(_cfg, _cdInfo, _callName);

  if(m_logicalOperator == AND) {
    Compose<VCIterator, logical_and<bool>, ValidityCheckerFunctor<MPTraits>>
        andRelation;
    return andRelation(vcMethod.begin(), vcMethod.end(), logical_and<bool>(),
        comFunc);
  }
  else if(m_logicalOperator == OR) {
    Compose<VCIterator, logical_or<bool>, ValidityCheckerFunctor<MPTraits>>
        orRelation;
    return orRelation(vcMethod.begin(), vcMethod.end(), logical_or<bool>(),
        comFunc);
  }
  else {
    cerr << "Error::Compose Validity read unknown label." << endl;
    return false;
  }
}

/*----------------------------------------------------------------------------*/

#endif
