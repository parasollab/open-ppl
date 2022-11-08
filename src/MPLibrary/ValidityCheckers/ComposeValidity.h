#ifndef PMPL_COMPOSE_VALIDITY_H_
#define PMPL_COMPOSE_VALIDITY_H_

#include "MPLibrary/ValidityCheckers/ValidityCheckerMethod.h"

#include <algorithm>


////////////////////////////////////////////////////////////////////////////////
/// Composed validity checker which applies two or more validity conditions.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ComposeValidity : public ValidityCheckerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    enum LogicalOperator {AND, OR}; ///< The supported logical operators.

    ///@}
    ///@name Construction
    ///@{

    ComposeValidity();

    ComposeValidity(XMLNode& _node);

    virtual ~ComposeValidity() = default;

    ///@}

  protected:

    ///@name ValidityChecker Overrides
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    virtual bool IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _caller) override;

    virtual bool IsValidImpl(GroupCfgType& _cfg, Robot* _robot, std::vector<Robot*> _robots,
        CDInfo& _cdInfo, const std::string& _caller) override;

    ///@}
    ///@name Internal State
    ///@{

    LogicalOperator m_operator; ///< The logical operator joining VC's.
    std::vector<std::string> m_vcLabels; ///< The VC labels to combine.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ComposeValidity<MPTraits>::
ComposeValidity() {
  this->SetName("ComposeValidity");
}


template <typename MPTraits>
ComposeValidity<MPTraits>::
ComposeValidity(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("ComposeValidity");

  std::string logicalOperator = _node.Read("operator", true, "", "operator");
  std::transform(logicalOperator.begin(), logicalOperator.end(),
                 logicalOperator.begin(), ::tolower);

  if(logicalOperator == "and")
    m_operator = AND;
  else if(logicalOperator == "or")
    m_operator = OR;
  else
    throw ParseException(_node.Where()) << "Operator '" << logicalOperator
                                        << "' is unknown.";

  for(auto& child : _node)
    if(child.Name() == "ValidityChecker")
      m_vcLabels.push_back(child.Read("label", true, "",
          "ValidityChecker method to include in this set."));

  if(m_vcLabels.size() < 2)
    throw ParseException(_node.Where()) << "Must specify at least two methods.";
}

/*---------------------- ValidityCheckerMethod Overrides ---------------------*/

template <typename MPTraits>
bool
ComposeValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  if(this->m_debug)
    std::cout << "ComposeValidity:: checking validity..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_vcLabels) {
        auto vc = this->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _callName);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(!passed)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_vcLabels) {
        auto vc = this->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _callName);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(passed)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}


template <typename MPTraits>
bool
ComposeValidity<MPTraits>::
IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  for(size_t i = 0; i < _cfg.GetNumRobots(); i++) {
    auto& cfg = _cfg.GetRobotCfg(i);
    if(!this->IsValidImpl(cfg,_cdInfo,_caller))
      return false;
  } 

  return true;
}

template <typename MPTraits>
bool
ComposeValidity<MPTraits>::
IsValidImpl(GroupCfgType& _cfg, Robot* _robot, std::vector<Robot*> _robots,
    CDInfo& _cdInfo, const std::string& _caller) {

    throw NotImplementedException(WHERE) << "Not Implemented";
}

/*----------------------------------------------------------------------------*/

#endif
