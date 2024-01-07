#ifndef PMPL_TOPOLOGICAL_MAP_VALIDITY_H
#define PMPL_TOPOLOGICAL_MAP_VALIDITY_H

#include "ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Considers configurations valid iff they are 'contained' by a region in a
/// topological map.
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
class TopologicalMapValidity : virtual public ValidityCheckerMethod {
 public:
  ///@name Local Types
  ///@{

  ///@}
  ///@name Construction
  ///@{

  TopologicalMapValidity();
  TopologicalMapValidity(XMLNode& _node);
  virtual ~TopologicalMapValidity() = default;

  ///@}
  ///@name ValidityChecker Interface
  ///@{

  virtual bool IsValidImpl(Cfg& _cfg,
                           CDInfo& _cdInfo,
                           const std::string& _caller) override;

  ///@}

 protected:
  ///@name Internal State
  ///@{

  std::string m_tmLabel;  ///< The topological map label.

  ///@}
};

#endif
