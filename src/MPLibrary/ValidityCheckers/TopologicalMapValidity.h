#ifndef PMPL_TOPOLOGICAL_MAP_VALIDITY_H
#define PMPL_TOPOLOGICAL_MAP_VALIDITY_H

#include "ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Considers configurations valid iff they are 'contained' by a region in a
/// topological map.
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TopologicalMapValidity : virtual public ValidityCheckerMethod<MPTraits> {
 public:
  ///@name Local Types
  ///@{

  typedef typename MPTraits::CfgType CfgType;

  ///@}
  ///@name Construction
  ///@{

  TopologicalMapValidity();
  TopologicalMapValidity(XMLNode& _node);
  virtual ~TopologicalMapValidity() = default;

  ///@}
  ///@name ValidityChecker Interface
  ///@{

  virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
                           const std::string& _caller) override;

  ///@}

 protected:
  ///@name Internal State
  ///@{

  std::string m_tmLabel;  ///< The topological map label.

  ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
TopologicalMapValidity<MPTraits>::
    TopologicalMapValidity() {
  this->SetName("TopologicalMapValidity");
}

template <typename MPTraits>
TopologicalMapValidity<MPTraits>::
    TopologicalMapValidity(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("TopologicalMapValidity");

  m_tmLabel = _node.Read("tmLabel", true, "", "The topological map to use.");
}

/*------------------------- ValidityChecker Interface ------------------------*/

template <typename MPTraits>
bool TopologicalMapValidity<MPTraits>::
    IsValidImpl(CfgType& _cfg, CDInfo&, const std::string& _caller) {
  this->GetStatClass()->IncCfgIsColl(_caller);

  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);

  // Position the robots within the environment.
  _cfg.ConfigureRobot();

  using NeighborhoodKey = typename TopologicalMap<MPTraits>::NeighborhoodKey;

  const NeighborhoodKey key = tm->LocateNeighborhood(_cfg);
  for (const WorkspaceRegion* r : key)
    if (!r)
      return false;

  return true;
}

/*----------------------------------------------------------------------------*/

#endif
