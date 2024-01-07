#include "TopologicalMapValidity.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------ Construction --------------------------------*/

TopologicalMapValidity::TopologicalMapValidity() {
  this->SetName("TopologicalMapValidity");
}

TopologicalMapValidity::TopologicalMapValidity(XMLNode& _node)
    : ValidityCheckerMethod(_node) {
  this->SetName("TopologicalMapValidity");

  m_tmLabel = _node.Read("tmLabel", false, "", "The topological map to use.");
}

/*------------------------- ValidityChecker Interface ------------------------*/

bool TopologicalMapValidity::IsValidImpl(Cfg& _cfg,
                                         CDInfo&,
                                         const std::string& _caller) {
  this->GetStatClass()->IncCfgIsColl(_caller);
  auto tm = this->GetMPLibrary()->GetMPTools()->GetTopologicalMap(m_tmLabel);

  // Position the robots within the environment.
  _cfg.ConfigureRobot();

  using NeighborhoodKey = typename TopologicalMap::NeighborhoodKey;

  const NeighborhoodKey key = tm->LocateNeighborhood(_cfg);
  for (const WorkspaceRegion* r : key)
    if (!r)
      return false;

  return true;
}

/*----------------------------------------------------------------------------*/
