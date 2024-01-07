#include "NodeClearanceValidity.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------ Construction --------------------------------*/

NodeClearanceValidity::NodeClearanceValidity() : ValidityCheckerMethod() {
  this->SetName("NodeClearanceValidity");
}

NodeClearanceValidity::NodeClearanceValidity(XMLNode& _node)
    : ValidityCheckerMethod(_node) {
  this->SetName("NodeClearanceValidity");

  m_minClearance =
      _node.Read("delta", true, 1., 0., std::numeric_limits<double>::max(),
                 "Minimum clearance from every other node in the roadmap.");

  m_nfLabel = _node.Read("nfLabel", true, "",
                         "Neighborhood Finder to find "
                         "nearest nodes and their distance.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void NodeClearanceValidity::Print(std::ostream& _os) const {
  ValidityCheckerMethod::Print(_os);
  _os << "\tminClearance: " << m_minClearance << "\n\tnfLabel: " << m_nfLabel
      << std::endl;
}

/*-------------------- ValidityCheckerMethod Overrides -----------------------*/

bool NodeClearanceValidity::IsValidImpl(Cfg& _cfg,
                                        CDInfo& _cdInfo,
                                        const std::string& _callName) {
  // Find the nearest neighbors using the NF.
  auto r = this->GetRoadmap(_cfg.GetRobot());
  std::vector<Neighbor> neighbors;
  this->GetMPLibrary()->GetNeighborhoodFinder(m_nfLabel)->FindNeighbors(
      r, _cfg, std::back_inserter(neighbors));

  // The cfg is valid if we found no neighbors or if the nearest is outside the
  // clearance zone.
  const bool valid =
      neighbors.empty() or neighbors[0].distance > m_minClearance;

  _cfg.SetLabel("VALID", valid);
  return valid;
}

/*----------------------------------------------------------------------------*/
