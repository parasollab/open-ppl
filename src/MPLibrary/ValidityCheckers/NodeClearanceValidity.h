#ifndef PMPL_NODE_CLEARANCE_VALIDITY_H
#define PMPL_NODE_CLEARANCE_VALIDITY_H

#include "ValidityCheckerMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Considers a configuration as valid if it lies outside a threshold distance
/// from all other nodes in the roadmap.
///
/// @warning This won't work correctly with batch sampling because it only
///          checks clearance against the other nodes in the roadmap. For
///          consistency it must check only one configuration at a time.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NodeClearanceValidity : virtual public ValidityCheckerMethod<MPTraits> {

 public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType     CfgType;
    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID      VID;

    ///@}
    ///@name Construction
    ///@{

    NodeClearanceValidity();

    NodeClearanceValidity(XMLNode& _node);

    virtual ~NodeClearanceValidity() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name ValidityCheckerMethod Overrides
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    double m_minClearance;  ///< Minimum required clearance from other nodes.
    std::string m_nfLabel;  ///< NF to find nearest nodes and distance.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
NodeClearanceValidity<MPTraits>::
NodeClearanceValidity() : ValidityCheckerMethod<MPTraits>() {
  this->SetName("NodeClearanceValidity");
}

template <typename MPTraits>
NodeClearanceValidity<MPTraits>::
NodeClearanceValidity(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("NodeClearanceValidity");

  m_minClearance = _node.Read("delta", true, 1., 0.,
    std::numeric_limits<double>::max(),
    "Minimum clearance from every other node in the roadmap.");

  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder to find "
      "nearest nodes and their distance.");

}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
NodeClearanceValidity<MPTraits>::
Print(std::ostream& _os) const {
  ValidityCheckerMethod<MPTraits>::Print(_os);
  _os << "\tminClearance: " << m_minClearance
      << "\n\tnfLabel: " << m_nfLabel
      << std::endl;
}

/*-------------------- ValidityCheckerMethod Overrides -----------------------*/

template <typename MPTraits>
bool
NodeClearanceValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) { 
  // Find the nearest neighbors using the NF.
  auto r = this->GetRoadmap(_cfg.GetRobot());
  std::vector<Neighbor> neighbors;
  this->GetNeighborhoodFinder(m_nfLabel)->FindNeighbors(r, _cfg,
      std::back_inserter(neighbors));

  // The cfg is valid if we found no neighbors or if the nearest is outside the
  // clearance zone.
  const bool valid = neighbors.empty()
                  or neighbors[0].distance > m_minClearance;

  _cfg.SetLabel("VALID", valid);
  return valid;
}

/*----------------------------------------------------------------------------*/

#endif
