#ifndef PMPL_NEIGHBORHOOD_CONNECTOR_H
#define PMPL_NEIGHBORHOOD_CONNECTOR_H

#include "ConnectorMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Connect nearby neighbors together. In this method, the 'second set' of
/// vertices referred to by ConnectorMethod is determined by a nearest neighbors
/// method.
///
/// Connect nodes in map to their neighbors. The following algorithm is used:
/// - for each node, cfg1, in roadmap
///     - find neighbors N for cfg1
///     - lp is a local planner
///     - for each node cfg2 in N and numFailures < m_fail
///         - test lp.IsConnected(cfg1, cfg2)
///         - if connected, add this edge to map, _r.
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NeighborhoodConnector: public ConnectorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename RoadmapType::VID            VID;

    ///@}
    ///@name Construction
    ///@{

    NeighborhoodConnector();

    NeighborhoodConnector(XMLNode& _node);

    virtual ~NeighborhoodConnector() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name ConnectorMethod Interface
    ///@{

    template <typename AbstractRoadmapType, typename InputIterator1,
              typename InputIterator2, typename OutputIterator>
    void Connect(AbstractRoadmapType* _r,
        InputIterator1 _itr1First, InputIterator1 _itr1Last,
        InputIterator2 _itr2First, InputIterator2 _itr2Last,
        bool _fromFullRoadmap,
        OutputIterator _collision);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::string m_nfLabel;   ///< NeighborhoodFinder for selecting connections.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
NeighborhoodConnector<MPTraits>::
NeighborhoodConnector() {
  this->SetName("NeighborhoodConnector");
}


template <typename MPTraits>
NeighborhoodConnector<MPTraits>::
NeighborhoodConnector(XMLNode& _node) : ConnectorMethod<MPTraits>(_node) {
  this->SetName("NeighborhoodConnector");

  m_nfLabel = _node.Read("nfLabel", true, "",
      "The neighborhood finder for identifying connections to attempt.");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
NeighborhoodConnector<MPTraits>::
Print(std::ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
  _os << "\tnfLabel: " << m_nfLabel
      << std::endl;
}

/*------------------------ ConnectorMethod Interface -------------------------*/

template <typename MPTraits>
template <typename AbstractRoadmapType, typename InputIterator1,
          typename InputIterator2, typename OutputIterator>
void
NeighborhoodConnector<MPTraits>::
Connect(AbstractRoadmapType* _r,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {

  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);

  if(this->m_debug)
    std::cout << this->GetName() << "::Connect"
              << std::endl;

  // Attempt to connect each element in the first range to each element in the
  // second.
  std::vector<Neighbor> closest;
  for(InputIterator1 itr1 = _itr1First; itr1 != _itr1Last; ++itr1) {
    // Get the VID and cfg.
    const VID vid = _r->GetVID(itr1);
    const auto& cfg = _r->GetVertex(vid);

    if(this->m_debug)
      std::cout << "\tAttempting connections from node " << vid
                << " at " << cfg.PrettyPrint()
                << std::endl;

    // Determine nearest neighbors.
    closest.clear();
    nf->FindNeighbors(_r, _itr2First, _itr2Last, _fromFullRoadmap, cfg,
        std::back_inserter(closest));

    // Attempt connections.
    this->ConnectNeighbors(_r, vid, closest.begin(), closest.end(), _collision);
  }
}

/*----------------------------------------------------------------------------*/

#endif
