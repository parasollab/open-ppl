#ifndef PMPL_NEIGHBORHOOD_CONNECTOR_H
#define PMPL_NEIGHBORHOOD_CONNECTOR_H

#include "ConnectorMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Connect nearby neighbors together.
///
/// Connect nodes in map to their neighbors. The following algorithm is used:
/// -# for evry node, cfg1, in roadmap
///     -# find neighbors N for cfg1
///     -# lp is a local planner
///     -# for every node cfg2 in N and numFailures < m_fail
///         -# test lp.IsConnected(cfg1, cfg2)
///         -# if connected, add this edge to map, _rm.
///     -# end for
/// -# end for
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NeighborhoodConnector: public ConnectorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    ///@}
    ///@name Construction
    ///@{

    NeighborhoodConnector(std::string _nfLabel = "", std::string _lpLabel = "",
        bool _checkIfSameCC = false, bool _countFailures = false,
        size_t _fail = 5);

    NeighborhoodConnector(XMLNode& _node);

    virtual ~NeighborhoodConnector() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name ConnectorMethod Interface
    ///@{

    template <typename InputIterator1, typename InputIterator2,
              typename OutputIterator>
    void Connect(RoadmapType* _rm,
        InputIterator1 _itr1First, InputIterator1 _itr1Last,
        InputIterator2 _itr2First, InputIterator2 _itr2Last,
        bool _fromFullRoadmap,
        OutputIterator _collision);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    template<typename InputIterator, typename OutputIterator>
    void ConnectNeighbors(
        RoadmapType* _rm, VID _vid,
        InputIterator _first, InputIterator _last,
        OutputIterator _collision);

    ///@}

  private:

    ///@name Internal State
    ///@{

    bool m_checkIfSameCC{true};  ///< Skip connections within the same CC?
    bool m_countFailures{false}; ///< Limit the failures per iteration?
    size_t m_fail{0};            ///< Number of allowed failures per iteration.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
NeighborhoodConnector<MPTraits>::
NeighborhoodConnector(std::string _nfLabel, std::string _lpLabel,
    bool _checkIfSameCC, bool _countFailures, size_t _fail) :
    ConnectorMethod<MPTraits>(_nfLabel, _lpLabel),
    m_checkIfSameCC(_checkIfSameCC),
    m_countFailures(_countFailures),
    m_fail(_fail) {
  this->SetName("NeighborhoodConnector");
}


template <typename MPTraits>
NeighborhoodConnector<MPTraits>::
NeighborhoodConnector(XMLNode& _node) : ConnectorMethod<MPTraits>(_node) {
  this->SetName("NeighborhoodConnector");

  m_checkIfSameCC = _node.Read("checkIfSameCC", false, m_checkIfSameCC,
      "If true, do not connect if edges are in the same CC");

  m_countFailures = _node.Read("countFailures", false, m_countFailures,
      "If false, ignore failure count and just attempt k; if true, attempt k "
      "neighbors until too many failures detected.");

  m_fail = _node.Read("fail", false, m_fail, size_t(0),
      std::numeric_limits<size_t>::max(),
      "Number of failed connections allowed before operation terminates.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
NeighborhoodConnector<MPTraits>::
Print(std::ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
  _os << "\tfail = " << m_fail
      << "\n\tcountFailures = " << m_countFailures
      << std::endl;
}

/*------------------------ ConnectorMethod Interface -------------------------*/

template <typename MPTraits>
template <typename InputIterator1, typename InputIterator2,
          typename OutputIterator>
void
NeighborhoodConnector<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {

  auto nfptr = this->GetNeighborhoodFinder(this->m_nfLabel);

  if(this->m_debug)
    std::cout << this->GetName() << "::Connect"
              << std::endl;

  // Attempt to connect each element in the first range to each element in the
  // second.
  std::vector<Neighbor> closest;
  for(InputIterator1 itr1 = _itr1First; itr1 != _itr1Last; ++itr1) {
    // Get the VID and cfg.
    const VID vid = _rm->GetGraph()->GetVID(itr1);
    const CfgType& cfg = _rm->GetGraph()->GetVertex(vid);

    if(this->m_debug)
      std::cout << "\tAttempting connections from node " << vid
                << " at " << cfg.PrettyPrint()
                << std::endl;

    // Determine nearest neighbors.
    closest.clear();
    nfptr->FindNeighbors(_rm, _itr2First, _itr2Last, _fromFullRoadmap, cfg,
        std::back_inserter(closest));

    // Attempt connections.
    ConnectNeighbors(_rm, vid, closest.begin(), closest.end(), _collision);
  }
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
void
NeighborhoodConnector<MPTraits>::
ConnectNeighbors(RoadmapType* _rm, VID _vid,
    InputIterator _first, InputIterator _last, OutputIterator _collision) {
  auto robot = this->GetTask()->GetRobot();
  auto env = this->GetEnvironment();
  auto lp = this->GetLocalPlanner(this->m_lpLabel);
  auto g = _rm->GetGraph();

  LPOutput<MPTraits> lpOutput;
  size_t failCount = 0;

  // connect the found k-closest to the current iteration's CfgType
  for(InputIterator itr2 = _first; itr2 != _last; ++itr2) {
    // stopping conditions
    if(this->m_countFailures and failCount >= m_fail) {
      if(this->m_debug)
        std::cout << "\t\tFailures (" << failCount << ") exceed threshold "
                  << "of " << m_fail << ", stopping."
                  << std::endl;
      break;
    }

    const VID v2 = itr2->target;
    if(this->m_debug)
      std::cout << "\t\tAttempting connection to " << v2
                << " at distance " << itr2->distance
                << std::endl;

    // Check if this attempt is cached.
    const bool cached = this->IsCached(_vid, v2);
    this->GetStatClass()->GetAverage(this->GetName() + "::CacheHitRatio") += cached;
    if(cached) {
      const bool success = this->GetCached(_vid, v2);
      if(this->m_debug)
        std::cout << "\t\t\tConnection already "
                  << (success ? "passed" : "failed") << ", skipping."
                  << std::endl;
      failCount += !success;;
      continue;
    }

    // if the edge already exists, so no need to call LP. Count as success.
    if(g->IsEdge(_vid, v2)) {
      if(this->m_debug)
        std::cout << "\t\t\tEdge already exists in roadmap, skipping."
                  << std::endl;
      continue;
    }

    // If the nodes are in the same connected component, count as success.
    if(m_checkIfSameCC) {
      typename GraphType::ColorMap colorMap;
      if(stapl::sequential::is_same_cc(*g, colorMap, _vid, v2)) {
        if(this->m_debug)
          std::cout << "\t\t\tNodes are in the same connected component, "
                    << "skipping."
                    << std::endl;
        continue;
      }
    }

    // attempt connection with the local planner
    CfgType& c1 = g->GetVertex(_vid);
    CfgType& c2 = g->GetVertex(v2);

    CfgType collision(robot);
    lpOutput.Clear();
    const bool connectable = lp->IsConnected(c1, c2, collision, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true);

    if(collision != CfgType(robot))
      *_collision++ = collision;

    // Track connection attempt.
    this->AddConnectionAttempt(_vid, v2, connectable);

    if(connectable) {
      if(this->m_debug)
        std::cout << "\t\tConnection succeeded." << std::endl;

      _rm->GetGraph()->AddEdge(_vid, v2, lpOutput.m_edge);
    }
    else {
      if(this->m_debug)
        std::cout << "\t\tConnection failed." << std::endl;
      ++failCount;
    }
  }
}

/*----------------------------------------------------------------------------*/

#endif
