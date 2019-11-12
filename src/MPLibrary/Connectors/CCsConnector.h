#ifndef PMPL_CCS_CONNECTOR_H_
#define PMPL_CCS_CONNECTOR_H_

#include "ConnectorMethod.h"

#include <containers/sequential/graph/algorithms/connected_components.h>


////////////////////////////////////////////////////////////////////////////////
/// Attempt connection between nearby CCs
///
/// Attempts connection between different connected components of the roadmap.
/// We try to connect k-closest pairs of connected components.
/// When connecting the CCs, we only attempt neighboring pairs of nodes.
///
/// @todo This class is very bad; it is making very crude estimates about
///       which CC's are closer and spending a lot of computation to do this.
///       I think there is no efficient means of implementing this concept
///       without CC tracking in the roadmap graph. We've asked STAPL for it,
///       but they will probably never implement it because it is a sequential
///       issue. An alternative is to implement our own CC tracker using the
///       roadmap hooks.
///
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class CCsConnector: public ConnectorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;

    ///@}
    ///@name Local Types
    ///@{

    /// A connected component, represented by a vertex count and representative
    /// VID.
    typedef std::pair<size_t, VID> ConnectedComponent;

    ///@}
    ///@name Construction
    ///@{

    CCsConnector();

    CCsConnector(XMLNode& _node);

    virtual ~CCsConnector() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Connection Interface
    ///@{

    template <typename InputIterator1, typename InputIterator2,
              typename OutputIterator>
    void Connect(RoadmapType* _r,
        InputIterator1 _itr1First, InputIterator1 _itr1Last,
        InputIterator2 _itr2First, InputIterator2 _itr2Last,
        bool _fromFullRoadmap,
        OutputIterator _collision);


    template <typename InputIterator1, typename InputIterator2,
              typename OutputIterator>
    void Connect(GroupRoadmapType* _r,
        InputIterator1 _itr1First, InputIterator1 _itr1Last,
        InputIterator2 _itr2First, InputIterator2 _itr2Last,
        bool _fromFullRoadmap,
        OutputIterator _collision);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Attempt to create a single connection between two connected components.
    /// @param _r The roadmap.
    /// @param _cc1 The VIDs in the first connected component.
    /// @param _cc2 The VIDs in the second connected component.
    /// @param _collision Output for invalid nodes discovered during connection
    ///                   attempts.
    template <typename OutputIterator>
    void ConnectCC(RoadmapType* _r,
        std::vector<VID>& _cc1, std::vector<VID>& _cc2,
        OutputIterator _collision);

    /// Compute all pair distance between ccs, approximated using coms of ccs
    /// @param _r The roadmap.
    void ComputeAllPairsCCDist(RoadmapType* _r,
        std::vector<ConnectedComponent>& _ccs);

    /// Find the nearest K connected components to a source component.
    /// @param _k The number of components to find.
    /// @param _representative A VID from the source cc.
    /// @return A VID from each of the _k nearest components to the source.
    std::vector<VID> GetKCCs(const size_t _k, const VID _representative);

    ///@}

  private:

    ///@name Internal State
    ///@{

    size_t m_k; ///< How many closest CCs to connect to.

    /// Maps distance between CC centroids.
    std::map<VID, std::vector<std::pair<VID, double>>> m_ccDist;

    ///@}
};

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
CCsConnector<MPTraits>::
CCsConnector() {
  this->SetName("CCsConnector");
}


template <typename MPTraits>
CCsConnector<MPTraits>::
CCsConnector(XMLNode& _node) : ConnectorMethod<MPTraits>(_node) {
  this->SetName("CCsConnector");
  m_k = _node.Read("k", true, 5, 0, 1000,
      "Try to each CC to its k-nearest CCs.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
CCsConnector<MPTraits>::
Print(std::ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
  _os << "\tk: " << m_k << std::endl;
}

/*------------------------ ConnectorMethod Overrides -------------------------*/

template <typename MPTraits>
template <typename InputIterator1, typename InputIterator2,
          typename OutputIterator>
void
CCsConnector<MPTraits>::
Connect(RoadmapType* _r,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {
  if(this->m_debug) {
    std::cout << "Before connecting CCs:\n";
    this->GetStatClass()->DisplayCCStats(std::cout, *_r);
    std::cout << std::endl;
  }

  std::vector<ConnectedComponent> ccs;

  typename RoadmapType::ColorMap colorMap;
  get_cc_stats(*_r, colorMap, ccs);

  if(ccs.size() <= 1)
    return;

  size_t k = m_k ? m_k : ccs.size() - 1;

  if(this->m_debug)
    std::cout << "Connecting " << m_k << "-closest CCs" << std::endl;

  ComputeAllPairsCCDist(_r, ccs);

  for(auto itr1 = ccs.begin(); itr1 != ccs.end(); ++itr1) {

    // Grab the k-closest CCs to connect to.
    std::vector<VID> kCCID = GetKCCs(k, itr1->second);

    for(auto itr2 = kCCID.begin(); itr2 != kCCID.end(); ++itr2) {

      //even though this might be inefficient, double check to make sure the CCs
      //are different
      if(!stapl::sequential::is_same_cc(*_r, colorMap, itr1->second, *itr2)) {

        vector<VID> cc1, cc2;

        get_cc(*_r, colorMap, itr1->second, cc1);
        get_cc(*_r, colorMap, *itr2, cc2);

        ConnectCC(_r, cc1, cc2, _collision);
        colorMap.reset();
      }
    }
  }

  if(this->m_debug) {
    std::cout << "After connecting CCs:\n";
    this->GetStatClass()->DisplayCCStats(std::cout, *_r);
    std::cout << std::endl;
  }
}


template <typename MPTraits>
template <typename InputIterator1, typename InputIterator2,
          typename OutputIterator>
void
CCsConnector<MPTraits>::
Connect(GroupRoadmapType* _r,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {
  throw NotImplementedException(WHERE);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
template <typename OutputIterator>
void
CCsConnector<MPTraits>::
ConnectCC(RoadmapType* _r,
    std::vector<VID>& _cc1, std::vector<VID>& _cc2,
    OutputIterator _collision) {
  Environment* env = this->GetEnvironment();

  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  auto lp = this->GetLocalPlanner(this->m_lpLabel);

  LPOutput<MPTraits> lpOutput;

  // Find neighbor pairs between the two CCs.
  std::vector<Neighbor> neighborPairs;
  nf->FindNeighborPairs(_r, _cc1.begin(), _cc1.end(), _cc2.begin(), _cc2.end(),
      std::back_inserter(neighborPairs));

  // Begin the connection attempts
  auto robot = this->GetTask()->GetRobot();
  for(const auto& neighbor : neighborPairs) {
    const VID cc1Elem = neighbor.source,
              cc2Elem = neighbor.target;

    CfgType _col(robot);
    lpOutput.Clear();
    if(lp->IsConnected(_r->GetVertex(cc1Elem), _r->GetVertex(cc2Elem),
          _col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true)) {
      _r->AddEdge(cc1Elem, cc2Elem, lpOutput.m_edge);
      return;
    }
    if(_col != CfgType(robot))
      *_collision++ = _col;
  }
}


template <typename MPTraits>
void
CCsConnector<MPTraits>::
ComputeAllPairsCCDist(RoadmapType* _r, std::vector<ConnectedComponent>& _ccs) {
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  auto dm = this->GetDistanceMetric(nf->GetDMLabel());
  typename RoadmapType::ColorMap colorMap;

  //compute com of ccs
  std::map<VID, CfgType> coms;
  for(auto cc = _ccs.begin(); cc != _ccs.end(); ++cc) {
    std::vector<VID> ccVIDs;
    get_cc(*_r, colorMap, cc->second, ccVIDs);
    coms[cc->second] = GetCentroid(_r, ccVIDs);
  }

  //dist between ccs
  m_ccDist.clear();
  for(auto i = coms.begin(); i != coms.end(); ++i) {
    for(auto j = i; j != coms.end(); ++j) {
      //dont track the CC distance if i and j are the same
      if(i != j) {
        const double dist = dm->Distance(i->second, j->second);
        m_ccDist[i->first].push_back(make_pair(j->first, dist));
        m_ccDist[j->first].push_back(make_pair(i->first, dist));
      }
    }
  }
}


template <typename MPTraits>
std::vector<typename MPTraits::RoadmapType::VID>
CCsConnector<MPTraits>::
GetKCCs(const size_t _k, const VID _representative) {
  std::vector<std::pair<VID, double>>& dis2CCs = m_ccDist[_representative];

  std::partial_sort(dis2CCs.begin(), dis2CCs.begin() + _k, dis2CCs.end(),
      CompareSecond<VID, double>());

  std::vector<VID> output;
  output.reserve(_k);
  for(auto i = dis2CCs.begin(); i != dis2CCs.begin() + _k; ++i)
    output.push_back(i->first);

  return output;
}

/*----------------------------------------------------------------------------*/

#endif
