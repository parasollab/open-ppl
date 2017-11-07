#ifndef PRM_QUERY_H_
#define PRM_QUERY_H_

#include "QueryMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Evaluate a roadmap under construction to see if a query has been satisfied.
///
/// This query is specialized for PRM methods.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PRMQuery : public QueryMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    PRMQuery();
    PRMQuery(XMLNode& _node);
    virtual ~PRMQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name QueryMethod Overrides
    ///@{

    virtual bool PerformSubQuery(const CfgType& _start, const CfgType& _goal)
        override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Ensure a given configuration is in the roadmap, adding it if necessary.
    /// @param[in] _cfg The configuration to ensure.
    /// @return The VID of _cfg, and a bool indicating whether or not it had to
    ///         be added to the map.
    pair<VID, bool> EnsureCfgInMap(const CfgType& _cfg);

    /// Remove an ensured configuration if it was added to the map.
    /// @param[in] _temp The ensured configuration's VID and an indicator of
    ///                  whether or not it was added as a temporary.
    void RemoveTempCfg(pair<VID, bool> _temp);

    /// Get the CC stats of the roadmap.
    /// @return A vector with one element per CC. The elements contain the size
    ///         of a CC and one VID within it.
    vector<pair<size_t, VID>> FindCCs();

    /// Try to connect a given VID to the CC containing a second VID.
    /// @param[in] _toConnect The VID to connect.
    /// @param[in] _inCC One of the VIDs in the CC of interest.
    void ConnectToCC(const VID _toConnect, const VID _inCC);

    ///@}
    ///@name MP Object Labels
    ///@{

    vector<string> m_ncLabels{"kClosest"};

    ///@}
    ///@name Graph Search
    ///@{

    bool m_deleteNodes{false}; ///< Delete any added nodes?

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
PRMQuery<MPTraits>::
PRMQuery() : QueryMethod<MPTraits>() {
  this->SetName("PRMQuery");
}


template <typename MPTraits>
PRMQuery<MPTraits>::
PRMQuery(XMLNode& _node) : QueryMethod<MPTraits>(_node) {
  this->SetName("PRMQuery");

  m_deleteNodes = _node.Read("deleteNodes", false, m_deleteNodes, "Whether or "
      "not to delete start and goal from roadmap");

  bool defaultsCleared = false;
  for(auto& child : _node) {
    if(child.Name() == "NodeConnectionMethod") {
      if(!defaultsCleared) {
        defaultsCleared = true;
        m_ncLabels.clear();
      }
      m_ncLabels.push_back(child.Read("method", true, "", "Connector method"));
    }
  }
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
PRMQuery<MPTraits>::
Print(ostream& _os) const {
  QueryMethod<MPTraits>::Print(_os);
  _os << "\n\tDelete Nodes: " << m_deleteNodes
      << "\n\tConnectors:" << endl;
  for(const auto label : m_ncLabels)
    _os << "\t\t" << label << endl;
}

/*------------------------------ Query Interface -----------------------------*/

template <typename MPTraits>
bool
PRMQuery<MPTraits>::
PerformSubQuery(const CfgType& _start, const CfgType& _goal) {
  if(this->m_debug)
    cout << "Evaluating sub-query:" << endl
         << "\tfrom " << _start << endl
         << "\tto   " << _goal << endl;

  // Add start and goal to roadmap (if not already there).
  auto start = EnsureCfgInMap(_start);
  auto goal  = EnsureCfgInMap(_goal);

  // Find connected components.
  auto ccs = FindCCs();

  // Check each connected component.
  bool connected = false;
  for(auto cc : ccs) {
    // Try connecting the start and goal to this CC.
    ConnectToCC(start.first, cc.second);
    ConnectToCC(goal.first, cc.second);

    // If start and goal are connected to the same CC, generate path and end.
    if(this->SameCC(start.first, goal.first)) {
      connected = true;
      this->GeneratePath(start.first, goal.first);
      break;
    }
  }

  if(this->m_debug) {
    if(connected)
      cout << "\tSuccess: found path from start node " << start.first
           << " to goal node " << goal.first << "." << endl;
    else
      cout << "\tFailed to connect start node " << start.first
           << " to goal node " << goal.first << "." << endl;
  }

  // Remove start and goal if necessary.
  if(m_deleteNodes) {
    RemoveTempCfg(start);
    RemoveTempCfg(goal);
  }

  return connected;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
pair<typename PRMQuery<MPTraits>::VID, bool>
PRMQuery<MPTraits>::
EnsureCfgInMap(const CfgType& _cfg) {
  auto g = this->GetRoadmap()->GetGraph();

  const bool exists = g->IsVertex(_cfg);

  if(exists)
    // The vertex already exists.
    return std::make_pair(g->GetVID(_cfg), false);
  else if(!m_deleteNodes)
    // The vertex is new.
    return std::make_pair(g->AddVertex(_cfg), true);
  else {
    // The vertex is new, but we are adding a temporary node. Do not run the hook
    // functions.
    g->DisableHooks();
    auto out = std::make_pair(g->AddVertex(_cfg), true) ;
    g->EnableHooks();
    return out;
  }
}


template <typename MPTraits>
void
PRMQuery<MPTraits>::
RemoveTempCfg(pair<VID, bool> _temp) {
  // Return if this wasn't added as a temporary cfg.
  if(!_temp.second)
    return;

  auto g = this->GetRoadmap()->GetGraph();
  g->DisableHooks();
  g->DeleteVertex(_temp.first);
  g->EnableHooks();
}


template <typename MPTraits>
vector<pair<size_t, typename PRMQuery<MPTraits>::VID>>
PRMQuery<MPTraits>::
FindCCs() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "PRMQuery::FindCCs");
  stats->IncStat("CC Operations");

  vector<pair<size_t, VID>> ccs;
  auto colorMap = this->GetColorMap();
  get_cc_stats(*this->GetRoadmap()->GetGraph(), colorMap, ccs);

  if(this->m_debug)
    cout << "\tThere are " << ccs.size() << " CCs." << endl;

  return ccs;
}


template <typename MPTraits>
void
PRMQuery<MPTraits>::
ConnectToCC(const VID _toConnect, const VID _inCC) {
  // If the nodes are already in the same CC, return.
  if(this->SameCC(_toConnect, _inCC)) {
    if(this->m_debug)
      cout << "\tNodes " << _toConnect << " and " << _inCC << " are already in "
           << "the same CC." << endl;
    return;
  }

  auto stats = this->GetStatClass();
  stats->IncStat("CC Operations");
  MethodTimer mt(stats, "PRMQuery::ConnectToCC");

  // Get the CC containing _inCC.
  vector<VID> cc;
  auto colorMap = this->GetColorMap();
  stapl::sequential::get_cc(*this->GetRoadmap()->GetGraph(), colorMap, _inCC, cc);

  // Try to join _toConnect to that CC using each connector.
  for(auto& label : m_ncLabels)
    this->GetConnector(label)->Connect(this->GetRoadmap(), _toConnect,
        cc.begin(), cc.end(), false);
}

/*----------------------------------------------------------------------------*/

#endif
