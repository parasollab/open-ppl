#ifndef PRM_QUERY_H_
#define PRM_QUERY_H_

#include "QueryMethod.h"

#include "LocalPlanners/LPOutput.h"
#include "LocalPlanners/StraightLine.h"
#include "Utilities/MetricUtils.h"

#include <containers/sequential/graph/algorithms/astar.h>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief Evaluate a roadmap under construction to see if a query has been
///        satisfied.
/// @tparam MPTraits Motion planning universe
///
/// This query is specialized for PRM methods.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PRMQuery : public QueryMethod<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::CfgRef           CfgRef;
    typedef typename MPTraits::WeightType       WeightType;
    typedef typename MPTraits::MPProblemType    MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType   GraphType;
    typedef typename MPProblemType::VID         VID;

    ///@}
    ///\name Construction
    ///@{

    PRMQuery();
    PRMQuery(MPProblemType* _problem, XMLNode& _node);
    virtual ~PRMQuery() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const override;

    ///@}
    ///\name QueryMethod Overrides
    ///@{

    virtual bool PerformSubQuery(const CfgType& _start, const CfgType& _goal)
        override;

    ///@}

  protected:

    ///\name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Ensure a given configuration is in the roadmap, adding it if
    ///        necessary.
    /// \param[in] _cfg The configuration to ensure.
    /// \return The VID of _cfg, and a bool indicating whether or not it had to
    ///         be added to the map.
    pair<VID, bool> EnsureCfgInMap(const CfgType& _cfg);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Remove an ensured configuration if it was added to the map.
    /// \param[in] _temp The ensured configuration's VID and an indicator of
    ///                  whether or not it was added as a temporary.
    void RemoveTempCfg(pair<VID, bool> _temp);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Get the CC stats of the roadmap.
    /// \return A vector with one element per CC. The elements contain the size
    ///         of a CC and one VID within it.
    vector<pair<size_t, VID>> FindCCs();

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Try to connect a given VID to the CC containing a second VID.
    /// \param[in] _toConnect The VID to connect.
    /// \param[in] _inCC One of the VIDs in the CC of interest.
    void ConnectToCC(const VID _toConnect, const VID _inCC);

    ///@}
    ///\name MP Object Labels
    ///@{

    vector<string> m_ncLabels{"kClosest"};

    ///@}
    ///\name Graph Search
    ///@{

    bool m_deleteNodes{false};     // Delete any added nodes?

    ///@}
    ///\name Unhide QueryMethod names.
    ///@{

    using QueryMethod<MPTraits>::m_path;

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
PRMQuery(MPProblemType* _problem, XMLNode& _node) :
    QueryMethod<MPTraits>(_problem, _node) {
  this->SetName("PRMQuery");
  ParseXML(_node);
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
PRMQuery<MPTraits>::
ParseXML(XMLNode& _node) {
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

  // Find connected components.
  auto ccs = FindCCs();

  // Add start and goal to roadmap (if not already there).
  auto start = EnsureCfgInMap(_start);
  auto goal  = EnsureCfgInMap(_goal);

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
  auto g = m_path->GetRoadmap()->GetGraph();
  return g->IsVertex(_cfg) ? make_pair(g->GetVID(_cfg), false) :
                             make_pair(g->AddVertex(_cfg), true) ;
}


template <typename MPTraits>
void
PRMQuery<MPTraits>::
RemoveTempCfg(pair<VID, bool> _temp) {
  if(_temp.second)
    m_path->GetRoadmap()->GetGraph()->delete_vertex(_temp.first);
}


template <typename MPTraits>
vector<pair<size_t, typename PRMQuery<MPTraits>::VID>>
PRMQuery<MPTraits>::
FindCCs() {
  auto stats = this->GetStatClass();
  stats->IncGOStat("CC Operations");

  stats->StartClock("PRMQuery::FindCCs");
  vector<pair<size_t, VID>> ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  get_cc_stats(*m_path->GetRoadmap()->GetGraph(), cmap, ccs);
  stats->StopClock("PRMQuery::FindCCs");

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
  stats->IncGOStat("CC Operations");
  stats->StartClock("PRMQuery::ConnectToCC");

  // Get the CC containing _inCC.
  vector<VID> cc;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  stapl::sequential::get_cc(*m_path->GetRoadmap()->GetGraph(), cmap, _inCC, cc);

  // Try to join _toConnect to that CC using each connector.
  for(auto& label : m_ncLabels)
    this->GetConnector(label)->Connect(m_path->GetRoadmap(), _toConnect,
        cc.begin(), cc.end(), false);

  stats->StopClock("PRMQuery::ConnectToCC");
}

/*----------------------------------------------------------------------------*/

#endif
