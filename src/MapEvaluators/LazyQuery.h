#ifndef LAZY_QUERY_H_
#define LAZY_QUERY_H_

#include "PRMQuery.h"
#include <functional>
#include <algorithm>

using namespace std;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief Lazy @prm, extract path, validate, and repeat
/// @tparam MPTraits Motion planning universe
///
/// First assumes all nodes and edges are valid, then checks for validity in the
/// query phase and deletes nodes and edges found to be invalid.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LazyQuery : public PRMQuery<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::MPProblemType    MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType   GraphType;
    typedef typename MPProblemType::VID         VID;

    ///@}
    ///\name Construction
    ///@{

    LazyQuery();
    LazyQuery(MPProblemType* _problem, XMLNode& _node);
    virtual ~LazyQuery() = default;

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
    /// \brief Checks validity of nodes and edges and deletes any invalid ones.
    /// \return True if the path was valid.
    bool ValidatePath();

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Check each vertex and ensure it is valid. Upon discovering an
    ///        invalid vertex, delete it and return.
    /// \return True if a vertex was deleted.
    bool PruneInvalidVertices();

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Check each edge and ensure it is valid. Upon discovering an
    ///        invalid edge, delete it and return.
    /// \return True if an edge was deleted.
    bool PruneInvalidEdges();

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Choose a random deleted edge and generate nodes with a gaussian
    ///        distribution around the edge's midpoint.
    virtual void NodeEnhance();

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Additional handling of invalid vertices.
    /// \param[in] _cfg The invalid configuration to handle.
    virtual void ProcessInvalidNode(const CfgType& _cfg) { }

    ///@}
    ///\name MP Object Labels
    ///@{
    /// These objects are used to lazily validate the computed path.

    string m_dmLabel{"euclidean"};
    string m_vcLabel{"pqp_solid"};
    string m_lpLabel{"sl"};

    ///@}
    ///\name Enhancement State
    ///@{

    vector<int> m_resolutions{1}; ///< List of resolution multiples to check.
    int m_numEnhance{0};          ///< Number of enhancement nodes to generate.
    double m_d{0};                ///< Gaussian distance for enhancement sampling.
    vector<pair<CfgType, CfgType>> m_edges; ///< Candidate edges.

    ///@}
    ///\name Unhide QueryMethod names.
    ///@{

    using QueryMethod<MPTraits>::m_path;

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
LazyQuery<MPTraits>::
LazyQuery() : PRMQuery<MPTraits>() {
  this->SetName("LazyQuery");
}


template <typename MPTraits>
LazyQuery<MPTraits>::
LazyQuery(MPProblemType* _problem, XMLNode& _node) :
    PRMQuery<MPTraits>(_problem, _node) {
  this->SetName("LazyQuery");
  ParseXML(_node);
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
LazyQuery<MPTraits>::
ParseXML(XMLNode& _node) {
  m_dmLabel = _node.Read("dmLabel", false, m_dmLabel, "Distance metric method");
  m_vcLabel = _node.Read("vcMethod", false, m_vcLabel, "Validity checker method");
  m_lpLabel = _node.Read("lpLabel", false, m_lpLabel, "Local planner method");

  m_numEnhance = _node.Read("numEnhance", false, m_numEnhance, 0, MAX_INT,
      "Number of nodes to generate in node enhancement");
  m_d = _node.Read("d", false, m_d, 0., MAX_DBL, "Gaussian d value for node "
      "enhancement");

  for(auto& child : _node)
    if(child.Name() == "Resolution")
      m_resolutions.push_back(child.Read("mult", true, 1, 1, MAX_INT,
          "Multiple of finest resolution checked"));

  // Sort resolutions in decreasing order, ensure that '1' is included
  sort(m_resolutions.begin(), m_resolutions.end(), greater<int>());
  auto iter = unique(m_resolutions.begin(), m_resolutions.end());
  m_resolutions.erase(iter, m_resolutions.end());
  if(m_resolutions.back() != 1)
    throw RunTimeException(WHERE, "Last resolution should be 1, but it is "
        + to_string(m_resolutions.back()) + ".");
}


template <typename MPTraits>
void
LazyQuery<MPTraits>::
Print(ostream& _os) const {
  PRMQuery<MPTraits>::Print(_os);
  _os << "\tDistance Metric: " << m_dmLabel
      << "\n\tValidity Checker: " << m_vcLabel
      << "\n\tLocal Planner: " << m_lpLabel
      << "\n\tnumEnhance: " << m_numEnhance
      << "\n\td: " << m_d
      << "\n\tresolutions:";
  for(const auto r : m_resolutions)
    _os << " " << r;
  _os << endl;
}

/*--------------------------- QueryMethod Overrides --------------------------*/

template <typename MPTraits>
bool
LazyQuery<MPTraits>::
PerformSubQuery(const CfgType& _start, const CfgType& _goal) {
  if(this->m_debug)
    cout << "Evaluating sub-query:" << endl
         << "\tfrom " << _start << endl
         << "\tto   " << _goal << endl;

  // Find connected components.
  auto ccs = this->FindCCs();

  // Add start and goal to roadmap (if not already there).
  auto start = this->EnsureCfgInMap(_start);
  auto goal  = this->EnsureCfgInMap(_goal);

  // Check each connected component.
  bool connected = false;
  for(auto cc : ccs) {
    // Try connecting the start and goal to this CC.
    this->ConnectToCC(start.first, cc.second);
    this->ConnectToCC(goal.first, cc.second);

    // If start and goal are connected to the same CC, generate path and end.
    while(!connected && this->SameCC(start.first, goal.first)) {
      this->GeneratePath(start.first, goal.first);
      connected = this->ValidatePath();
    }

    // Quit when we find a valid path.
    if(connected)
      break;
  }

  if(this->m_debug) {
    if(connected)
      cout << "\tSuccess: found path from start node " << start.first
           << " to goal node " << goal.first << "." << endl;
    else
      cout << "\tFailed to connect start node " << start.first
           << " to goal node " << goal.first << "." << endl;
  }

  // Use node enhancement.
  if(!connected)
    NodeEnhance();

  // Remove start and goal if necessary.
  if(this->m_deleteNodes) {
    this->RemoveTempCfg(start);
    this->RemoveTempCfg(goal);
  }

  return connected;
}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
bool
LazyQuery<MPTraits>::
ValidatePath() {
  if(this->m_debug)
    cout << "\tValidating path for lazy query..." << endl;

  // Check vertices and edges for validity. If any are removed, the path is
  // invalid.
  if(PruneInvalidVertices() || PruneInvalidEdges()) {
    m_path->Clear();
    if(this->m_debug)
      cout << "\tPath is invalid." << endl;
    return false;
  }
  else {
    if(this->m_debug)
      cout << "\tPath is valid." << endl;
    return true;
  }
}


template <typename MPTraits>
bool
LazyQuery<MPTraits>::
PruneInvalidVertices() {
  if(this->m_debug)
    cout << "\t\tChecking vertices..." << endl;

  auto g  = m_path->GetRoadmap()->GetGraph();
  auto vc = this->GetValidityChecker(m_vcLabel);

  // Check each vertex in the path.
  for(size_t i = 0; i < m_path->Size(); ++i) {
    // Work from the outside towards the middle.
    const size_t index = i % 2 ? m_path->Size() - i / 2 - 1 : i / 2;
    const VID vid = m_path->VIDs()[index];

    // Skip checks if already validated.
    CfgType& cfg = g->GetVertex(vid);
    if(cfg.IsLabel("VALID") && cfg.GetLabel("VALID"))
      continue;

    // Validate cfg. Move on to the next if it is valid.
    if(vc->IsValid(cfg, "LazyQuery::ValidatePath"))
      continue;

    // If we're here, the cfg is invalid.
    if(this->m_debug)
      cout << "\t\tNode " << vid << " found invalid during path validation.\n"
           << "\t\t\tAdding enhancement edges:";

    // Collect enhancment info.
    if(m_numEnhance && !cfg.IsLabel("Enhance")) {
      auto vertex = g->find_vertex(vid);
      for(auto edge = vertex->begin(); edge != vertex->end(); ++edge) {
        CfgType& target = g->GetVertex(edge->target());
        if(!target.IsLabel("Enhance")) {
          m_edges.push_back(make_pair(cfg, target));
          if(this->m_debug)
            cout << " (" << vid << ", " << edge->target() << ")";
        }
      }
    }
    if(this->m_debug)
      cout << endl;

    // Delete invalid vertex.
    this->ProcessInvalidNode(cfg);
    g->delete_vertex(vid);
    return true;
  }

  if(this->m_debug)
    cout << "\t\tVertices are ok." << endl;

  return false;
}


template <typename MPTraits>
bool
LazyQuery<MPTraits>::
PruneInvalidEdges() {
  auto g = m_path->GetRoadmap()->GetGraph();
  auto env = this->GetEnvironment();
  auto lp = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);

  if(this->m_debug)
    cout << "\t\tChecking edges..." << endl;

  // Perform the check for each resolution.
  for(const auto res : m_resolutions) {
    if(this->m_debug)
      cout << "\t\tChecking with resolution " << res << "...";

    for(size_t i = 0; i < m_path->Size() - 1; ++i) {
      // Check from outside to middle
      size_t index = i % 2 ? m_path->Size() - i / 2 - 2 : i / 2;
      VID v1 = m_path->VIDs()[index];
      VID v2 = m_path->VIDs()[index + 1];

      // Get graph edge iterator.
      typename GraphType::adj_edge_iterator edge;
      {
        typename GraphType::vertex_iterator vi;
        typename GraphType::edge_descriptor ed(v1, v2);
        g->find_edge(ed, vi, edge);
      }

      // Skip checks if already checked and valid
      if(edge->property().IsChecked(res))
        continue;

      // Validate edge with local planner.
      CfgType witness;
      LPOutput<MPTraits> lpo;
      if(lp->IsConnected(g->GetVertex(v1), g->GetVertex(v2), witness, &lpo,
            env->GetPositionRes() * res, env->GetOrientationRes() * res, true)) {
        // The edge is valid: mark it as checked at this resolution.
        edge->property().SetChecked(res);
      }
      else {
        // The edge is invalid.
        if(this->m_debug)
          cout << "\n\t\tEdge (" << v1 << ", " << v2 << ") is invalid at "
               << "resolultion factor " << res << "." << endl;

        // Add invalid edge to enhancement sampling list.
        if(m_numEnhance) {
          CfgType cfg1 = g->GetVertex(v1);
          CfgType cfg2 = g->GetVertex(v2);
          if(!cfg1.IsLabel("Enhance") && !cfg2.IsLabel("Enhance")) {
            m_edges.push_back(make_pair(cfg1, cfg2));
            if(this->m_debug)
              cout << "\t\t\tAdding node enhancement edge (" << v1 << ", " << v2
                   << ")." << endl;
          }
        }

        // Delete invalid (bidirectional) edge.
        this->ProcessInvalidNode(witness);
        g->delete_edge(v1, v2);
        g->delete_edge(v2, v1);
        return true;
      }
    }

    if(this->m_debug)
      cout << "ok." << endl;
  }

  if(this->m_debug)
    cout << "\t\tEdges are ok." << endl;

  return false;
}


template <typename MPTraits>
void
LazyQuery<MPTraits>::
NodeEnhance() {
  if(!m_numEnhance || m_edges.empty())
    return;

  if(this->m_debug)
    cout << "\tLazyQuery is enhancing nodes...\n\t  Generated VIDs:";

  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto roadmap = m_path->GetRoadmap();

  for(int i = 0; i < m_numEnhance; ++i) {
    // Pick a random edge from m_edges.
    size_t index = LRand() % m_edges.size();

    // Get its midpoint and a random ray.
    CfgType midpoint = (m_edges[index].first + m_edges[index].second) / 2.;
    CfgType ray;
    ray.GetRandomRay(fabs(GaussianDistribution(fabs(m_d), fabs(m_d))), dm);

    // Create an enhancement cfg by adding the random ray to the midpoint.
    CfgType enhance = midpoint + ray;
    enhance.SetLabel("Enhance", true);

    // If enchancement cfg is in bounds, add it to the roadmap and connect.
    if(this->GetEnvironment()->InBounds(enhance)) {
      VID newVID = roadmap->GetGraph()->AddVertex(enhance);
      for(auto& label : this->m_ncLabels)
        this->GetConnector(label)->Connect(roadmap, newVID);
      if(this->m_debug)
        cout << " " << newVID;
    }
  }

  if(this->m_debug)
    cout << endl;
}

/*----------------------------------------------------------------------------*/

#endif
