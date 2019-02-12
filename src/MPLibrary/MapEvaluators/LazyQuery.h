#ifndef PMPL_LAZY_QUERY_H_
#define PMPL_LAZY_QUERY_H_

#include "QueryMethod.h"

#include <algorithm>
#include <functional>
#include <unordered_map>


////////////////////////////////////////////////////////////////////////////////
/// First assumes all nodes and edges are valid, then checks for validity of the
/// nodes/edges used in the path.
///
/// Reference:
///   Robert Bohlin and Lydia E. Kavraki. "Path Planning Using Lazy PRM".
///   ICRA 2000.
///
/// @note Node enhancement does not work like in the paper. Here we use a
///       flat gaussian distribution with fixed distance.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LazyQuery : public QueryMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType              CfgType;
    typedef typename MPTraits::RoadmapType          RoadmapType;
    typedef typename RoadmapType::VID               VID;
    typedef typename RoadmapType::EID::edge_id_type EID;
    typedef typename MPTraits::GoalTracker          GoalTracker;
    typedef typename GoalTracker::VIDSet            VIDSet;

    ///@}
    ///@name Construction
    ///@{

    LazyQuery();
    LazyQuery(XMLNode& _node);
    virtual ~LazyQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

    ///@}

  protected:

    ///@name QueryMethod Overrides
    ///@{

    virtual bool PerformSubQuery(const VID _start, const VIDSet& _goals) override;

    ///@}
    ///@name Helpers
    ///@{

    /// Checks validity of nodes and edges and deletes any invalid ones.
    /// @return True if the path was valid.
    bool ValidatePath();

    /// Check each vertex and ensure it is valid. Upon discovering an invalid
    /// vertex, delete it and return.
    /// @return True if a vertex was deleted.
    bool PruneInvalidVertices();

    /// Check each edge and ensure it is valid. Upon discovering an invalid edge,
    /// delete it and return.
    /// @return True if an edge was deleted.
    bool PruneInvalidEdges();

    /// Choose a random deleted edge and generate nodes with a gaussian
    /// distribution around the edge's midpoint.
    virtual void NodeEnhance();

    /// Additional handling of invalid vertices.
    /// @param _cfg The invalid configuration to handle.
    virtual void ProcessInvalidNode(const CfgType& _cfg) { }

    /// Invalidate a roadmap configuration.
    /// @param _vid The vertex descriptor.
    void InvalidateVertex(const VID _vid);

    /// Invalidate a roadmap edge.
    /// @param _source The source vertex descriptor.
    /// @param _target The target vertex descriptor.
    void InvalidateEdge(const VID _source, const VID _target);

    ///@}
    ///@name MP Object Labels
    ///@{

    std::string m_vcLabel;         ///< The lazy validity checker label.
    std::string m_lpLabel;         ///< The lazy local planner label.
    std::string m_enhanceDmLabel;  ///< The distance metric for enhancement.

    std::vector<std::string> m_ncLabels; ///< The connectors for enhancement.

    ///@}
    ///@name Internal State
    ///@{

    bool m_deleteInvalid{true};   ///< Remove invalid vertices from the roadmap?

    vector<int> m_resolutions{1}; ///< List of resolution multiples to check.
    int m_numEnhance{0};          ///< Number of enhancement nodes to generate.
    double m_d{0};                ///< Gaussian distance for enhancement sampling.
    std::vector<std::pair<CfgType, CfgType>> m_edges; ///< Candidate edges.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
LazyQuery<MPTraits>::
LazyQuery() : QueryMethod<MPTraits>() {
  this->SetName("LazyQuery");
}


template <typename MPTraits>
LazyQuery<MPTraits>::
LazyQuery(XMLNode& _node) : QueryMethod<MPTraits>(_node) {
  this->SetName("LazyQuery");

  m_vcLabel = _node.Read("vcLabel", true, "", "Lazy validity checker method.");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local planner method.");

  m_deleteInvalid = _node.Read("deleteInvalid", false, m_deleteInvalid,
      "Remove invalid vertices from the roadmap?");

  m_numEnhance = _node.Read("numEnhance", false, m_numEnhance, 0, MAX_INT,
      "Number of nodes to generate in node enhancement");
  m_d = _node.Read("d", false, m_d, 0., MAX_DBL,
      "Gaussian d value for node enhancement");
  m_enhanceDmLabel = _node.Read("enhanceDmLabel", m_numEnhance, "",
      "Distance metric method for generating enhancement nodes.");

  for(auto& child : _node) {
    if(child.Name() == "Resolution")
      m_resolutions.push_back(child.Read("mult", true, 1, 1, MAX_INT,
          "Multiple of finest resolution checked, >= 1. Higher resolutions are "
          "coarser initial checks."));
    else if(child.Name() == "NodeConnectionMethod")
      m_ncLabels.push_back(child.Read("label", true, "",
          "Connector method for enhancement nodes."));
  }

  // Sort resolutions in decreasing order.
  std::sort(m_resolutions.begin(), m_resolutions.end(), std::greater<int>());
  auto iter = std::unique(m_resolutions.begin(), m_resolutions.end());
  m_resolutions.erase(iter, m_resolutions.end());

  // Ensure that resolution '1' is included.
  if(m_resolutions.back() != 1)
    throw RunTimeException(WHERE) << "Last resolution should be '1', but it is '"
                                  << m_resolutions.back() << "'.";
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
LazyQuery<MPTraits>::
Print(std::ostream& _os) const {
  QueryMethod<MPTraits>::Print(_os);
  _os << "\tEnhancement distance Metric: " << m_enhanceDmLabel
      << "\n\tLocal Planner: " << m_lpLabel
      << "\n\tValidity Checker: " << m_vcLabel
      << "\n\tDelete Invalid: " << m_deleteInvalid
      << "\n\tnumEnhance: " << m_numEnhance
      << "\n\td: " << m_d
      << "\n\tresolutions:";
  for(const auto r : m_resolutions)
    _os << " " << r;
  _os << std::endl;
}


template <typename MPTraits>
void
LazyQuery<MPTraits>::
Initialize() {
  QueryMethod<MPTraits>::Initialize();
  m_edges.clear();
}

/*--------------------------- QueryMethod Overrides --------------------------*/

template <typename MPTraits>
bool
LazyQuery<MPTraits>::
PerformSubQuery(const VID _start, const VIDSet& _goals) {
  // Extract paths and validate them until there are no more left to try.
  while(QueryMethod<MPTraits>::PerformSubQuery(_start, _goals)) {
    if(ValidatePath())
      return true;
  }

  // There are no valid paths, enhance and return false.
  NodeEnhance();
  return false;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
LazyQuery<MPTraits>::
ValidatePath() {
  auto path = this->GetPath();

  if(this->m_debug) {
    std::cout << "\tValidating path for lazy query...\n\t";
    for(const auto vid : path->VIDs())
      std::cout << "  " << vid;
    std::cout << std::endl;
  }

  // Check vertices and edges for validity. If any are removed, the path is
  // invalid.
  if(path->Size() == 0 or PruneInvalidVertices() or PruneInvalidEdges()) {
    path->Clear();
    if(this->m_debug)
      std::cout << "\tPath is invalid." << std::endl;
    return false;
  }

  if(this->m_debug)
    std::cout << "\tPath is valid." << std::endl;
  return true;
}


template <typename MPTraits>
bool
LazyQuery<MPTraits>::
PruneInvalidVertices() {
  if(this->m_debug)
    std::cout << "\t\tChecking vertices..." << std::endl;

  auto g  = this->GetRoadmap();
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto path = this->GetPath();

  // Check each vertex in the path.
  for(size_t i = 0; i < path->Size(); ++i) {
    // Work from the outside towards the middle.
    const size_t index = i % 2 ? path->Size() - i / 2 - 1 : i / 2;
    const VID vid = path->VIDs()[index];

    // Skip checks if already validated.
    CfgType& cfg = g->GetVertex(vid);
    if(cfg.IsLabel("VALID") && cfg.GetLabel("VALID"))
      continue;

    // Validate cfg. Move on to the next if it is valid.
    if(vc->IsValid(cfg, "LazyQuery::ValidatePath"))
      continue;

    // If we're here, the cfg is invalid.
    if(this->m_debug)
      std::cout << "\t\tNode " << vid << " found invalid during path validation."
                << std::endl;

    // Delete invalid vertex.
    InvalidateVertex(vid);
    return true;
  }

  if(this->m_debug)
    std::cout << "\t\tVertices are ok." << std::endl;

  return false;
}


template <typename MPTraits>
bool
LazyQuery<MPTraits>::
PruneInvalidEdges() {
  auto g = this->GetRoadmap();
  auto env = this->GetEnvironment();
  auto lp = this->GetLocalPlanner(m_lpLabel);
  auto path = this->GetPath();

  if(this->m_debug)
    std::cout << "\t\tChecking edges..." << std::endl;

  // Perform the check for each resolution.
  for(const auto res : m_resolutions) {
    if(this->m_debug)
      std::cout << "\t\tChecking with resolution " << res << "...";

    for(size_t i = 0; i < path->Size() - 1; ++i) {
      // Check from outside to middle
      const size_t index = i % 2 ? path->Size() - i / 2 - 2 : i / 2;
      const VID v1 = path->VIDs()[index],
                v2 = path->VIDs()[index + 1];

      // Skip checks if already checked and valid.
      {
        typename RoadmapType::edge_descriptor ed(v1, v2);
        typename RoadmapType::vertex_iterator vi;
        typename RoadmapType::adj_edge_iterator edge;
        g->find_edge(ed, vi, edge);

        if(edge->property().IsChecked(res))
          continue;
        edge->property().SetChecked(res);
      }

      // Validate edge with local planner.
      CfgType witness;
      LPOutput<MPTraits> lpo;
      const bool valid = lp->IsConnected(g->GetVertex(v1), g->GetVertex(v2),
          witness, &lpo,
          env->GetPositionRes() * res, env->GetOrientationRes() * res, true);

      // Handle invalid edges.
      if(!valid) {
        if(this->m_debug)
          std::cout << "\n\t\tEdge (" << v1 << ", " << v2 << ") is invalid at "
                    << "resolultion factor " << res << "." << std::endl;

        // Delete invalid edge.
        ProcessInvalidNode(witness);
        InvalidateEdge(v1, v2);
        return true;
      }
    }

    if(this->m_debug)
      std::cout << "ok." << std::endl;
  }

  if(this->m_debug)
    std::cout << "\t\tEdges are ok." << std::endl;

  return false;
}


template <typename MPTraits>
void
LazyQuery<MPTraits>::
NodeEnhance() {
  if(!m_numEnhance || m_edges.empty())
    return;

  if(this->m_debug)
    std::cout << "\tLazyQuery is enhancing nodes...\n\t  Generated VIDs:";

  auto dm = this->GetDistanceMetric(m_enhanceDmLabel);
  auto roadmap = this->GetRoadmap();

  for(int i = 0; i < m_numEnhance and !m_edges.empty(); ++i) {
    // Pick a random edge from m_edges.
    const size_t index = LRand() % m_edges.size();

    // Get its midpoint and a random ray.
    const CfgType midpoint = (m_edges[index].first + m_edges[index].second) / 2.;
    CfgType ray(midpoint.GetRobot());
    ray.GetRandomRay(std::abs(GaussianDistribution(0, m_d)), dm);

    // Create an enhancement cfg by adding the random ray to the midpoint.
    CfgType enhance = midpoint + ray;
    enhance.SetLabel("Enhance", true);

    // If enchancement cfg is in bounds, add it to the roadmap and connect.
    if(enhance.InBounds(this->GetEnvironment())) {
      const VID newVID = roadmap->AddVertex(enhance);
      for(auto& label : m_ncLabels)
        this->GetConnector(label)->Connect(roadmap, newVID);
      if(this->m_debug)
        std::cout << " " << newVID;
    }

    // Release the enhancement edge.
    m_edges.erase(m_edges.begin() + index);
  }

  if(this->m_debug)
    std::cout << std::endl;
}


template <typename MPTraits>
void
LazyQuery<MPTraits>::
InvalidateVertex(const VID _vid) {
  auto g = this->GetRoadmap();

  const CfgType& cfg = g->GetVertex(_vid);
  ProcessInvalidNode(cfg);

  // If we are deleting invalid vertices, delete _vid and return.
  if(m_deleteInvalid) {
    g->DeleteVertex(_vid);
    return;
  }

  // Otherwise, mark this vertex and its edges as lazy invalid.
  g->SetVertexInvalidated(_vid, true);
  auto vi = g->find_vertex(_vid);
  for(auto ei = vi->begin(); ei != vi->end(); ++ei)
    g->SetEdgeInvalidated(ei->id(), true);
}


template <typename MPTraits>
void
LazyQuery<MPTraits>::
InvalidateEdge(const VID _source, const VID _target) {
  auto g = this->GetRoadmap();

  // Add the invalid edge to enhancement sampling list.
  if(m_numEnhance) {
    const CfgType& cfg1 = g->GetVertex(_source),
                 & cfg2 = g->GetVertex(_target);
    if(!cfg1.IsLabel("Enhance") and !cfg2.IsLabel("Enhance")) {
      m_edges.emplace_back(cfg1, cfg2);
      if(this->m_debug)
        std::cout << "\t\t\tAdding node enhancement edge (" << _source << ", "
                  << _target << ")." << std::endl;
    }
  }

  // If we are deleting invalid edges, delete (_source, _target) and return.
  if(m_deleteInvalid)
    g->DeleteEdge(_source, _target);
  // Otherwise, mark this edge as lazy invalid.
  else
    g->SetEdgeInvalidated(_source, _target, true);
}

/*----------------------------------------------------------------------------*/

#endif
