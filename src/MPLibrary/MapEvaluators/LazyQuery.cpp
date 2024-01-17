#include "LazyQuery.h"

#include "MPLibrary/MPLibrary.h"

#include <algorithm>
#include <functional>
#include <unordered_map>

/*------------------------------- Construction -------------------------------*/

LazyQuery::
LazyQuery() : QueryMethod() {
  this->SetName("LazyQuery");
}


LazyQuery::
LazyQuery(XMLNode& _node) : MapEvaluatorMethod(_node), QueryMethod(_node) {
  this->SetName("LazyQuery");

  m_vcLabel = _node.Read("vcLabel", true, "", "Lazy validity checker method.");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local planner method.");

  m_deleteInvalid = _node.Read("deleteInvalid", false, m_deleteInvalid,
      "Remove invalid vertices from the roadmap?");

  m_numEnhance = _node.Read("numEnhance", false, m_numEnhance,
      size_t(0), std::numeric_limits<size_t>::max(),
      "Number of nodes to generate in node enhancement");
  m_d = _node.Read("d", false, m_d,
      0., std::numeric_limits<double>::max(),
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

void
LazyQuery::
Print(std::ostream& _os) const {
  QueryMethod::Print(_os);
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


void
LazyQuery::
Initialize() {
  QueryMethod::Initialize();
  m_edges.clear();
  m_invalidVertices.clear();
  m_invalidEdges.clear();
}

/*--------------------------- QueryMethod Overrides --------------------------*/

void
LazyQuery::
SetPathWeightFunction(SSSPPathWeightFunction<RoadmapType> _f) {
  using EI = typename RoadmapType::adj_edge_iterator;

  // Wrap the requested weight function with a preceding check on invalidation.
  this->m_weightFunction = [this, _f](EI& _ei,
                                      const double _sourceDistance,
                                      const double _targetDistance) {
    if(this->IsEdgeInvalidated(_ei->id()))
      return std::numeric_limits<double>::infinity();
    return _f(_ei, _sourceDistance, _targetDistance);
  };
}


void
LazyQuery::
Reset(RoadmapType* const _r) {
  QueryMethod::Reset(_r);

  // Create storage for this roadmap's invalidations.
  m_invalidVertices[_r];
  m_invalidEdges[_r];
}


bool
LazyQuery::
PerformSubQuery(const VID _start, const VIDSet& _goals) {
  // Extract paths and validate them until there are no more left to try.
  while(QueryMethod::PerformSubQuery(_start, _goals)) {
    if(ValidatePath())
      return true;
  }

  // There are no valid paths, enhance and return false.
  NodeEnhance();
  return false;
}


double
LazyQuery::
StaticPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  // First check if the edge is lazily invalidated. If so, the distance is
  // infinite.
  if(this->IsEdgeInvalidated(_ei->id()))
    return std::numeric_limits<double>::infinity();

  return QueryMethod::StaticPathWeight(_ei, _sourceDistance,
      _targetDistance);
}


double
LazyQuery::
DynamicPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  // First check if the edge is lazily invalidated. If so, the distance is
  // infinite.
  if(this->IsEdgeInvalidated(_ei->id()))
    return std::numeric_limits<double>::infinity();

  return QueryMethod::DynamicPathWeight(_ei, _sourceDistance,
      _targetDistance);
}

/*--------------------------------- Helpers ----------------------------------*/

bool
LazyQuery::
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


bool
LazyQuery::
PruneInvalidVertices() {
  if(this->m_debug)
    std::cout << "\t\tChecking vertices..." << std::endl;

  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto path = this->GetPath();

  // Check each vertex in the path.
  for(size_t i = 0; i < path->Size(); ++i) {
    // Work from the outside towards the middle.
    const size_t index = i % 2 ? path->Size() - i / 2 - 1 : i / 2;
    const VID vid = path->VIDs()[index];

    // Skip checks if already validated.
    Cfg& cfg = this->m_roadmap->GetVertex(vid);
    if(cfg.IsLabel("VALID") && cfg.GetLabel("VALID"))
      continue;

    // Validate cfg. Move on to the next if it is valid.
    if(vc->IsValid(cfg, "LazyQuery::ValidatePath"))
      continue;

    // If we're here, the cfg is invalid.
    if(this->m_debug)
      std::cout << "\t\tNode " << vid << " found invalid during path validation."
                << std::endl;

    // Invalidate the cfg.
    InvalidateVertex(vid);
    return true;
  }

  if(this->m_debug)
    std::cout << "\t\tVertices are ok." << std::endl;

  return false;
}


bool
LazyQuery::
PruneInvalidEdges() {
  auto env = this->GetEnvironment();
  auto lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);
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
        this->m_roadmap->find_edge(ed, vi, edge);

        if(edge->property().IsChecked(res))
          continue;
        edge->property().SetChecked(res);
      }

      // Validate edge with local planner.
      Cfg witness;
      LPOutput lpo;
      const bool valid = lp->IsConnected(
          this->m_roadmap->GetVertex(v1), this->m_roadmap->GetVertex(v2),
          witness, &lpo,
          env->GetPositionRes() * res, env->GetOrientationRes() * res, true);

      // If the edge is valid, move on.
      if(valid)
        continue;

      if(this->m_debug)
        std::cout << "\n\t\tEdge (" << v1 << ", " << v2 << ") is invalid at "
                  << "resolultion factor " << res << "." << std::endl;

      // Invalidate the edge.
      ProcessInvalidNode(witness);
      InvalidateEdge(v1, v2);
      return true;
    }

    if(this->m_debug)
      std::cout << "ok." << std::endl;
  }

  if(this->m_debug)
    std::cout << "\t\tEdges are ok." << std::endl;

  return false;
}


void
LazyQuery::
NodeEnhance() {
  if(!m_numEnhance || m_edges.empty())
    return;

  if(this->m_debug)
    std::cout << "\tLazyQuery is enhancing nodes...\n\t  Generated VIDs:";

  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_enhanceDmLabel);
  auto roadmap = this->GetRoadmap();

  for(size_t i = 0; i < m_numEnhance and !m_edges.empty(); ++i) {
    // Pick a random edge from m_edges.
    const size_t index = LRand() % m_edges.size();

    // Get its midpoint and a random ray.
    const Cfg midpoint = (m_edges[index].first + m_edges[index].second) / 2.;
    Cfg ray(midpoint.GetRobot());
    ray.GetRandomRay(std::abs(GaussianDistribution(0, m_d)), dm);

    // Create an enhancement cfg by adding the random ray to the midpoint.
    Cfg enhance = midpoint + ray;
    enhance.SetLabel("Enhance", true);

    // If enchancement cfg is in bounds, add it to the roadmap and connect.
    if(enhance.InBounds(this->GetEnvironment())) {
      const VID newVID = roadmap->AddVertex(enhance);
      for(auto& label : m_ncLabels)
        this->GetMPLibrary()->GetConnector(label)->Connect(roadmap, newVID);
      if(this->m_debug)
        std::cout << " " << newVID;
    }

    // Release the enhancement edge.
    m_edges.erase(m_edges.begin() + index);
  }

  if(this->m_debug)
    std::cout << std::endl;
}


void
LazyQuery::
InvalidateVertex(const VID _vid) {
  // This part for LazyToggleQuery.
  const Cfg& cfg = this->m_roadmap->GetVertex(_vid);
  ProcessInvalidNode(cfg);

  // Collect the edge list (deleting as we go will invalidate our iterators).
  auto vi = this->m_roadmap->find_vertex(_vid);
  std::vector<std::pair<VID, VID>> edgeList;
  edgeList.reserve(vi->size());
  for(auto ei = vi->begin(); ei != vi->end(); ++ei)
    edgeList.emplace_back(ei->source(), ei->target());

  // Invalidate the edges.
  for(auto ei = edgeList.begin(); ei != edgeList.end(); ++ei)
    InvalidateEdge(ei->first, ei->second);

  // Delete or invalidate the vertex as appropriate.
  if(m_deleteInvalid)
    this->m_roadmap->DeleteVertex(_vid);
  else
    SetVertexInvalidated(_vid);
}


void
LazyQuery::
InvalidateEdge(const VID _source, const VID _target) {
  // Add the invalid edge to enhancement sampling list.
  if(m_numEnhance) {
    const Cfg& cfg1 = this->m_roadmap->GetVertex(_source),
             & cfg2 = this->m_roadmap->GetVertex(_target);
    if(!cfg1.IsLabel("Enhance") and !cfg2.IsLabel("Enhance")) {
      m_edges.emplace_back(cfg1, cfg2);
      if(this->m_debug)
        std::cout << "\t\t\tAdding node enhancement edge (" << _source << ", "
                  << _target << ")." << std::endl;
    }
  }

  // If we are deleting invalid edges, delete (_source, _target) and return.
  // Also delete the reverse edge if it exists.
  if(m_deleteInvalid) {
    this->m_roadmap->DeleteEdge(_source, _target);
    if(this->m_roadmap->IsEdge(_target, _source))
      this->m_roadmap->DeleteEdge(_target, _source);
  }
  // Otherwise, mark this edge (and its counterpart if applicable) as lazy
  // invalid.
  else {
    SetEdgeInvalidated(_source, _target);
    if(this->m_roadmap->IsEdge(_target, _source))
      SetEdgeInvalidated(_target, _source);
  }
}

/*---------------------------- Lazy Invalidation -----------------------------*/

bool
LazyQuery::
IsVertexInvalidated(const VID _vid) const noexcept {
  return m_invalidVertices.at(this->m_roadmap).count(_vid);
}


void
LazyQuery::
SetVertexInvalidated(const VID _vid) noexcept {
  m_invalidVertices.at(this->m_roadmap).insert(_vid);
}


bool
LazyQuery::
IsEdgeInvalidated(const EdgeID _eid) const noexcept {
  return m_invalidEdges.at(this->m_roadmap).count(_eid);
}


bool
LazyQuery::
IsEdgeInvalidated(const VID _source, const VID _target) const noexcept {
  typename RoadmapType::CEI ei;
  if(!this->m_roadmap->GetEdge(_source, _target, ei))
    throw RunTimeException(WHERE) << "Requested non-existent edge ("
                                  << _source << ", " << _target << ").";
  return IsEdgeInvalidated(ei->id());
}


void
LazyQuery::
SetEdgeInvalidated(const EdgeID _eid) noexcept {
  m_invalidEdges.at(this->m_roadmap).insert(_eid);
}


void
LazyQuery::
SetEdgeInvalidated(const VID _source, const VID _target) noexcept {
  typename RoadmapType::EI ei;
  if(!this->m_roadmap->GetEdge(_source, _target, ei))
    throw RunTimeException(WHERE) << "Requested non-existent edge ("
                                  << _source << ", " << _target << ").";
  SetEdgeInvalidated(ei->id());
}
