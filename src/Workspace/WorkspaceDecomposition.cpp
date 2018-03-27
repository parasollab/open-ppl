#include "WorkspaceDecomposition.h"

#include "Geometry/Boundaries/TetrahedralBoundary.h"

#include "containers/sequential/graph/algorithms/dijkstra.h"

#include <unordered_map>

#ifndef INVALID_VID
#define INVALID_VID (std::numeric_limits<size_t>::max())
#endif


/*------------------------------- Construction -------------------------------*/

WorkspaceDecomposition::
WorkspaceDecomposition() = default;


WorkspaceDecomposition::
WorkspaceDecomposition(const WorkspaceDecomposition& _other) {
  *this = _other;
}


WorkspaceDecomposition::
WorkspaceDecomposition(WorkspaceDecomposition&& _other) {
  *this = std::move(_other);
}


WorkspaceDecomposition::
~WorkspaceDecomposition() = default;

/*-------------------------------- Assignment --------------------------------*/

WorkspaceDecomposition&
WorkspaceDecomposition::
operator=(const WorkspaceDecomposition& _other) {
  // Copy base class parts first.
  static_cast<RegionGraph&>(*this) = static_cast<const RegionGraph&>(_other);

  // Copy extra bits.
  m_points = _other.m_points;
  m_dual = _other.m_dual;

  UpdateDecompositionPointers();

  m_finalized = _other.m_finalized;
  return *this;
}


WorkspaceDecomposition&
WorkspaceDecomposition::
operator=(WorkspaceDecomposition&& _other) {
  // Move base class parts first.
  static_cast<RegionGraph&>(*this) = std::move(static_cast<RegionGraph&>(_other));

  // Move extra bits.
  m_points = std::move(_other.m_points);
  m_dual = std::move(_other.m_dual);

  UpdateDecompositionPointers();

  m_finalized = _other.m_finalized;
  return *this;
}

/*----------------------------- Point Accessors ------------------------------*/

const size_t
WorkspaceDecomposition::
GetNumPoints() const noexcept {
  return m_points.size();
}


const Point3d&
WorkspaceDecomposition::
GetPoint(const size_t _i) const noexcept {
  return m_points[_i];
}


const std::vector<Point3d>&
WorkspaceDecomposition::
GetPoints() const noexcept {
  return m_points;
}

/*----------------------------- Region Accessors -----------------------------*/

const size_t
WorkspaceDecomposition::
GetNumRegions() const noexcept {
  return get_num_vertices();
}


const WorkspaceRegion&
WorkspaceDecomposition::
GetRegion(const size_t _i) const noexcept {
  return find_vertex(_i)->property();
}


const WorkspaceDecomposition::vertex_descriptor
WorkspaceDecomposition::
GetDescriptor(const WorkspaceRegion& _region) const noexcept {
  for(const_vertex_iterator iter = this->begin(); iter != this->end(); ++iter)
    if(iter->property() == _region)
      return iter->descriptor();

  return INVALID_VID;
}

/*----------------------------- Accessors ------------------------------------*/

const WorkspacePortal&
WorkspaceDecomposition::
GetPortal(const size_t _i1, const size_t _i2) const noexcept {
  const_vertex_iterator vert;
  const_adj_edge_iterator edge;
  RegionGraph::edge_descriptor ed(_i1, _i2);
  find_edge(ed, vert, edge);
  return edge->property();
}

/*---------------------------- Dual Graph Accessors --------------------------*/

WorkspaceDecomposition::DualGraph&
WorkspaceDecomposition::
GetDualGraph() noexcept {
  return m_dual;
}


const WorkspaceDecomposition::DualGraph&
WorkspaceDecomposition::
GetDualGraph() const noexcept {
  return m_dual;
}

/*----------------------------- Modifiers ------------------------------------*/

void
WorkspaceDecomposition::
AddPoint(const Point3d& _p) {
  AssertMutable();
  m_points.push_back(_p);
}


void
WorkspaceDecomposition::
AddTetrahedralRegion(const int _pts[4]) {
  AssertMutable();
  WorkspaceRegion wr(this);

  // Add points to the region.
  for(size_t i = 0; i < 4; ++i)
    wr.AddPoint(_pts[i]);

  // Add facets to the region. Create a facet for each combination of points,
  // and ensure that their normals face away from the center point.
  const Point3d center = wr.FindCenter();
  for(size_t i = 0; i < 4; ++i) {
    WorkspaceRegion::Facet f(_pts[i], _pts[(i + 1) % 4], _pts[(i + 2) % 4],
        m_points);
    f.ComputeNormal();
    if(f.PointIsAbove(center))
      f.Reverse();
    wr.AddFacet(move(f));
  }

  // Create a tetrahedral boundary object for the region.
  /// @TODO The boundary currently double-stores the points. We would like to
  /// have a non-mutable boundary that holds only references to the points.
  wr.AddBoundary(std::unique_ptr<TetrahedralBoundary>(
      new TetrahedralBoundary(wr.GetPoints())
  ));

  // Add region to the graph.
  add_vertex(std::move(wr));
}


void
WorkspaceDecomposition::
AddPortal(const size_t _s, const size_t _t) {
  AssertMutable();
  add_edge(_s, _t, WorkspacePortal(this, _s, _t));
}


void
WorkspaceDecomposition::
Finalize() {
  ComputeDualGraph();
  m_finalized = true;
}

/*------------------------------------ I/O -----------------------------------*/

void
WorkspaceDecomposition::
Print(std::ostream& _os) const {
  // Print counts.
  _os << "WorkspaceDecomposition " << this
      << "\n\tNum vertices: " << this->get_num_vertices()
      << "\n\tNum edges: " << this->get_num_edges();

  for(auto vi = this->begin(); vi != this->end(); ++vi) {
    _os << "\n\t  Vertex " << vi->descriptor() << " " << &vi->property(); /// @TODO print properties
    for(auto ei = vi->begin(); ei != vi->end(); ++ei)
      _os << "\n\t    Edge to " << ei->target();
  }

  _os << std::endl;
}

/*------------------------------- Path Finding -------------------------------*/

std::vector<size_t>
WorkspaceDecomposition::
FindPath(const size_t _source, const size_t _target) const {
  if(_source == INVALID_VID or _target == INVALID_VID)
    return {};

  /// @TODO Homogenize our custom SSSP implementations. We have them now at
  ///       least in WorkspaceDecomposition, TopologicalMap, and QueryMethod.

  /// @TODO Remove the const-cast after STAPL fixes its API.
  WorkspaceDecomposition* wd = const_cast<WorkspaceDecomposition*>(this);

  /// An element in the PQ for dijkstra's.
  struct element {

    vertex_descriptor parent; ///< The parent descriptor.
    vertex_descriptor vd;     ///< The vertex descriptor.
    double distance;  ///< Computed distance from souce at the time of insertion.

    /// Total order by decreasing distance.
    bool operator>(const element& _e) const noexcept {
      return distance > _e.distance;
    }

  };

  // Define a min priority queue for dijkstras. We will not update elements when
  // better distances are found - instead we will add them again and ignore all
  // copies after the first (with best distance) is visited.
  std::priority_queue<element,
                      std::vector<element>,
                      std::greater<element>> pq;

  std::unordered_map<vertex_descriptor, bool> visited;
  std::unordered_map<vertex_descriptor, double> distance;
  std::unordered_map<vertex_descriptor, vertex_descriptor> parent;

  // Define the relax function.
  auto relax = [&pq, &distance](adj_edge_iterator& _ei) {
    const vertex_descriptor source = _ei->source(),
                            target = _ei->target();

    const double sourceDistance = distance.count(source)
                                  ? distance[source]
                                  : std::numeric_limits<double>::infinity(),
                 targetDistance = distance.count(target)
                                  ? distance[target]
                                  : std::numeric_limits<double>::infinity(),
                 edgeWeight     = _ei->property().GetWeight(),
                 newDistance    = sourceDistance + edgeWeight;

    // Skip this target if the new distance is worse.
    if(newDistance >= targetDistance)
      return;

    distance[target] = newDistance;
    pq.push(element{source, target, newDistance});
  };

  // Add the root to the queue at distance 0.
  distance[_source] = 0;
  pq.push(element{_source, _source, 0});

  // Search outward through the decomposition graph to locate the neighborhood.
  while(!pq.empty()) {
    // Get the next element
    const element current = pq.top();
    pq.pop();

    // If we are done with this node, discard the element.
    if(visited.count(current.vd))
      continue;
    visited[current.vd] = true;
    parent[current.vd] = current.parent;

    // If this is the target node, we are done.
    if(current.vd == _target)
      break;

    // Relax edges to unvisited adjacent vertices.
    auto vi = wd->find_vertex(current.vd);
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      const size_t target = ei->target();
      if(!visited.count(target))
        relax(ei);
    }
  }

  // If the target has no parent, then it wasn't found.
  if(!parent.count(_target))
    return {};

  // Copy each visited node into the output set.
  std::vector<size_t> output;
  vertex_descriptor current = _target;
  while(current != _source) {
    output.push_back(current);
    current = parent[current];
  }
  output.push_back(_source);
  std::reverse(output.begin(), output.end());

  return output;
}


std::vector<size_t>
WorkspaceDecomposition::
FindPath(const WorkspaceRegion* const _source,
    const WorkspaceRegion* const _target) const {
  return FindPath(GetDescriptor(*_source), GetDescriptor(*_target));
}


std::vector<size_t>
WorkspaceDecomposition::
FindNeighborhood(const std::vector<size_t>& _roots, const double _threshold)
    const {
  /// @TODO Homogenize our custom SSSP implementations. We have them now at
  ///       least in WorkspaceDecomposition, TopologicalMap, and QueryMethod.

  /// @TODO Remove the const-cast after STAPL fixes its API.
  WorkspaceDecomposition* wd = const_cast<WorkspaceDecomposition*>(this);

  /// An element in the PQ for dijkstra's.
  struct element {

    vertex_descriptor vd; ///< The vertex descriptor.
    double distance;  ///< Computed distance from souce at the time of insertion.

    /// Total order by decreasing distance.
    bool operator>(const element& _e) const noexcept {
      return distance > _e.distance;
    }

  };

  // Define a min priority queue for dijkstras. We will not update elements when
  // better distances are found - instead we will add them again and ignore all
  // copies after the first (with best distance) is visited.
  std::priority_queue<element,
                      std::vector<element>,
                      std::greater<element>> pq;

  std::unordered_map<vertex_descriptor, bool> visited;
  std::unordered_map<vertex_descriptor, double> distance;

  // Define the relax function.
  auto relax = [&pq, &distance, &_threshold](adj_edge_iterator& _ei) {
    const vertex_descriptor source = _ei->source(),
                            target = _ei->target();

    const double sourceDistance = distance.count(source)
                                  ? distance[source]
                                  : std::numeric_limits<double>::infinity(),
                 targetDistance = distance.count(target)
                                  ? distance[target]
                                  : std::numeric_limits<double>::infinity(),
                 edgeWeight     = _ei->property().GetWeight(),
                 newDistance    = sourceDistance + edgeWeight;

    // Skip this target if the new distance is worse or exceeds the neighborhood
    // threshold.
    if(newDistance >= std::min(targetDistance, _threshold))
      return;

    distance[target] = newDistance;
    pq.push(element{target, newDistance});
  };

  // Add each root to the queue at distance 0.
  for(const auto& root : _roots) {
    distance[root] = 0;
    pq.push(element{root, 0});
  }

  // Search outward through the decomposition graph to locate the neighborhood.
  while(!pq.empty()) {
    // Get the next element
    const element current = pq.top();
    pq.pop();

    // If we are done with this node, discard the element.
    if(visited.count(current.vd))
      continue;
    visited[current.vd] = true;

    // Relax edges to unvisited adjacent vertices.
    auto vi = wd->find_vertex(current.vd);
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      const size_t target = ei->target();
      if(!visited.count(target))
        relax(ei);
    }
  }

  // Copy each visited node into the output set.
  std::vector<size_t> output;
  output.reserve(visited.size());
  for(auto& pair : visited)
    output.push_back(pair.first);

  return output;
}

/*------------------------------ Helpers -------------------------------------*/

void
WorkspaceDecomposition::
AssertMutable() const {
  if(m_finalized)
    throw PMPLException("WorkspaceDecomposition error", WHERE, "Can't modify "
        "the decomposition after it has been finalized.");
}


void
WorkspaceDecomposition::
ComputeDualGraph() {
  // Make vertices, ensuring that descriptors match.
  for(const_vertex_iterator vit = begin(); vit != end(); ++vit)
    m_dual.add_vertex(vit->descriptor(), vit->property().FindCenter());

  // Make edges.
  for(const_edge_iterator iter = edges_begin(); iter != edges_end(); ++iter) {
    size_t s = iter->source();
    size_t t = iter->target();
    double length = (m_dual.find_vertex(t)->property() -
                     m_dual.find_vertex(s)->property()).norm();
    m_dual.add_edge(s, t, length);
  }
}


void
WorkspaceDecomposition::
UpdateDecompositionPointers() {
  for(auto vi = this->begin(); vi != this->end(); ++vi) {
    vi->property().SetDecomposition(this);
    for(auto ei = vi->begin(); ei != vi->end(); ++ei)
      ei->property().SetDecomposition(this);
  }
}

/*----------------------------------------------------------------------------*/
