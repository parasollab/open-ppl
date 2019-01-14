#include "WorkspaceSkeleton.h"

//#include <containers/sequential/graph/algorithms/graph_input_output.h>

/*--------------------------------- Locators ---------------------------------*/

WorkspaceSkeleton::vertex_iterator
WorkspaceSkeleton::
FindNearestVertex(const Point3d& _target) {
  double closestDistance = numeric_limits<double>::max();
  vertex_iterator closestVI;

  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit) {
    const double distance = (vit->property() - _target).norm();
    if(distance < closestDistance) {
      closestDistance = distance;
      closestVI = vit;
    }
  }
  return closestVI;
}


WorkspaceSkeleton::adj_edge_iterator
WorkspaceSkeleton::
FindEdge(const ED& _edgeDescriptor) {
  vertex_iterator vit;
  adj_edge_iterator eit;
  m_graph.find_edge(_edgeDescriptor, vit, eit);
  return eit;
}


std::vector<WorkspaceSkeleton::adj_edge_iterator>
WorkspaceSkeleton::
FindInboundEdges(const VD& _vertexDescriptor) {
  return FindInboundEdges(m_graph.find_vertex(_vertexDescriptor));
}


std::vector<WorkspaceSkeleton::adj_edge_iterator>
WorkspaceSkeleton::
FindInboundEdges(const vertex_iterator& _vertexIter) {
  std::vector<adj_edge_iterator> inEdges;

  // stapl's directed_preds_graph only tells us the predecessors; it doesn't let
  // us iterate over inbound edges. Because we have multi-edges, finding all
  // inbound edges thus requires that we iterate over all out-bound edges of the
  // predecessors and collect their iterators.
  const std::vector<VD>& predecessors = _vertexIter->predecessors();
  for(const auto pred : predecessors) {
    auto predIter = m_graph.find_vertex(pred);
    for(auto eit = predIter->begin(); eit != predIter->end(); ++eit)
      if(eit->target() == _vertexIter->descriptor())
        inEdges.push_back(eit);
  }

  return inEdges;
}

/*--------------------------------- Modifiers --------------------------------*/

WorkspaceSkeleton
WorkspaceSkeleton::
Direct(const Point3d& _start) {
  GraphType skeletonGraph;

  // Define coloring for the breadth-first direction algorithm.
  enum Color {
    White, // The vertex is undiscovered.
    Gray,  // The vertex is discovered but not visited.
    Black  // The vertex is visited.
  };
  unordered_map<VD, Color> visited;

  // Copy nodes into the new directed skeleton, preserving the descriptors.
  // Also mark each node as undiscovered.
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit) {
    skeletonGraph.add_vertex(vit->descriptor(), vit->property());
    visited[vit->descriptor()] = White;
  }

  // Specialized BFS to make flow network
  //
  // Differs from regular BFS because:
  // - Treats skeleton as undirected graph even though it is directed. We do
  //   this because we may want to re-direct a skeleton from a different vertex.
  // - Computes a graph instead of BFS tree, i.e., cross edges are added
  //
  // Also note that the skeleton is a multi-edge graph, so there may be multiple
  // edges between any two vertices.

  // Find the vertex that is nearest to _start, and copy edges from the original
  // skeleton so that all flow away from it.
  VD closest = FindNearestVertex(_start)->descriptor();

  queue<VD> q;
  q.push(closest);
  while(!q.empty()) {
    // Get the next node in the queue and visit it.
    VD current = q.front();
    q.pop();
    visited[current] = Black;

    auto currentIter = m_graph.find_vertex(current);

    // Copy edges leaving the current vertex, and mark their targets as
    // discovered.
    for(auto eit = currentIter->begin(); eit != currentIter->end(); ++eit) {
      VD target = eit->target();
      switch(visited.at(target)) {
        case White:
          // Node was undiscovered, discover it and continue with the copy.
          visited[target] = Gray;
          q.push(target);
        case Gray:
          // Node was previously discovered but hasn't been visited yet. Copy
          // the path from current to target into the directed skeleton.
          skeletonGraph.add_edge(eit->descriptor(), eit->property());
          break;
        default:
          // Target node was previously visited. We don't copy the edge because
          // the target is already on a shorter directed path from the start to
          // the current vertex.
          break;
      }
    }

    // Copy edges inbound on the current vertex, and mark their sources as
    // discovered.
    const auto inEdges = FindInboundEdges(currentIter);
    for(auto pit : inEdges) {
      VD source = pit->source();
      switch(visited.at(source)) {
        case White:
          // Node was undiscovered, discover it and continue with the copy.
          visited[source] = Gray;
          q.push(source);
        case Gray:
          {
            // Node was previously discovered but hasn't been visited yet.
            // Copy the reversed edge (from source to current) into the
            // directed skeleton.
            const vector<Point3d>& path = pit->property();
            skeletonGraph.add_edge(ED(current, source),
                vector<Point3d>(path.rbegin(), path.rend()));
            break;
          }
        default:
          // Source node was previously visited. We don't copy the edge
          // because the source is already on a shorter directed path to
          // the current vertex.
          break;
      }
    }
  }

  WorkspaceSkeleton skeleton;
  skeleton.SetGraph(skeletonGraph);
  skeleton.m_start = closest;
  return skeleton;
}


void
WorkspaceSkeleton::
Prune(const Point3d& _goal) {
  // This function only makes sense after calling Direct. Ensure that we've set
  // m_start to something valid.
  if(m_start == std::numeric_limits<VD>::max())
    throw RunTimeException(WHERE, "Cannot prune the skeleton without directing "
        "it first.");

  // Find the vertex in the skeleton that is closest to the goal point.
  auto iter = FindNearestVertex(_goal);
  if(iter->descriptor() == m_start)
    throw RunTimeException(WHERE, "Requested pruning of workspace skeleton "
        "where the start and goal points are nearest the same vertex. This "
        "produces an empty skeleton.");

  if(m_debug)
    std::cout << "WorkspaceSkeleton:: pruning for goal point " << _goal
              << " (VID " << iter->descriptor() << ")."
              << std::endl;

  // Initialize a list of vertices to prune with every vertex in the graph.
  vector<VD> toPrune;
  toPrune.reserve(m_graph.get_num_vertices());
  for(const auto& v : m_graph)
    toPrune.push_back(v.descriptor());

  // Remove vertices from the prune list by starting from the goal and working
  // backwards up the incoming edges. Don't prune any vertex that is an ancestor
  // of the goal.
  queue<VD> q;
  q.push(iter->descriptor());
  do {
    // back track every vertex in the flow graph
    VD current = q.front();
    q.pop();
    // try to find current vertex in the toPrune list
    // i.e., the list in which all vertices are not currently found to be an
    // ancestor yet
    auto iter = find(toPrune.begin(), toPrune.end(), current);
    // if found, we can erase it from the list so that we won't be pruning it
    if(iter != toPrune.end())
      toPrune.erase(iter);
    // if not found, just keep it there

    // the backtrack BFS logic
    for(auto ancestor : m_graph.find_vertex(current)->predecessors())
      q.push(ancestor);
  } while(!q.empty());

  // Remove the vertices we aren't keeping.
  for(auto vd : toPrune)
    if(m_graph.find_vertex(vd) != m_graph.end())
      m_graph.delete_vertex(vd);

  if(m_debug)
    std::cout << "\tPruned " << toPrune.size() << " vertices."
              << std::endl;
}

/*--------------------------- Setters & Getters ------------------------------*/

void
WorkspaceSkeleton::
SetGraph(GraphType& _graph) noexcept {
  this->m_graph = _graph;
}


WorkspaceSkeleton::GraphType&
WorkspaceSkeleton::
GetGraph() noexcept {
  return m_graph;
}


const WorkspaceSkeleton::GraphType&
WorkspaceSkeleton::
GetGraph() const noexcept {
  return m_graph;
}

void
WorkspaceSkeleton::
Write(const std::string& _file) {
  // stapl::sequential::write_graph(m_graph, _file.c_str());
  /// @Todo: Replace with stapl version.
  ofstream ff(_file);
  auto& g = m_graph;
  ff << g.get_num_vertices() << " " << g.get_num_edges() << endl;
  for(auto vit = g.begin(); vit != g.end(); ++vit)
    ff << vit->descriptor() << " " << vit->property() << endl;
  for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)	{
    ff << eit->source() << " " << eit->target();
    auto prop = eit->property();
    ff << " " << prop.size();
    for(auto v: prop)
      ff << " " << v;
    ff << endl;
  }
}

/*----------------------------------------------------------------------------*/
