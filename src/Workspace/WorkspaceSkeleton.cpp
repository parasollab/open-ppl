#include "WorkspaceSkeleton.h"

#include <containers/sequential/graph/algorithms/graph_input_output.h>

#include "nonstd/io.h"


/*------------------------------- Construction -------------------------------*/

WorkspaceSkeleton::
WorkspaceSkeleton() : GenericStateGraph(nullptr) { }

/*--------------------------------- Locators ---------------------------------*/

WorkspaceSkeleton::vertex_iterator
WorkspaceSkeleton::
FindNearestVertex(const mathtool::Point3d& _target) {
  double closestDistance = std::numeric_limits<double>::max();
  vertex_iterator closestVI;

  for(auto vit = this->begin(); vit != this->end(); ++vit) {
    const double distance = (vit->property() - _target).norm();
    if(distance < closestDistance) {
      closestDistance = distance;
      closestVI = vit;
    }
  }
  return closestVI;
}


WorkspaceSkeleton::vertex_iterator
WorkspaceSkeleton::
FindVertex(const VD _vertexDescriptor) {
  return this->find_vertex(_vertexDescriptor);
}


WorkspaceSkeleton::adj_edge_iterator
WorkspaceSkeleton::
FindEdge(const ED& _ed) {
  vertex_iterator vit;
  adj_edge_iterator eit;
  if(!this->find_edge(_ed, vit, eit))
    throw RunTimeException(WHERE) << "Requested non-existing edge ("
                                  << _ed.id() << "|" << _ed.source()
                                  << "," << _ed.target() << ")";
  return eit;
}

/*--------------------------------- Modifiers --------------------------------*/

WorkspaceSkeleton
WorkspaceSkeleton::
Direct(const mathtool::Point3d& _start) {
  WorkspaceSkeleton* skeleton = new WorkspaceSkeleton();

  // Define coloring for the breadth-first direction algorithm.
  enum Color {
    White, // The vertex is undiscovered.
    Gray,  // The vertex is discovered but not visited.
    Black  // The vertex is visited.
  };
  std::unordered_map<VD, Color> visited;

  // Copy nodes into the new directed skeleton, preserving the descriptors.
  // Also mark each node as undiscovered.
  for(auto vit = this->begin(); vit != this->end(); ++vit) {
    skeleton->AddVertex(vit->descriptor(), vit->property());
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

  std::queue<VD> q;
  q.push(closest);
  while(!q.empty()) {
    // Get the next node in the queue and visit it.
    VD current = q.front();
    q.pop();
    visited[current] = Black;

    auto currentIter = this->find_vertex(current);

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
          skeleton->AddEdge(eit->descriptor(), eit->property());
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
    const auto inEdges = FindInboundEdges(current);
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
            const std::vector<mathtool::Point3d>& path = pit->property();
            skeleton->AddEdge(ED(current, source),
                std::vector<mathtool::Point3d>(path.rbegin(), path.rend()));
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

  skeleton->m_start = closest;
  return *skeleton;
}


void
WorkspaceSkeleton::
Prune(const mathtool::Point3d& _goal) {
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
  BaseType::VertexSet toPrune = this->m_allVIDs;

  // Remove vertices from the prune list by starting from the goal and working
  // backwards up the incoming edges. Don't prune any vertex that is an ancestor
  // of the goal.
  std::queue<VD> q;
  q.push(iter->descriptor());
  do {
    // back track every vertex in the flow graph
    VD current = q.front();
    q.pop();
    // try to find current vertex in the toPrune list
    // i.e., the list in which all vertices are not currently found to be an
    // ancestor yet
    auto iter = std::find(toPrune.begin(), toPrune.end(), current);
    // if found, we can erase it from the list so that we won't be pruning it
    if(iter != toPrune.end())
      toPrune.erase(iter);
    // if not found, just keep it there

    // the backtrack BFS logic
    // for(auto ancestor : this->find_vertex(current)->predecessors())
    auto predecessors = this->m_predecessors.find(current);
    for(auto ancestor : predecessors->second)
      q.push(ancestor);
  } while(!q.empty());

  // Remove the vertices we aren't keeping.
  for(auto vd : toPrune)
    if(this->find_vertex(vd) != this->end())
      this->DeleteVertex(vd);

  if(m_debug)
    std::cout << "\tPruned " << toPrune.size() << " vertices."
              << std::endl;
}

void
WorkspaceSkeleton::
RefineEdges(double _maxLength){
  std::vector<std::vector<Point3d>> refinedVertices;
  //std::vector<WorkspaceSkeleton::ED> originalEdges;
  //for(auto vi = m_graph.begin(); vi != m_graph.end(); vi++){
    //for(auto ei = vi->begin(); ei != vi->end(); ei++){
    for(auto ei = this->edges_begin(); ei != this->edges_end(); ei++){
      auto source = this->find_vertex(ei->source())->property();
      auto target = this->find_vertex(ei->target())->property();
      //original_edges.emplace_back(source,target);
      //if(target[2] != 0 or source[2] != 0)
      //  continue; //This is a hack because invalid edges are being examined
      const double distance = (source - target).norm();
      if(distance < _maxLength)
        continue;
      //size_t divisions = (size_t)std::ceil(distance/_maxLength);
      std::vector<Point3d> newVertices = {this->find_vertex(ei->source())->property()};
      auto& intermediates = ei->property();
      double currentDistance = 0;
      for(size_t i = 1; i < intermediates.size(); i++){
        double step = (intermediates[i-1] - intermediates[i]).norm();
        currentDistance += step;
        //newVertices.push_back(m_graph.add_vertex(intermediates[i*divisions]));
        if(currentDistance > _maxLength){
          newVertices.push_back(intermediates[i-1]);
          currentDistance = step;
        }
      }
      newVertices.push_back(this->find_vertex(ei->target())->property());
      refinedVertices.push_back(newVertices);
    }
  //}
  for(auto edge : refinedVertices){
    size_t firstVID = FindNearestVertex(edge[0])->descriptor();
    size_t vd1 = firstVID;
    size_t vd2 = firstVID;
    for(size_t i = 1; i < edge.size()-1; i++){
      vd2 = this->AddVertex(edge[i]);
      this->AddEdge(vd1,vd2);
      vd1 = vd2;;
    }
    size_t lastVID = FindNearestVertex(edge.back())->descriptor();
    this->AddEdge(vd2,lastVID);
    this->DeleteEdge(firstVID,lastVID);
  }
}


void
WorkspaceSkeleton::
DoubleEdges() {
  // Edge descriptors and paths to add
  std::vector<ED> eds;
  std::vector<EdgeType> paths;

  for(auto vi = this->begin(); vi != this->end(); ++vi) {
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      // Make a reverse descriptor.
      ED reverseEd = reverse(ei->descriptor());

      // Skip edges that already exist
      if(this->IsEdge(reverseEd.source(), reverseEd.target()))
        continue;

      // Get the path and make a reversed copy.
      auto path = ei->property();
      std::reverse(path.begin(), path.end());

      eds.push_back(reverseEd);
      paths.push_back(path);
    }
  }

  for(size_t i = 0; i < eds.size(); ++i)
    this->AddEdge(eds.at(i), paths.at(i));
}

/*------------------------------------- I/O helpers ---------------------------------*/

void
WorkspaceSkeleton::
Write(const std::string& _file) {
  std::ofstream ofs(_file);

  ofs << this->get_num_vertices() << " " << this->get_num_edges() << std::endl;

  for(auto vit = this->begin(); vit != this->end(); ++vit)
    ofs << vit->descriptor() << " " << vit->property() << std::endl;

  for(auto eit = this->edges_begin(); eit != this->edges_end(); ++eit)	{
    ofs << eit->source() << " " << eit->target() << " ";
    auto prop = eit->property();
    ofs << prop.size() << " ";
    for(auto v: prop)
      ofs << v << " ";
    ofs << std::endl;
  }
  ofs.close();
}

void
WorkspaceSkeleton::
Read(const std::string& _file) {
  std::ifstream ifs(_file);

  size_t nVerts, nEdges;

  ifs >> nVerts >> nEdges;

  for(size_t vit = 0 ; vit != nVerts; ++vit) {
    size_t id;
    Point3d data;
    ifs >> id >> data;
    this->AddVertex(data);
  }

  for(size_t eit = 0; eit != nEdges; ++eit) {
    size_t source, target, propSize;
    std::vector<Point3d> edgeProperty;
    ifs >> source >> target >> propSize;
    for (size_t propit = 0; propit < propSize; ++propit) {
      Point3d prop;
      ifs >> prop;
      edgeProperty.push_back(prop);
    }
    this->AddEdge(source, target, edgeProperty);
    edgeProperty.clear();
  }
}

/*----------------------------------------------------------------------------*/
