#include "WorkspaceSkeleton.h"

#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"


/*----------------------------------------------------------------------------*/

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
InitRegions(const Point3d& _start, const double _regionRadius) {
  // Clear all internal state.
  MarkAllNodesUnvisited();

  // Find the vertex nearest to start.
  auto sit = FindNearestVertex(_start);

  // Spark a region for each outgoing edge from _start.
  for(auto eit = sit->begin(); eit != sit->end(); ++eit) {
    auto r = new WorkspaceBoundingSphere(sit->property(), _regionRadius);
    m_regionData.emplace(r, RegionData{eit->descriptor(), 0, 0, 0, 1, 0.0, 0.0});
    m_regions.push_back(r);
  }
  m_visited[sit->descriptor()] = true;

  if(m_debug)
    std::cout << "WorkspaceSkeleton:: initialized new region with radius "
              << std::setprecision(4) << _regionRadius
              << " at vertex " << sit->descriptor()
              << " nearest to point " << _start << "."
              << std::endl;
}


void
WorkspaceSkeleton::
CreateRegions(const Point3d& _p, const double _regionRadius) {
  // Check each skeleton node to see if a new region should be created.
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit) {
    // Skip skeleton nodes that are already visited.
    if(m_visited[vit->descriptor()])
      continue;

    // Skip skeleton nodes that are too far away.
    const double dist = (vit->property() - _p).norm();
    if(dist >= _regionRadius)
      continue;

    // If we're still here, the region is in range and unvisited.
    // Create a new region for each outgoing edge of this skeleton node.
    for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
      auto r = new WorkspaceBoundingSphere(vit->property(), _regionRadius);
      m_regionData.emplace(r, RegionData{eit->descriptor(), 0, 0, 0, 1, 0.0, 0.0});
      m_regions.push_back(r);

      if(m_debug)
        std::cout << "WorkspaceSkeleton:: created new region with radius "
                  << std::setprecision(4) << _regionRadius
                  << " on edge (" << eit->source() << ", "
                  << eit->target() << ", " << eit->id() << ") "
                  << "nearest to point " << _p << "."
                  << std::endl;
    }
    m_visited[vit->descriptor()] = true;
  }
}


void
WorkspaceSkeleton::
AdvanceRegions(const Cfg& _cfg) {
  // Iterate all regions and see if any regions contain the new cfg
  // If a region contains the new cfg, advance it along the flow edge and
  // until the new region we get from this is at the end of the flow edge or
  // it no longer contains the cfg

  if(m_debug)
    std::cout << "WorkspaceSkeleton:: checking " << m_regions.size()
              << " regions for contact with new configuration."
              << std::endl;

  // Iterate through all regions to see which should be advanced.
  for(auto iter = m_regions.begin(); iter != m_regions.end(); ) {
    Boundary* region = *iter;
    bool increment = true;

    // Advance this region until the robot at _cfg is no longer touching it.
    while(IsTouching(_cfg, region)) {
      // Get region data and the edge path it is traversing.
      auto& regionData = m_regionData[region];
      const vector<Point3d>& path = FindEdge(regionData.edgeDescriptor)->
          property();
      size_t& i = regionData.edgeIndex;

      // If there are more points left on this edge, advance the region and
      // index.
      if(i < path.size() - 1) {
        const Point3d& current = path[i];
        const Point3d& next = path[++i];
        region->ApplyOffset(next - current);

        if(m_debug)
          std::cout << "\tAdvancing region " << region << " from index "
                    << i - 1 << " to " << i << " / " << path.size()
                    << "(" << (next - current).norm() << " units)."
                    << std::endl;
      }
      // If not, the region has finished traversing its edge and must be
      // deleted.
      else {
        if(m_debug)
          std::cout << "\tRegion " << region << " has reached the end of its "
                    << "path, erasing it now. " << m_regions.size() - 1
                    << " regions remain."
                    << std::endl;

        iter = m_regions.erase(iter);
        m_regionData.erase(region);

        delete region;
        region = nullptr;
        increment = false;

        break;
      }
    }
    if(increment && iter != m_regions.end())
      ++iter;
  }
}

/*-------------------------------- Helpers -----------------------------------*/

bool
WorkspaceSkeleton::
IsTouching(const Cfg& _cfg, const Boundary* const _region,
    const double _robotFactor) {
  auto region = static_cast<const WorkspaceBoundingSphere*>(_region);

  // Compute the distance between the robot's reference point and the region
  // center.
  const Point3d& robotCenter = _cfg.GetPoint();
  const auto& rc = region->GetCenter();
  const Point3d regionCenter(rc[0], rc[1], rc[2]);

  const double distance = (regionCenter - robotCenter).norm();

  const double robotRadius = _cfg.GetMultiBody()->GetBoundingSphereRadius();
  const double regionRadius = region->GetRadius();

  // The robot is touching if at least one robot factor of it's bounding sphere
  // penetrates into the region.
  const double maxPenetration = robotRadius + regionRadius - distance;

  if(m_debug)
    std::cout << "\tNew Cfg's center is " << distance << " units from the "
              << "region center."
              << "\n\t  New Cfg is potentially overlapping by up to "
              << std::setprecision(4) << maxPenetration << " units."
              << "\n\t  Robot is "
              << (maxPenetration >= 2 * robotRadius * _robotFactor ? "" : "not")
              << " within 'touching' criterion of "
              << std::setprecision(4) << 2 * robotRadius * _robotFactor
              << " units."
              << std::endl;

  return maxPenetration >= 2 * robotRadius * _robotFactor;
}


void
WorkspaceSkeleton::
Prune(const Point3d& _goal) {
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


//void
//WorkspaceSkeleton::
//FlowToMedialAxis(GraphType& _f) const {
//  // use euclidean as default
//  // @TODO how to use the MAUtility here
//  if(this->m_debug)
//    cout << "Flow graph has " << _f.get_num_vertices() << " vertices "
//         << " and " << _f.get_num_edges() << " edges."
//         << "\n\tPushing to medial axis:";
//
//  MedialAxisUtility<MPTraits> mau("pqp_solid", "euclidean",
//      true, true, 10, 10, true, true);
//  auto boundary = this->GetEnvironment()->GetBoundary();
//
//  // Define the push function
//  auto push = [&](Point3d& _p) {
//    Cfg cfg(_p);
//
//    // @TODO how do we get the boundary
//    // If success we use the new point instead
//    if(mau.PushToMedialAxis(cfg, boundary)) {
//      _p = cfg.GetPoint();
//    }
//    // Else we failed to push current vertex to MA
//  };
//
//  // Push flow graph vertices.
//  for(auto vit = _f.begin(); vit != _f.end(); ++vit)
//    push(vit->property());
//
//  // Push flowgraph edges.
//  for(auto eit = _f.edges_begin(); eit != _f.edges_end(); ++eit)
//    for(auto pit = eit->property().begin(); pit < eit->property().end(); ++pit)
//      push(*pit);
//
//  if(this->m_debug)
//    cout << "\n\tMedial axis push complete." << endl;
//}


void
WorkspaceSkeleton::
MarkAllNodesUnvisited() {
  m_visited.clear();
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit)
    m_visited[vit->descriptor()] = false;
}


/*--------------------------- Setters & Getters ------------------------------*/

void
WorkspaceSkeleton::
SetGraph(GraphType& _graph) {
  this->m_graph = _graph;
}


const std::vector<Boundary*>&
WorkspaceSkeleton::
GetRegions() const {
  return m_regions;
}


void
WorkspaceSkeleton::
ComputeProbability() {
  // Sum all weights of all current regions.
  double totalWeight = 0.;
  for(auto& info : m_regionData) {
    auto& regionInfo = info.second;

    // Compute weight of this region.
    regionInfo.weight = regionInfo.successSamples /
        static_cast<double>(regionInfo.totalSamples);

    // sum all weights
    totalWeight += regionInfo.weight;
  }

  for(auto& info : m_regionData) {
    auto& regionInfo = info.second;

    // Compute the probability for this region.
    regionInfo.probability = (1 - m_gamma) * (regionInfo.weight / totalWeight)
                           + (m_gamma / (m_regionData.size() + 1));
  }
}


Boundary*
WorkspaceSkeleton::
SelectRegion() {
  // Region weighting scheme.
  // Select a region based on its ratio of successful samples to total samples.
  // The entire environment is a region added to m_currentRegions.
  // Equation or probability of region i:
  // p_i = (1 - m_gamma)(weight_i / (sum of all weights)) + (m_gamma(1/(K + 1)))

  // Update all region probabilities.
  ComputeProbability();

  // Get the probabilities for the current regions.
  vector<double> probabilities;
  for(const auto& it : m_regionData)
    probabilities.push_back(it.second.probability);

  // Get the probability for the whole environment.
  probabilities.push_back(m_gamma / (m_regionData.size() + 1));

  // Construct with random number generator with the region probabilities.
  static std::default_random_engine generator(0);
  std::discrete_distribution<size_t> distribution(probabilities.begin(),
      probabilities.end());

  const size_t index = distribution(generator);
  const bool envSelected = index == m_regionData.size();

  if(m_debug) {
    std::cout << "WorkspaceSkeleton:: updated region selection probabilities ("
              << "last is whole env):\n\t";

    for(auto p : distribution.probabilities())
      std::cout << std::setprecision(4) << p << " ";

    std::cout << "\n\tSelected index " << index
              << (envSelected ? " (whole env)." : " ");
    if(envSelected)
      std::cout << std::endl;
  }

  if(envSelected)
    return nullptr;

  // Get the selected region.
  auto it = m_regionData.begin();
  advance(it, index);

  if(m_debug) {
    auto c = it->first->GetCenter();
    std::cout << "(region " << it->first << " with center at "
              << Vector3d(c[0], c[1], c[2]) << ", success rate so far "
              << it->second.successSamples << " / " << it->second.totalSamples
              << ")." << std::endl;
  }

  // total samples increment
  ++it->second.totalSamples;

  return it->first;
}

void
WorkspaceSkeleton::
IncrementSuccess(Boundary* _region) {
  auto& regionInfo = GetRegionData(_region);

  // Increment number of successful samples.
  ++regionInfo.successSamples;
}

/*----------------------------------------------------------------------------*/

/// @TODO Revisit visual debugging later.

// Creating the model storage:
//#ifdef VIZMO
//  // Make temporary models for the regions.
//  map<Boundary*, Model*> models;
//  TempObjsModel tom;
//#endif

// Initializing a new model:
//#ifdef VIZMO
//    models[m_regions.back()] = new ThreadSafeSphereModel(
//        m_regions.back()->GetCenter(), regionRadius);
//    tom.AddModel(models[m_regions.back()]);
//#endif
