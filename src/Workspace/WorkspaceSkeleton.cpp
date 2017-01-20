#include "WorkspaceSkeleton.h"

#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"


/*----------------------------------------------------------------------------*/

WorkspaceSkeleton
WorkspaceSkeleton::
Direct(const Vector3d& _start) {
  GraphType skeletonGraph;

  enum Color {White, Gray, Black};
  unordered_map<FVD, Color> visited;

  //add vertices of reeb graph and find closest
  double closestDist = numeric_limits<double>::max();
  FVD closestID = -1;
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit) {
    /// @TODO Make v a const-ref when stapl fixes sequential graph.
    Vector3d v = vit->property();
    FVD vd = vit->descriptor();
    skeletonGraph.add_vertex(vd, v);
    visited[vd] = White;
    double dist = (v - _start).norm();
    if(dist < closestDist) {
      closestDist = dist;
      closestID = vd;
    }
  }

  //Specialized BFS to make flow network
  //
  //Differs from regular BFS because:
  //  - Treats ReebGraph as undirected graph even though it is directed
  //  - Computes a graph instead of BFS tree, i.e., cross edges are added
  queue<FVD> q;
  q.push(closestID);
  visited[closestID] = Gray;
  while(!q.empty()) {
    FVD u = q.front();
    q.pop();
    auto uit = m_graph.find_vertex(u);

    //process outgoing edges
    for(auto eit = uit->begin(); eit != uit->end(); ++eit) {
      FVD v = eit->target();
      switch(visited[v]) {
        case White:
          visited[v] = Gray;
          q.push(v);
        case Gray:
          skeletonGraph.add_edge(eit->descriptor(), eit->property());
          break;
        default:
          break;
      }
    }

    //process incoming edges
    set<FVD> processed;
    for(auto pit = uit->predecessors().begin();
        pit != uit->predecessors().end(); ++pit) {
      FVD v = *pit;
      if(processed.count(v) == 0) {
        auto vit = m_graph.find_vertex(v);
        for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
          if(eit->target() == u) {
            switch(visited[v]) {
              case White:
                visited[v] = Gray;
                q.push(v);
              case Gray:
                {
                  const vector<Vector3d>& opath = eit->property();
                  vector<Vector3d> path(opath.rbegin(), opath.rend());
                  //TODO shall we change the edge descriptor part?
                  skeletonGraph.add_edge(FED(u, v, eit->descriptor().id()), path);
                  break;
                }
              default:
                break;
            }
          }
        }
      }
      processed.emplace(v);
    }
    visited[u] = Black;
  }

  WorkspaceSkeleton skeleton;
  skeleton.SetGraph(skeletonGraph);
  return skeleton;
}


void
WorkspaceSkeleton::
InitRegions(const Vector3d& _start, const double _regionRadius) {
  // Find the vertex nearest to start.
  auto sit = FindNearestVertex(_start);

  // Spark a region for each outgoing edge from _start.
  for(auto eit = sit->begin(); eit != sit->end(); ++eit) {
    auto r = new WorkspaceBoundingSphere(sit->property(), _regionRadius);
    m_regionData.emplace(r, RegionData{eit->descriptor(), 0, 0});
    m_regions.push_back(r);
  }
  m_visited[sit->descriptor()] = true;
}


void
WorkspaceSkeleton::
CreateRegions(const Vector3d& _p, const double _regionRadius) {
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
      m_regionData.emplace(r, RegionData{eit->descriptor(), 0, 0});
      m_regions.push_back(r);
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

  // Iterate through all regions to see which should be advanced.
  for(auto iter = m_regions.begin(); iter != m_regions.end(); ) {
    Boundary* region = *iter;
    bool increment = true;
    //while the robot still in the region
    while(IsTouching(_cfg, region)) {
      const auto& rc = region->GetCenter();
      Vector3d current(rc[0], rc[1], rc[2]);

      //get detailed info about current region
      //tuple of edge descriptor, vertex index, failed extensions
      auto& regionData = m_regionData[region];
      GraphType::vertex_iterator vi;
      GraphType::adj_edge_iterator ei;

      //get corresponding vertex iter and edge iter of a edge descriptor
      m_graph.find_edge(regionData.edgeDescriptor, vi, ei);
      vector<Vector3d>& path = ei->property();
      size_t& i = regionData.edgeIndex;
      size_t j = i + 1;

      //if after advancing we will still be on the edge
      if(j < path.size()) {
        //get the next region center
        //advance the region
        //change current region center index on the edge
        Vector3d& next = path[j];
        region->ApplyOffset(next - current);
        i = j;
      }
      //else we exceed the edge and need to delete region
      else {
        iter = m_regions.erase(iter);
        m_regionData.erase(region);
        increment = false;
        break;
      }
    }
    if(increment)
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
  const double dist = (regionCenter - robotCenter).norm();

  const double robotRadius = _cfg.GetMultiBody()->GetBoundingSphereRadius();
  const double regionRadius = region->GetRadius();

  // The robot is touching if at least one robot factor of it's bounding sphere
  // penetrates into the region.
  /// @TODO Check this, probably should be maxPenetration = regionRadius - dist.
  const double maxPenetration = robotRadius + regionRadius - dist;
  return maxPenetration > 0 && maxPenetration >= 2 * robotRadius * _robotFactor;
}


WorkspaceSkeleton::vertex_iterator
WorkspaceSkeleton::
FindNearestVertex(const Vector3d& _target) {
  double closestDist = numeric_limits<double>::max();
  vertex_iterator closestVI;

  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit) {
    Vector3d& v = vit->property();
    const double dist = (v - _target).norm();
    if(dist < closestDist) {
      closestDist = dist;
      closestVI = vit;
    }
  }
  return closestVI;
}


void
WorkspaceSkeleton::
PruneFlowGraph(const Cfg& _goal) {
  using VD = GraphType::vertex_descriptor;

  // Find the flow-graph node nearest to the goal.
  // @SOLVED how to get the goal cfg in our current implementation
  // used a new param
  //const Cfg& _goal = this->m_query->GetQuery()[1];
  Vector3d goalPoint(_goal[0], _goal[1], _goal[2]);
  double closestDistance = std::numeric_limits<double>::max();
  // Iterate all and find a vertex in the flow graph that is closest from the
  // actual goal cfg
  VD goal = VD(-1);
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit) {
    const auto& thisPoint = vit->property();
    double distance = (thisPoint - goalPoint).norm();
    if(distance < closestDistance) {
      closestDistance = distance;
      goal = vit->descriptor();
    }
  }

  // Initialize a list of vertices to prune with every vertex in the graph.
  vector<VD> toPrune;
  toPrune.reserve(m_graph.get_num_vertices());
  for(const auto& v : m_graph)
    toPrune.push_back(v.descriptor());

  // Remove vertices from the prune list by starting from the goal and working
  // backwards up the incoming edges. Don't prune any vertex that is an ancestor
  // of the goal.
  queue<VD> q;
  q.push(goal);
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


void
WorkspaceSkeleton::
SetFailedAttempts(const Boundary* const _region, const size_t _num) {
  // Const cast because STL map is stupid about pointer const-ness.
  m_regionData[const_cast<Boundary*>(_region)].failCount = _num;
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
