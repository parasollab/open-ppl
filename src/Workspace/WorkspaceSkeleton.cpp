#include "WorkspaceSkeleton.h"

WorkspaceSkeleton
WorkspaceSkeleton::
Direct(const Vector3d& _start)
{  
  //we assumes the m_graph object already contains the info we need
  //the object of directed workspace skeleton for return
  WorkspaceSkeleton directedWSSkeleton;
  //object of skeleton graph to construct
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
  directedWSSkeleton.SetGraph(skeletonGraph);

  return directedWSSkeleton;
}

void
WorkspaceSkeleton::
InitRegions(const Vector3d& _start, const double _regionRadius)
{
  //Spark a region for each outgoing edge from @param _start
  
  // @SOLVED how do we get the FVD from a vertex3d object (_start)?
  // used the nearest vertex in the flow graph

  // Find all outgoing edges from _start and create regions for each edge
  auto sit = FindNearestVertex(_start);
  for(auto eit = sit->begin(); eit != sit->end(); ++eit) {
    auto i = regions.emplace(
        new BoundingSphere(sit->property(), _regionRadius),
        make_tuple(eit->descriptor(), 0, 0));
    m_regions.push_back(i.first->first);
  }
  m_visited[sit->descriptor()] = true;
}

void
WorkspaceSkeleton::
CreateRegions(const Vector3d& _p, double _regionRadius){
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit) {
    if(m_visited[vit->descriptor()])
      continue;
    const double dist = (vit->property() - _p).norm();
    if(dist < _regionRadius) {
      for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
        auto i = regions.emplace(
            new BoundingSphere(vit->property(), _regionRadius),
            make_tuple(eit->descriptor(), 0, 0));
        m_regions.push_back(i.first->first);
      }
      m_visited[vit->descriptor()] = true;
    }
  }
}

void 
WorkspaceSkeleton::
AdvanceRegions(const Cfg& _cfg){
  // Iterate all regions and see if any regions contain the new cfg
  // If a region contains the new cfg, advance it along the flow edge and 
  // until the new region we get from this is at the end of the flow edge or
  // it no longer contains the cfg
  
  //iterate through all regions to see which should be advanced
  for(auto itr = m_regions.begin(); itr != m_regions.end(); )
  {
    //get the region from all regions sequentially
    RegionPtr region = *itr;
    //the bool flag for if the regions is advanced before reaching edge end
    bool increment = true;
    //while the robot still in the region
    while(IsTouching(_cfg, region)) {
      Vector3d cur = region->GetCenter();
      //get detailed info about current region
      //tuple of edge descriptor, vertex index, failed extensions
      auto& pr = regions[region];
      GraphType::vertex_iterator vi;
      GraphType::adj_edge_iterator ei;
      //get corresponding vertex itr and edge itr of a edge descriptor
      m_graph.find_edge(get<0>(pr), vi, ei);
      vector<Vector3d>& path = ei->property();
      size_t& i = get<1>(pr);
      size_t j = i+1;
      //if after advancing we will still be on the edge
      if(j < path.size()) {
        //get the next region center
        Vector3d& next = path[j];
        //advance the region
        region->ApplyOffset(next-cur);
        //change current region center index on the edge
        i = j;
      }
      //else we exceed the edge and need to delete region
      else {
        itr = m_regions.erase(itr);
        regions.erase(region);
        increment = false;
        break;
      }
    }
    if(increment) ++itr;
  }
}

/*-------------------------------- Helpers -----------------------------------*/

bool
WorkspaceSkeleton::
IsTouching(const Cfg& _cfg, RegionPtr _region, double _robotFactor) {
  auto region = static_cast<BoundingSphere*>(_region);

  const Point3d& robotCenter = _cfg.GetPoint();
  const Point3d& regionCenter = region->GetCenter();

  double robotRadius = _cfg.GetRobot()->GetBoundingSphereRadius();
  double regionRadius = region->GetRadius();

  // distance between the region and the robot
  double dist = (regionCenter - robotCenter).norm();

  // the maximum distance the the robot is inisde the region
  double maxPenetration = robotRadius + regionRadius - dist;

  return maxPenetration > 0 && maxPenetration >= 2 * robotRadius * _robotFactor;
}

WorkspaceSkeleton::vertex_itr
WorkspaceSkeleton::
FindNearestVertex(const Vector3d& _target){
  double closestDist = numeric_limits<double>::max();
  vertex_itr closestVI;
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit)
  {
    Vector3d v = vit->property();
    double dist = (v - _target).norm();
    if(dist < closestDist)
    {
      closestDist = dist;
      closestVI = vit;
    }
  }
  return closestVI;
}

void
WorkspaceSkeleton::
PruneFlowGraph(const Cfg& goalCfg) {
  using VD = GraphType::vertex_descriptor;

  // Find the flow-graph node nearest to the goal.
  // @SOLVED how to get the goal cfg in our current implementation
  // used a new param
  //const Cfg& goalCfg = this->m_query->GetQuery()[1];
  Vector3d goalPoint(goalCfg[0], goalCfg[1], goalCfg[2]);
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
    auto itr = find(toPrune.begin(), toPrune.end(), current);
    // if found, we can erase it from the list so that we won't be pruning it
    if(itr != toPrune.end())
      toPrune.erase(itr);
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
//  // @SOLVED how do we get the m_dmLabel and environment boundary here?
//  // use euclidean as default
//  // @TODO how to use the MAUtility here
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
//}

void
WorkspaceSkeleton::
MarkAllNodesUnvisited(){
  m_visited.clear();
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit){
    m_visited[vit->descriptor()] = false;
  }
}

void
WorkspaceSkeleton::
SetFailedAttempts(RegionPtr _region, size_t _num){
  get<2>(regions[_region]) = _num;
}

/*--------------------------- Setters & Getters ------------------------------*/

void
WorkspaceSkeleton::
SetGraph(GraphType& _graph){
  this->m_graph = _graph;
}

vector<WorkspaceSkeleton::RegionPtr>&
WorkspaceSkeleton::
GetRegions(){
  return m_regions;
}
