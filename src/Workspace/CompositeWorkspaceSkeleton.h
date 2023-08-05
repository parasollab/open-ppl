#ifndef PPL_COMPOSITE_WORKSPACE_SKELETON_H_
#define PPL_COMPOSITE_WORKSPACE_SKELETON_H_

#include "ConfigurationSpace/CompositeGraph.h"
#include "ConfigurationSpace/CompositeState.h"
#include "ConfigurationSpace/CompositeEdge.h"
#include "Workspace/WorkspaceSkeleton.h"

#include "Vector.h"

#include <queue>
#include <set>
#include <unordered_map>
#include <string>


////////////////////////////////////////////////////////////////////////////////
/// A CompositeWorkspaceSkeleton is an aggregate of workspace skeletons
////////////////////////////////////////////////////////////////////////////////
template <typename Vertex, typename Edge>
class CompositeWorkspaceSkeleton : public CompositeGraph<Vertex, Edge> {

  public:

    ///@name Local Types
    ///@{

    typedef CompositeGraph<Vertex, Edge> BaseType;
    typedef typename BaseType::VID  VD;
    typedef typename BaseType::VI   VI;
    typedef typename BaseType::ED   ED;
    typedef typename BaseType::EI   EI;
    typedef typename BaseType::adj_edge_iterator adj_edge_iterator;
    typedef typename BaseType::vertex_iterator vertex_iterator;

    typedef std::vector<WorkspaceSkeleton*>      WorkspaceSkeletonSet;

    // typedef WorkspaceSkeleton::vertex_descriptor             VD;
    // typedef WorkspaceSkeleton::edge_descriptor               ED;
    // typedef typename WorkspaceSkeleton::vertex_iterator      vertex_iterator;
    // typedef typename WorkspaceSkeleton::vertex_descriptor    vertex_descriptor;
    // typedef typename WorkspaceSkeleton::adj_edge_iterator    adj_edge_iterator;

    // typedef std::vector<vertex_iterator>        vertex_iterator_set;
    typedef std::vector<mathtool::Point3d>      PointSet;
    // typedef std::vector<VD>                     VDSet;

    ///@}
    ///@name Construction
    ///@{

    CompositeWorkspaceSkeleton();

    CompositeWorkspaceSkeleton(RobotGroup* const _g, WorkspaceSkeletonSet _skeletons, 
                                double intermediateFactor = 10.0);

    ///@}
    ///@name Locators
    ///@{

    /// Helper function to find the skeleton vertex that is closest to a given
    /// workspace point for each skeleton.
    /// @param _targets The workspace point for each skeleton.
    /// @return An vector of iterators to the nearest skeleton vertices.
    vertex_iterator FindNearestVertex(const PointSet& _targets);

    ///@}
    ///@name Modifiers
    ///@{

    /// Direct the skeletons so that all edges point outwards from the vertex
    /// nearest to a given starting point.
    /// @param _start Workspace points from which the direction should eminate.
    /// @return A directed composite skeleton with flow pointing outward from _start.
    CompositeWorkspaceSkeleton Direct(const PointSet& _start);

    /// Prune the skeletons by removing vertices and edges that are not reachable
    /// by back-tracking the edges from the vertex nearst to a goal point. Only
    /// makes sense after calling Direct.
    /// @param _goal Workspace goal points.
    void Prune(const PointSet& _goal);

    /// Divide existing edges into sizes of at most _maxLength.
    void RefineEdges(double _maxLength);

    /// Double up the edges so that each one has a duplicate facing the opposite
    /// direction with reversed path and same edge ID. This does not make sense
    /// with Direct or Prune. If you want to use with RefineEdges you should
    /// call that first to ensure the edge counterparts match exactly.
    /// @note This function assumes that all edges currently in the skeleton
    ///       are unique and not reverse duplicates.
    void DoubleEdges();

    // Override this to add intermediates
    virtual ED AddEdge(const VD _source, const VD _target, const Edge& _w)
        noexcept override;

    using BaseType::AddEdge;

    /// Overriding to suppress output
    virtual VD AddVertex(const Vertex& _v) noexcept override;

  ///@}

  private:

    double m_intermediateFactor{10.0};

    
};

/*------------------------------- Construction -------------------------------*/

template <typename Vertex, typename Edge>
CompositeWorkspaceSkeleton<Vertex, Edge>::
CompositeWorkspaceSkeleton() :
  CompositeGraph<Vertex, Edge>() {}

template <typename Vertex, typename Edge>
CompositeWorkspaceSkeleton<Vertex, Edge>::
CompositeWorkspaceSkeleton(RobotGroup* const _g, WorkspaceSkeletonSet _skeletons,
  double intermediateFactor) : CompositeGraph<Vertex, Edge>(_g, _skeletons),
  m_intermediateFactor(intermediateFactor) {}

/*--------------------------------- Locators ---------------------------------*/

template <typename Vertex, typename Edge>
typename CompositeWorkspaceSkeleton<Vertex, Edge>::vertex_iterator
CompositeWorkspaceSkeleton<Vertex, Edge>::
FindNearestVertex(const PointSet& _targets) {
  double closestDistance = std::numeric_limits<double>::max();
  vertex_iterator closestVI;

  for(auto vit = this->begin(); vit != this->end(); ++vit) {
    double distance = 0;
    for(size_t i = 0; i < _targets.size(); ++i)
      distance += (vit->property().GetRobotCfg(i) - _targets[i]).norm();

    if(distance < closestDistance) {
      closestDistance = distance;
      closestVI = vit;
    }
  }
  return closestVI;
}

/*-------------------------------- Modifiers ---------------------------------*/

template <typename Vertex, typename Edge>
CompositeWorkspaceSkeleton<Vertex, Edge>
CompositeWorkspaceSkeleton<Vertex, Edge>::
Direct(const PointSet& _start) {
  std::vector<WorkspaceSkeleton> skeletons;
  for (size_t i = 0; i < _start.size(); ++i) {
    skeletons.push_back(this->m_graphs[i].Direct(_start[i]));
  }

  return new CompositeWorkspaceSkeleton(this->m_group, skeletons);
}


template <typename Vertex, typename Edge>
void
CompositeWorkspaceSkeleton<Vertex, Edge>::
Prune(const PointSet& _goal) {

  for (size_t i = 0; i < _goal.size(); ++i) {
    this->m_graphs[i].Prune(_goal[i]);
  }
}


template <typename Vertex, typename Edge>
void
CompositeWorkspaceSkeleton<Vertex, Edge>::
RefineEdges(double _maxLength) {

  for (auto skeleton : this->m_graphs) {
    skeleton.RefineEdges(_maxLength);
  }
}


template <typename Vertex, typename Edge>
void
CompositeWorkspaceSkeleton<Vertex, Edge>::
DoubleEdges() {

  for (auto skeleton : this->m_graphs) {
    skeleton.DoubleEdges();
  }
}

/*----------------------------------------------------------------------------*/

template <typename Vertex, typename Edge>
typename CompositeWorkspaceSkeleton<Vertex, Edge>::ED
CompositeWorkspaceSkeleton<Vertex, Edge>::
AddEdge(const VD _source, const VD _target, const Edge& _lp) noexcept {
  if(_source == _target)
    throw RunTimeException(WHERE) << "Tried to add edge between same VID "
                                  << _source << ".";

  if(_lp.GetWeight() == 0 or _lp.GetWeight() == 0)
    throw RunTimeException(WHERE) << "Tried to add zero weight edge!";

  // We need to adjust _lp, but we still want to override the base class
  // function, so make a local copy of the edge.
  Edge edge = _lp;

  // Vector of edge descriptors, which are edges already in individual roadmaps
  std::vector<ED>& edgeDescriptors = edge.GetEdgeDescriptors();

  const Vertex& sourceCfg = this->GetVertex(_source),
              & targetCfg = this->GetVertex(_target);

  size_t numInactiveRobots = 0;

  // First, make sure all the local edges are in the individual roadmaps.
  for(size_t i = 0; i < edgeDescriptors.size(); ++i) {
    auto roadmap = this->m_graphs[i];

    const VD individualSourceVID = sourceCfg.GetVID(i),
             individualTargetVID = targetCfg.GetVID(i);

    // If they are the same, it means this is an inactive robot. Record the
    // number of these that occur so that we can ensure SOME robot(s) moved.
    if(individualSourceVID == individualTargetVID) {
      ++numInactiveRobots;
    //  continue;
    }

    // Assert that the individual vertices exist.
    const bool verticesExist =
        roadmap->find_vertex(individualSourceVID) != roadmap->end() and
        roadmap->find_vertex(individualTargetVID) != roadmap->end();
    if(!verticesExist)
      throw RunTimeException(WHERE, "Cannot add edge for non-existent vertices.");

    // If we received a valid edge descriptor, it must agree with the individual
    // VIDs, and the individual edge must exist.
    const bool edgeExists = roadmap->IsEdge(individualSourceVID,
                                            individualTargetVID);
    if(edgeDescriptors[i] != INVALID_ED) {
      if(!edgeExists)
        throw RunTimeException(WHERE) << "Expected edge (" << individualSourceVID
                                      << ", " << individualTargetVID << ") does "
                                      << "not exist for robot "
                                      << roadmap->GetRobot()->GetLabel() << ".";

      const bool consistent = edgeDescriptors[i].source() == individualSourceVID
                          and edgeDescriptors[i].target() == individualTargetVID;
      if(!consistent)
        throw RunTimeException(WHERE) << "Edge descriptors are not consistent. "
                                      << "Fetched from group cfg: ("
                                      << individualSourceVID << ", "
                                      << individualTargetVID << "), fetched from "
                                      << "group edge: ("
                                      << edgeDescriptors[i].source() << ", "
                                      << edgeDescriptors[i].target() << ").";
    }
    // If not, assert that the edge to be added does not already exist and then
    // add it.
    else 
      throw RunTimeException(WHERE) << "Expected edge (" << individualSourceVID
                                      << ", " << individualTargetVID << ") does "
                                      << "not exist for robot "
                                      << roadmap->GetRobot()->GetLabel() << ".";
  }

  if(numInactiveRobots >= this->GetNumRobots())
    throw RunTimeException(WHERE) << "No robots were moved in this edge!";

  // Add Intermediates
  // Find the length of each individual edge and the maximum length
  const auto robots = this->GetGroup()->GetRobots();

  // Intermediate length is multiple of the average robot diameter
  double diam = 0;
  for(auto robot : robots)
    diam += robot->GetMultiBody()->GetBoundingSphereRadius() * 2;
  double intLength = (diam / robots.size()) * m_intermediateFactor;
  double tolerance = 1e-5;

  // Initialize intermediate and used length trackers
  std::vector<double> dists;
  std::vector<size_t> inters; // current start intermediate
  std::vector<double> useds; // distance used between start and current position
  std::vector<std::vector<Point3d>> edges;
  double maxDist = 0;
  for(size_t r = 0; r < robots.size(); r++) {
    auto edge = this->m_graphs[r]->GetEdge(edgeDescriptors[r]);
    edges.push_back(edge);

    double total = 0.0;
    if(edgeDescriptors[r].source() != edgeDescriptors[r].target()) {
      for(size_t i = 0; i < edge.size()-1; i++)
        total += (edge.at(i+1) - edge.at(i)).norm();
    } else {
      edges[r] = {this->m_graphs[r]->GetVertex(edgeDescriptors[r].source())};
    }

    dists.push_back(total);
    if(total > maxDist)
      maxDist = total;

    inters.push_back(0);
    useds.push_back(0);
  }

  // Find the number of composite edge intermediates
  std::vector<Vertex> compInters;
  int numInter = (int)ceil(maxDist/intLength);

  // Make the first intermediate
  auto v = Vertex(this->GetGroup());
  for(size_t r = 0; r < robots.size(); r++) {
    auto inter = inters.at(r);
    auto edge = edges.at(r);

    Point3d disp(0.0, 0.0, 0.0);
    if(edge.size() > 1)
      disp = edge.at(inter+1) - edge.at(inter);

    auto point = edge.at(inter) + (disp * useds.at(r));
    v.SetRobotCfg(r, std::move(point));
  }
  compInters.push_back(v);

  // Set the intermediate values along the edge
  for(int i = 0; i < numInter; ++i){
    auto u = Vertex(this->GetGroup());

    for(size_t r = 0; r < robots.size(); r++) {
      // Get the intermediate distance for this robot
      if(dists.at(r) < tolerance or edges.at(r).size() == 1) {
        // Make sure we don't infinite loop for self edges
        auto point = edges.at(r).at(0);
        u.SetRobotCfg(r, std::move(point));
        continue;
      }

      auto intDist = dists.at(r) / numInter;
      double elapsed = 0.;

      auto inter = inters.at(r);
      auto used = useds.at(r);
      auto edge = edges.at(r);
      while(fabs(elapsed - intDist) > tolerance) {
        auto fullLength = (edge.at(inter+1) - edge.at(inter)).norm();
        used = std::min(fullLength - used, intDist - elapsed);
        elapsed += used;
        if(fabs(elapsed - intDist) > tolerance) {
          inter++;
          used = 0.;
        }
      }
      useds[r] = used;
      inters[r] = inter;

      auto disp = edge.at(inter+1) - edge.at(inter);
      auto point = edge.at(inter) + (disp * used);
      u.SetRobotCfg(r, std::move(point));
    }

    compInters.push_back(u);
  }

  // Set the intermediates
  edge.SetIntermediates(compInters);

  const auto edgeDescriptor = this->add_edge(_source, _target, edge);
  const bool notNew = edgeDescriptor.id() == INVALID_EID;

  if(notNew) {
    // std::cerr << "\nCompositeGraph::AddEdge: edge (" << _source << ", "
    //           << _target << ") already exists, not adding."
    //           << std::endl;
    return edgeDescriptor;
  }
  else {
    // Find the edge iterator.
    VI vi;
    EI ei;
    this->find_edge(edgeDescriptor, vi, ei);

    // Execute post-add hooks.
    this->ExecuteAddEdgeHooks(ei);

    this->m_predecessors[_target].insert(_source);
    ++this->m_timestamp;
  }

  return edgeDescriptor;
}


template <typename Vertex, typename Edge>
typename CompositeWorkspaceSkeleton<Vertex, Edge>::VD
CompositeWorkspaceSkeleton<Vertex, Edge>::
AddVertex(const Vertex& _v) noexcept {
  Vertex cfg; // Will be a copy of the const Vertex
  // Check that the group map is correct.
  if((typename BaseType::CompositeGraphType*)_v.GetGroupGraph() != this) {
    throw RunTimeException(WHERE) << "Composite Graph of vertex does not "
                                  << "match this composite graph.";
  }
  else { // Roadmaps match, no change to attempt.
    cfg = _v;
  }

  // Find the vertex and ensure it does not already exist.
  typename BaseType::CVI vi;
  if(this->IsVertex(cfg, vi)) {
    // std::cerr << "\nCompositeGraph::AddVertex: vertex " << vi->descriptor()
    //           << " already in graph"
    //           << std::endl;
    return vi->descriptor();
  }

  // Add each vid to individual roadmaps if not already present.
  for(size_t i = 0; i < this->m_group->Size(); ++i) {
    auto roadmap = this->GetIndividualGraph(i);
    auto robot   = roadmap->GetRobot();
    const VD individualVID = cfg.GetVID(i);

    // If the vid is invalid, we must add the local cfg.
    if(individualVID == INVALID_VID)
      cfg.SetRobotCfg(robot, roadmap->AddVertex(cfg.GetRobotCfg(i)));
    // If the vid was valid, make sure it exists.
    else if(roadmap->find_vertex(individualVID) == roadmap->end())
      throw RunTimeException(WHERE) << "Individual vertex " << individualVID
                                    << " does not exist!";
  }

  // The vertex does not exist. Add it now.
  const VD vid = this->add_vertex(cfg);
  this->m_predecessors[vid];
  this->m_allVIDs.insert(vid);
  ++this->m_timestamp;

  // Execute post-add hooks.
  this->ExecuteAddVertexHooks(this->find_vertex(vid));

  return vid;
}

/*----------------------------------------------------------------------------*/

#endif
