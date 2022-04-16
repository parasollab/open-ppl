#ifndef PPL_GROUP_ROADMAP_H_
#define PPL_GROUP_ROADMAP_H_

#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/CompositeGraph.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include <containers/sequential/graph/algorithms/graph_input_output.h>


////////////////////////////////////////////////////////////////////////////////
/// Represents a roadmap for a robot group.
///
/// Rather than duplicating the data for each robot, this object refers to an
/// individual roadmap for each robot. This object however does not own the
/// individual roadmaps - it merely points to them.
///
/// Note that VIDs in this object refer to GROUP configuration VIDs.
////////////////////////////////////////////////////////////////////////////////
template <typename Vertex, typename Edge>
class GroupRoadmap final : public CompositeGraph<Vertex, Edge> {

  public:

    ///@name Local Types
    ///@{

    typedef CompositeGraph<Vertex, Edge>         BaseType;
    typedef GroupRoadmap<Vertex, Edge>           GroupRoadmapType;

    typedef typename BaseType::EID               ED;
    typedef typename Vertex::IndividualGraph     IndividualRoadmap;
    typedef typename IndividualRoadmap::CfgType  IndividualCfg;
    typedef typename IndividualRoadmap::EdgeType IndividualEdge;

    typedef typename BaseType::adj_edge_iterator adj_edge_iterator;
    typedef typename BaseType::edge_descriptor   edge_descriptor;
    typedef typename BaseType::vertex_iterator   vertex_iterator;
    typedef typename BaseType::vertex_descriptor vertex_descriptor;

    using typename BaseType::CVI;
    using typename BaseType::VI;
    using typename BaseType::EI;
    using typename BaseType::VID;
    using typename BaseType::STAPLGraph;

    using typename BaseType::VertexHook;
    using typename BaseType::EdgeHook;
    using typename BaseType::HookType;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a group roadmap.
    template <typename MPSolution>
    GroupRoadmap(RobotGroup* const _g, MPSolution* const _solution);

    ///@}
    ///@name Disabled Functions
    ///@{
    /// Move and copy are disabled because the group cfgs and edges need to know
    /// their group roadmap pointer. To implement these, we'll need to update
    /// the pointer on every component.

    GroupRoadmap(const GroupRoadmap&) = delete;
    GroupRoadmap(GroupRoadmap&&)      = delete;

    GroupRoadmap& operator=(const GroupRoadmap&) = delete;
    GroupRoadmap& operator=(GroupRoadmap&&)      = delete;

    ///@}
    ///@name Input/Output
    ///@{

    std::string PrettyPrint() const;

    ///@}
    ///@name Modifiers
    ///@{

    /// Add a new unique vertex to the graph. If it already exists, a warning
    /// will be printed to cerr.
    /// @param _v The vertex to add.
    /// @return A new VID of the added vertex, or the VID of the existing vertex.
    virtual VID AddVertex(const Vertex& _v) noexcept override;

    /// Remove a vertex (and attached edges) from the graph if it exists.
    /// @param _v The vertex descriptor.
    virtual void DeleteVertex(const VID _v) noexcept override;

    /// Add an edge from source to target.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    /// @param _w  The edge property.
    virtual void AddEdge(const VID _source, const VID _target, const Edge& _w)
        noexcept override;

    /// Import the base-class version for this (required because we've
    /// overridden one of the overloads).
    using CompositeGraph<Vertex, Edge>::AddEdge;

    /// Remove an edge from the graph if it exists.
    /// @param _source The source vertex.
    /// @param _source The target vertex.
    virtual void DeleteEdge(const VID _source, const VID _target) noexcept override;

    /// Remove an edge from the graph if it exists.
    /// @param _iterator An iterator to the edge.
    virtual void DeleteEdge(EI _iterator) noexcept override;

    ///@}
    ///@name Hooks
    ///@{

    // /// Uninstall all hooks from each individual roadmap. Should only be used at
    // /// the end of a library run to clean the roadmap object.
    // virtual void ClearHooks() noexcept override;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename Vertex, typename Edge>
template <typename MPSolution>
GroupRoadmap<Vertex, Edge>::
GroupRoadmap(RobotGroup* const _g, MPSolution* const _solution) :
  CompositeGraph<Vertex, Edge>(_g) {

  std::vector<IndividualRoadmap*> roadmaps;
  for(Robot* const robot : *_g)
    roadmaps.push_back(_solution->GetRoadmap(robot));
  
  this->m_roadmaps = roadmaps;
}

/*-------------------------------Input/Output---------------------------------*/

template <typename Vertex, typename Edge>
std::string
GroupRoadmap<Vertex, Edge>::
PrettyPrint() const {
  std::ostringstream out;
  out << "Number of group vertices: " << this->get_num_vertices() << std::endl;
  out << "Vertices in each individual roadmap:" << std::endl << "| ";
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    out << "(Robot " << i << ") " << this->GetRoadmap(i)->get_num_vertices() << " | ";
  }
  out << std::endl;

  return out.str();
}

/*-------------------------------- Modifiers ---------------------------------*/

template <typename Vertex, typename Edge>
void
GroupRoadmap<Vertex, Edge>::
AddEdge(const VID _source, const VID _target, const Edge& _lp) noexcept {
  if(_source == _target)
    throw RunTimeException(WHERE) << "Tried to add edge between same VID "
                                  << _source << ".";

  if(_lp.GetWeight() == 0 or _lp.GetWeight() == 0)
    throw RunTimeException(WHERE) << "Tried to add zero weight edge!";

  // We need to adjust _lp, but we still want to override the base class
  // function, so make a local copy of the edge.
  Edge edge = _lp;

  // Vector of local edges, which are NOT already in individual roadmaps.
  std::vector<IndividualEdge>& localEdges = edge.GetLocalEdges();

  // Vector of edge descriptors, which are edges already in individual roadmaps
  std::vector<ED>& edgeDescriptors = edge.GetEdgeDescriptors();

  const Vertex& sourceCfg = this->GetVertex(_source),
              & targetCfg = this->GetVertex(_target);

  size_t numInactiveRobots = 0;

  // First, make sure all the local edges are in the individual roadmaps.
  for(size_t i = 0; i < edgeDescriptors.size(); ++i) {
    auto roadmap = this->m_roadmaps[i];

    const VID individualSourceVID = sourceCfg.GetVID(i),
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
    else {
      if(edgeExists)
        std::cerr << "\nGroupRoadmap::AddEdge: robot " << i
                  << "'s individual edge (" << individualSourceVID << ", "
                  << individualTargetVID << ") already exists, "
                  << "not adding to its roadmap." << std::endl;

      // NOTE: If you are getting a seg fault here, it's most likely due to not
      //       calling GroupLPOutput::SetIndividualEdges() before calling this!
      roadmap->AddEdge(individualSourceVID, individualTargetVID, localEdges[i]);
      edgeDescriptors[i] = ED(individualSourceVID, individualTargetVID);
    }
  }

  if(numInactiveRobots >= this->GetNumRobots())
    throw RunTimeException(WHERE) << "No robots were moved in this edge!";

  // Now all of the individual edges are in the local maps. Clear out the local
  // copies in edge before we add it to the group map.
  edge.ClearLocalEdges();
  const auto edgeDescriptor = this->add_edge(_source, _target, edge);
  const bool notNew = edgeDescriptor.id() == INVALID_EID;

  if(notNew) {
    std::cerr << "\nGroupRoadmap::AddEdge: edge (" << _source << ", "
              << _target << ") already exists, not adding."
              << std::endl;
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
}


template <typename Vertex, typename Edge>
typename GroupRoadmap<Vertex, Edge>::VID
GroupRoadmap<Vertex, Edge>::
AddVertex(const Vertex& _v) noexcept {
  Vertex cfg; // Will be a copy of the const Vertex
  // Check that the group map is correct, if not, try and change it.
  if(_v.GetGroupRoadmap() != this) {
    std::cerr << "GroupRoadmap::AddVertex: Warning! Group roadmap "
              << "doesn't match this, attempting to exchange the roadmap..."
              << std::endl;
    cfg = _v.SetGroupRoadmap(this);
  }
  else { // Roadmaps match, no change to attempt.
    cfg = _v;
  }

  // Find the vertex and ensure it does not already exist.
  CVI vi;
  if(this->IsVertex(cfg, vi)) {
    std::cerr << "\nGroupRoadmap::AddVertex: vertex " << vi->descriptor()
              << " already in graph"
              << std::endl;
    return vi->descriptor();
  }

  // Add each vid to individual roadmaps if not already present.
  for(size_t i = 0; i < this->m_group->Size(); ++i) {
    auto roadmap = this->GetRoadmap(i);
    auto robot   = roadmap->GetRobot();
    const VID individualVID = cfg.GetVID(i);

    // If the vid is invalid, we must add the local cfg.
    if(individualVID == INVALID_VID)
      cfg.SetRobotCfg(robot, roadmap->AddVertex(cfg.GetRobotCfg(i)));
    // If the vid was valid, make sure it exists.
    else if(roadmap->find_vertex(individualVID) == roadmap->end())
      throw RunTimeException(WHERE) << "Individual vertex " << individualVID
                                    << " does not exist!";
  }

  // The vertex does not exist. Add it now.
  const VID vid = this->add_vertex(cfg);
  this->m_predecessors[vid];
  this->m_allVIDs.insert(vid);
  ++this->m_timestamp;

  // Execute post-add hooks.
  this->ExecuteAddVertexHooks(this->find_vertex(vid));

  return vid;
}


template <typename Vertex, typename Edge>
void
GroupRoadmap<Vertex, Edge>::
DeleteVertex(const VID _v) noexcept {
 // Find the vertex and crash if it doesn't exist.
  VI vi = this->find_vertex(_v);
  if(vi == this->end())
    throw RunTimeException(WHERE, "VID " + std::to_string(_v) +
        " is not in the graph.");

  // We must manually delete the edges rather than letting STAPL do this in
  // order to call the DeleteEdge hooks.

  // Delete the outbound edges.
  for(auto edge = vi->begin(); edge != vi->end(); edge = vi->begin())
    DeleteEdge(edge);

  // Delete the inbound edges.
  /// @TODO This could be done faster with a directed preds graph, but I want to
  ///       profile it both ways to be sure of the costs before doing that. We
  ///       rarely delete vertices, so right now this is an edge case. If we
  ///       start working with methods that delete a lot of vertices, we will
  ///       likely need to switch to the directed preds graph.
  for(VI vertex = this->begin(); vertex != this->end(); ++vertex) {
    while(true) {
      // Jump to the next edge that targets this vertex.
      EI edge = stapl::graph_find(vertex->begin(), vertex->end(),
          stapl::eq_target<VID>(_v));
      if(edge == vertex->end())
        break;

      // If we found one, delete it.
      DeleteEdge(edge);
    }
  }

  // Execute pre-delete hooks and update vizmo debug.
  this->ExecuteDeleteVertexHooks(vi);

  // Delete the group vertex.
  this->delete_vertex(vi->descriptor());
  this->m_allVIDs.erase(_v);
  ++this->m_timestamp;
}

template <typename Vertex, typename Edge>
void
GroupRoadmap<Vertex, Edge>::
DeleteEdge(const VID _source, const VID _target) noexcept {
  // Find the edge and crash if it isn't found.
  const ED edgeDescriptor(_source, _target);
  VI dummy;
  EI edgeIterator;

  const bool found = this->find_edge(edgeDescriptor, dummy, edgeIterator);
  if(!found)
    throw RunTimeException(WHERE) << "Edge (" << _source << ", " << _target
                                  << ") does not exist.";

  DeleteEdge(edgeIterator);
}

template <typename Vertex, typename Edge>
void
GroupRoadmap<Vertex, Edge>::
DeleteEdge(EI _iterator) noexcept {
  // Execute pre-delete hooks and update vizmo debug.
  this->ExecuteDeleteEdgeHooks(_iterator);

  // Delete the individual edges.
  auto& edge = _iterator->property();
  auto& descriptors = edge.GetEdgeDescriptors();
  for(size_t i = 0; i < this->m_group->Size(); ++i)
    this->GetRoadmap(i)->DeleteEdge(descriptors[i].source(), descriptors[i].target());

  // Delete the group edge.
  this->delete_edge(_iterator->descriptor());

  // Remove predessors as appropriate.
  const VID source = _iterator->source(),
            target = _iterator->target();

  this->m_predecessors[target].erase(source);
  ++this->m_timestamp;
}

/*----------------------------------------------------------------------------*/

#endif
