#ifndef GROUP_ROADMAP_H_
#define GROUP_ROADMAP_H_

#include "ConfigurationSpace/RoadmapGraph.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#ifndef INVALID_ED
#define INVALID_ED ED{std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max()}
#endif


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
class GroupRoadmap final : public RoadmapGraph<Vertex, Edge> {

  public:

    ///@name Local Types
    ///@{

    typedef RoadmapGraph<Vertex, Edge>     BaseType;

    typedef typename BaseType::EID         ED;
    typedef typename Vertex::IndividualCfg IndividualCfg;
    typedef typename Edge::IndividualEdge  IndividualEdge;
    typedef typename Vertex::VIDSet        VIDSet;

    using typename BaseType::CVI;
    using typename BaseType::VI;
    using typename BaseType::EI;
    using typename BaseType::VID;
    using typename BaseType::STAPLGraph;

    typedef RoadmapGraph<IndividualCfg, IndividualEdge> IndividualRoadmap;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a group roadmap.
    /// TODO MPSolution is a templated type I think.
    template <typename MPSolution>
    GroupRoadmap(RobotGroup* const _g, MPSolution* const _solution);

    ///@}
    ///@name Accessors
    ///@{

    /// Get the robot group.
    RobotGroup* GetGroup();

    /// Get the individual roadmap for a robot in the group.
    IndividualRoadmap* GetRoadmap(const size_t _index);

    ///@}
    ///@name Modifiers
    ///@{

    /// Add a new unique vertex to the graph. If it already exists, a warning
    /// will be printed to cerr.
    /// @param _v The vertex to add.
    /// @return A new VID of the added vertex, or the VID of the existing vertex.
    virtual VID AddVertex(const Vertex& _v) noexcept override;

    /// Add a new unique vertex to the graph with a designated descriptor. If it
    /// already exists or the descriptor is already in use, a warning will be
    /// printed to cerr.
    /// @param _vid The desired descriptor.
    /// @param _v The vertex property.
    /// @return A new VID of the added vertex, or the VID of the existing vertex.
    //virtual VID AddVertex(const VID _vid, const Vertex& _v) noexcept override;

    /// Remove a vertex (and attached edges) from the graph if it exists.
    /// @param _v The vertex descriptor.
    virtual void DeleteVertex(const VID _v) noexcept override;

    /// Add an edge from source to target.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    /// @param _w  The edge property.
    virtual void AddEdge(const VID _source, const VID _target, const Edge& _w)
        noexcept override;

    /// Remove an edge from the graph if it exists.
    /// @param _iterator An iterator to the edge.
    virtual void DeleteEdge(EI _iterator) noexcept override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    RobotGroup* const m_group; ///< The robot group.

    std::vector<IndividualRoadmap*> m_roadmaps; ///< The individual roadmaps.

    using BaseType::m_timestamp;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename Vertex, typename Edge>
template <typename MPSolution>
GroupRoadmap<Vertex, Edge>::
GroupRoadmap(RobotGroup* const _g, MPSolution* const _solution) : m_group(_g) {
  for(const Robot* const robot : m_group)
    m_roadmaps.push_back(_solution->GetRoadmap(robot));
}

/*-------------------------------- Accessors ---------------------------------*/

template <typename Vertex, typename Edge>
inline
RobotGroup*
GroupRoadmap<Vertex, Edge>::
GetGroup() {
  return m_group;
}


template <typename Vertex, typename Edge>
inline
typename GroupRoadmap<Vertex, Edge>::IndividualRoadmap*
GroupRoadmap<Vertex, Edge>::
GetRoadmap(const size_t _index) {
  return m_roadmaps[_index];
}

/*-------------------------------- Modifiers ---------------------------------*/

template <typename Vertex, typename Edge>
void
GroupRoadmap<Vertex, Edge>::
AddEdge(const VID _source, const VID _target, const Edge& _lp) noexcept {
  // We need to adjust _lp, but we still want to override the base class
  // function.
  Edge edge = _lp;
  std::vector<IndividualEdge>& localEdges = edge.GetLocalEdges();
  std::vector<ED>& edgeDescriptors = edge.GetEdgeDescriptors();

  const Vertex& sourceCfg = this->GetVertex(_source),
                & targetCfg = this->GetVertex(_target);

  // First, make sure all the local edges are in the individual roadmaps.
  for(size_t i = 0; i < edgeDescriptors.size(); ++i) {
    auto roadmap = m_roadmaps[i];

    const VID individualSourceVID = sourceCfg.GetVID(i),
              individualTargetVID = targetCfg.GetVID(i);

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
      const bool consistent = edgeDescriptors[i].source() == individualSourceVID
                          and edgeDescriptors[i].target() == individualTargetVID;

      if(!consistent)
        throw RunTimeException(WHERE, "Edge descriptors are not consistent!");
      if(!edgeExists)
        throw RunTimeException(WHERE, "Expected edge does not exist!");
    }
    // If not, assert that the edge to be added does not already exist and then
    // add it.
    else {
      if(edgeExists)
        throw RunTimeException(WHERE, "Cannot re-add existing edge.");

      roadmap->AddEdge(individualSourceVID,
                       individualTargetVID,
                       localEdges[i]);
      edgeDescriptors[i] = ED(individualSourceVID, individualTargetVID);
    }
  }

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
}


template <typename Vertex, typename Edge>
typename GroupRoadmap<Vertex, Edge>::VID
GroupRoadmap<Vertex, Edge>::
AddVertex(const Vertex& _v) noexcept {
  Vertex cfg = _v;

  // Find the vertex and ensure it does not already exist.
  CVI vi;
  if(this->IsVertex(cfg, vi)) {
    std::cerr << "\nGroupRoadmap::AddVertex: vertex " << vi->descriptor()
              << " already in graph"
              << std::endl;
    return vi->descriptor();
  }

  // Add each vid to individual roadmaps if not already present.
  for(size_t i = 0; i < m_group->Size(); ++i) {
    auto roadmap = GetRoadmap(i);
    auto robot   = roadmap->GetRobot();
    const VID individualVID = cfg.GetVID(i);

    // If the vid is invalid, we must add the local cfg.
    if(individualVID == INVALID_VID)
      cfg.SetCfg(robot, roadmap->AddVertex(cfg.GetRobotCfg(i)));
    // If the vid was valid, make sure it exists.
    else if(roadmap->find_vertex(individualVID) == roadmap->end())
      throw RunTimeException(WHERE, "Individual vertex " +
          std::to_string(individualVID) + " does not exist!");
  }

  // The vertex does not exist. Add it now.
  const VID vid = this->add_vertex(cfg);
  ++m_timestamp;

  return vid;
}


//template <typename Vertex, typename Edge>
//VID
//GroupRoadmap<Vertex, Edge>::
//AddVertex(const VID _vid, const Vertex& _v) noexcept {
//  // Find the vertex and ensure it does not already exist.
//  auto iter = this->find_vertex(_vid);
//  const bool exists = iter != this->end();
//  if(exists or IsVertex(_v)) {
//    std::cerr << "\nGroupRoadmap::AddVertex: already in graph" << std::endl;
//    return iter->descriptor();
//  }
//
//  // The vertex does not exist. Add it now.
//  const VID vid = this->add_vertex(_vid, _v);
//  ++m_timestamp; // TODO keep?
//
//  // TODO: update vizmo debug.
//  VDAddNode(_v);
//
//  return vid;
//}


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

  // No hooks or VD, unlike RoadmapGraph.

  // Delete the group vertex.
  this->delete_vertex(vi->descriptor());
  ++m_timestamp;
}


template <typename Vertex, typename Edge>
void
GroupRoadmap<Vertex, Edge>::
DeleteEdge(EI _iterator) noexcept {
  // No hooks or VD, unlike RoadmapGraph.

  // Delete the individual edges.
  auto& edge = _iterator->property();
  auto& descriptors = edge.GetEdgeDescriptors();
  for(size_t i = 0; i < m_group->Size(); ++i)
    GetRoadmap(i)->DeleteEdge(descriptors[i].source(), descriptors[i].target());

  // Delete the group edge.
  this->delete_edge(_iterator->descriptor());
  ++m_timestamp;
}

#endif
