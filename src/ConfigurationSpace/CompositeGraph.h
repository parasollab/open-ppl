#ifndef PPL_COMPOSITE_GRAPH_H_
#define PPL_COMPOSITE_GRAPH_H_

#include "ConfigurationSpace/GenericStateGraph.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include <containers/sequential/graph/algorithms/graph_input_output.h>

#ifndef INVALID_ED
#define INVALID_ED ED{std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max()}
#endif


////////////////////////////////////////////////////////////////////////////////
/// Represents a composite graph for a robot group.
///
/// Rather than duplicating the data for each robot, this object refers to an
/// individual graph for each robot. This object however does not own the
/// individual graphs - it merely points to them.
///
/// Note that VIDs in this object refer to composite state VIDs.
////////////////////////////////////////////////////////////////////////////////
template <typename Vertex, typename Edge>
class CompositeGraph : public GenericStateGraph<Vertex, Edge> {

  public:

    ///@name Local Types
    ///@{

    typedef GenericStateGraph<Vertex, Edge>      BaseType;
    typedef CompositeGraph<Vertex, Edge>         CompositeGraphType;
    typedef typename Vertex::IndividualGraph     IndividualGraph;

    typedef typename BaseType::EID               ED;
    typedef typename IndividualGraph::EdgeType IndividualEdge;

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

    /// Construct a composite graph.
    CompositeGraph(RobotGroup* const _g = nullptr, std::vector<IndividualGraph*> _graphs = {});

    ///@}
    ///@name Disabled Functions
    ///@{
    /// Move and copy are disabled because the composite states and edges need 
    /// to know their group graph pointer. To implement these, we'll need to update
    /// the pointer on every component.

    CompositeGraph(const CompositeGraph&) = delete;
    CompositeGraph(CompositeGraph&&)      = delete;

    CompositeGraph& operator=(const CompositeGraph&) = delete;
    CompositeGraph& operator=(CompositeGraph&&)      = delete;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the robot group.
    virtual RobotGroup* GetGroup();

    /// Get the individual graph for a robot in the group.
    /// @param _index The index of the desired robot.
    virtual IndividualGraph* GetIndividualGraph(const size_t _index);

    /// Get the individual graph for a robot in the group.
    /// @param _index The index of the desired robot.
    virtual const IndividualGraph* GetIndividualGraph(const size_t _index) const;

    /// Get the number of robots for the group this graph is for.
    virtual size_t GetNumRobots() const noexcept;

    ///@}
    ///@name Input/Output
    ///@{

    /// Write the graph using the current standard output rule.
    /// @param _filename The file to write to.
    /// @param _env The environment, to place in the graph.
    virtual void Write(const std::string& _filename, Environment* _env) const
        override;

    /// Write the graph as a Vizmo-compatible composite C-Space path.
    /// @param _filename The file to write to.
    /// @param _env The environment, to place in the graph.
    virtual void WriteCompositeGraph(const std::string& _filename,
                            Environment* const _env) const;

    /// Print the size of this composite graph and the individual graphs for
    /// terminal debugging.
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
    virtual ED AddEdge(const VID _source, const VID _target, const Edge& _w)
        noexcept override;

    /// Import the base-class version for this (required because we've
    /// overridden one of the overloads).
    using BaseType::AddEdge;

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

    /// Uninstall all hooks from each individual graph. Should only be used at
    /// the end of a library run to clean the graph object.
    virtual void ClearHooks() noexcept override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    RobotGroup* const m_group; ///< The robot group.

    std::vector<IndividualGraph*> m_graphs; ///< The individual graphs.

    using BaseType::m_timestamp;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename Vertex, typename Edge>
CompositeGraph<Vertex, Edge>::
CompositeGraph(RobotGroup* const _g, std::vector<IndividualGraph*> _graphs) :
  GenericStateGraph<Vertex, Edge>(nullptr), m_group(_g), m_graphs(_graphs) {

    if (m_graphs.size() != m_group->Size())
      m_graphs.reserve(m_group->Size());
}

/*-------------------------------- Accessors ---------------------------------*/

template <typename Vertex, typename Edge>
inline
RobotGroup*
CompositeGraph<Vertex, Edge>::
GetGroup() {
  return m_group;
}


template <typename Vertex, typename Edge>
inline
typename CompositeGraph<Vertex, Edge>::IndividualGraph*
CompositeGraph<Vertex, Edge>::
GetIndividualGraph(const size_t _index) {
  return m_graphs[_index];
}


template <typename Vertex, typename Edge>
inline
const typename CompositeGraph<Vertex, Edge>::IndividualGraph*
CompositeGraph<Vertex, Edge>::
GetIndividualGraph(const size_t _index) const {
  return m_graphs[_index];
}


template <typename Vertex, typename Edge>
size_t
CompositeGraph<Vertex, Edge>::
GetNumRobots() const noexcept {
  return m_group->Size();
}

/*-------------------------------Input/Output---------------------------------*/

template <typename Vertex, typename Edge>
void
CompositeGraph<Vertex, Edge>::
Write(const std::string& _filename, Environment* _env) const {
  /// For now, we only support composite C-Space output as vizmo is the only
  /// vizualiser that has support for groups/composite robots.
  WriteCompositeGraph(_filename, _env);
}


template <typename Vertex, typename Edge>
void
CompositeGraph<Vertex, Edge>::
WriteCompositeGraph(const std::string& _filename, Environment* _env) const {
  #ifndef VIZMO_MAP
    throw RunTimeException(WHERE, "Cannot use this method without the vizmo map"
                                  " option enabled in the Makefile!");
  #endif

  std::ofstream ofs(_filename);
  ofs << "#####ENVFILESTART#####" << std::endl
      << _env->GetEnvFileName() << std::endl
      << "#####ENVFILESTOP#####" << std::endl;

  stapl::sequential::write_graph(*this, ofs);
}


template <typename Vertex, typename Edge>
std::string
CompositeGraph<Vertex, Edge>::
PrettyPrint() const {
  std::ostringstream out;
  out << "Number of group vertices: " << this->get_num_vertices() << std::endl;
  out << "Vertices in each individual graph:" << std::endl << "| ";
  for(size_t i = 0; i < GetNumRobots(); ++i) {
    out << "(Robot " << i << ") " << GetIndividualGraph(i)->get_num_vertices() << " | ";
  }
  out << std::endl;

  return out.str();
}

/*-------------------------------- Modifiers ---------------------------------*/

template <typename Vertex, typename Edge>
typename CompositeGraph<Vertex, Edge>::ED
CompositeGraph<Vertex, Edge>::
AddEdge(const VID _source, const VID _target, const Edge& _lp) noexcept {
  if(_source == _target)
    throw RunTimeException(WHERE) << "Tried to add edge between same VID "
                                  << _source << ".";

  if(_lp.GetWeight() == 0 or _lp.GetWeight() == 0)
    throw RunTimeException(WHERE) << "Tried to add zero weight edge!";

  // We need to adjust _lp, but we still want to override the base class
  // function, so make a local copy of the edge.
  Edge edge = _lp;

  // Vector of edge descriptors, which are edges already in individual graphs
  std::vector<ED>& edgeDescriptors = edge.GetEdgeDescriptors();

  const Vertex& sourceCfg = this->GetVertex(_source),
              & targetCfg = this->GetVertex(_target);

  size_t numInactiveRobots = 0;

  // First, make sure all the local edges are in the individual graphs.
  for(size_t i = 0; i < edgeDescriptors.size(); ++i) {
    auto graph = m_graphs[i];

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
        graph->find_vertex(individualSourceVID) != graph->end() and
        graph->find_vertex(individualTargetVID) != graph->end();
    if(!verticesExist)
      throw RunTimeException(WHERE, "Cannot add edge for non-existent vertices.");

    // If we received a valid edge descriptor, it must agree with the individual
    // VIDs, and the individual edge must exist.
    const bool edgeExists = graph->IsEdge(individualSourceVID,
                                            individualTargetVID);
    if(edgeDescriptors[i] != INVALID_ED) {
      if(!edgeExists)
        throw RunTimeException(WHERE) << "Expected edge (" << individualSourceVID
                                      << ", " << individualTargetVID << ") does "
                                      << "not exist for robot "
                                      << graph->GetRobot()->GetLabel() << ".";

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
                                      << graph->GetRobot()->GetLabel() << ".";
  }

  if(numInactiveRobots >= GetNumRobots())
    throw RunTimeException(WHERE) << "No robots were moved in this edge!";

  const auto edgeDescriptor = this->add_edge(_source, _target, edge);
  const bool notNew = edgeDescriptor.id() == INVALID_EID;

  if(notNew) {
    std::cerr << "\nCompositeGraph::AddEdge: edge (" << _source << ", "
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
    ++m_timestamp;
  }

  return edgeDescriptor;
}


template <typename Vertex, typename Edge>
typename CompositeGraph<Vertex, Edge>::VID
CompositeGraph<Vertex, Edge>::
AddVertex(const Vertex& _v) noexcept {
  Vertex cfg; // Will be a copy of the const Vertex
  // Check that the group map is correct.
  if((CompositeGraphType*)_v.GetGroupGraph() != this) {
    throw RunTimeException(WHERE) << "Composite Graph of vertex does not "
                                  << "match this composite graph.";
  }
  else { // Graphs match, no change to attempt.
    cfg = _v;
  }

  // Find the vertex and ensure it does not already exist.
  CVI vi;
  if(this->IsVertex(cfg, vi)) {
    std::cerr << "\nCompositeGraph::AddVertex: vertex " << vi->descriptor()
              << " already in graph"
              << std::endl;
    return vi->descriptor();
  }

  // Add each vid to individual graphs if not already present.
  for(size_t i = 0; i < m_group->Size(); ++i) {
    auto graph = GetIndividualGraph(i);
    auto robot   = graph->GetRobot();
    const VID individualVID = cfg.GetVID(i);

    // If the vid is invalid, we must add the local cfg.
    if(individualVID == INVALID_VID)
      cfg.SetRobotCfg(robot, graph->AddVertex(cfg.GetRobotCfg(i)));
    // If the vid was valid, make sure it exists.
    else if(graph->find_vertex(individualVID) == graph->end())
      throw RunTimeException(WHERE) << "Individual vertex " << individualVID
                                    << " does not exist!";
  }

  // The vertex does not exist. Add it now.
  const VID vid = this->add_vertex(cfg);
  this->m_predecessors[vid];
  this->m_allVIDs.insert(vid);
  ++m_timestamp;

  // Execute post-add hooks.
  this->ExecuteAddVertexHooks(this->find_vertex(vid));

  return vid;
}


template <typename Vertex, typename Edge>
void
CompositeGraph<Vertex, Edge>::
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
  ++m_timestamp;
}

template <typename Vertex, typename Edge>
void
CompositeGraph<Vertex, Edge>::
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
CompositeGraph<Vertex, Edge>::
DeleteEdge(EI _iterator) noexcept {
  // Execute pre-delete hooks and update vizmo debug.
  this->ExecuteDeleteEdgeHooks(_iterator);

  // Delete the individual edges.
  auto& edge = _iterator->property();
  auto& descriptors = edge.GetEdgeDescriptors();
  for(size_t i = 0; i < m_group->Size(); ++i)
    GetIndividualGraph(i)->DeleteEdge(descriptors[i].source(), descriptors[i].target());

  // Delete the group edge.
  this->delete_edge(_iterator->descriptor());

  // Remove predessors as appropriate.
  const VID source = _iterator->source(),
            target = _iterator->target();

  this->m_predecessors[target].erase(source);
  ++m_timestamp;
}

/*----------------------------------- Hooks ----------------------------------*/

template <typename Vertex, typename Edge>
void
CompositeGraph<Vertex, Edge>::
ClearHooks() noexcept {
  BaseType::ClearHooks();

  for(IndividualGraph* const map : m_graphs)
    map->ClearHooks();
}

/*----------------------------------------------------------------------------*/

#endif
