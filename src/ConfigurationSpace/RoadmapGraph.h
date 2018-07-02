#ifndef ROADMAP_GRAPH_H_
#define ROADMAP_GRAPH_H_

#include "Utilities/PMPLExceptions.h"
#include "Utilities/RuntimeUtils.h"
#include "Utilities/PMPLExceptions.h"

#ifdef _PARALLEL
#include <containers/graph/dynamic_graph.hpp>
//#include "graph/view/graph_view_property_adaptor.h"
//#include <containers/graph/view/graph_view_property_adaptor.hpp>
#include <containers/sequential/graph/algorithms/connected_components.h>
#else
#include <containers/sequential/graph/graph.h>
#include <containers/sequential/graph/graph_util.h>
#include <containers/sequential/graph/vertex_iterator_adaptor.h>
#include <containers/sequential/graph/algorithms/connected_components.h>
#endif

#ifndef INVALID_VID
#define INVALID_VID (std::numeric_limits<size_t>::max())
#endif

#ifndef INVALID_EID
#define INVALID_EID (std::numeric_limits<size_t>::max())
#endif

#include <functional>
#include <string>
#include <unordered_map>

class Robot;


////////////////////////////////////////////////////////////////////////////////
/// Graph data structure of robot configurations (vertices) connected by local
/// plans (edges).
///
/// We often want to do some extra stuff whenever the roadmap is modified. To
/// support that, this object can install hook functions for each of the four
/// modifying events (add/delete a vertex/edge). Note that there is no
/// particular ordering to the hook execution - this is deliberate because we will
/// create a maintenance nightmare if hooks are allowed to depend on each other.
/// As such, no two hooks should modify the same data. Terrible and
/// unpredictable things will happen if you do this!
///
/// @tparam Vertex The vertex or configuration type.
/// @tparam Edge The edge or local plan type.
////////////////////////////////////////////////////////////////////////////////
template <typename Vertex, typename Edge>
class RoadmapGraph : public
#ifdef _PARALLEL
    stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Vertex, Edge>
#elif defined(VIZMO)
    stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Vertex,
        Edge, stapl::sequential::adj_map_int<stapl::DIRECTED,
        stapl::NONMULTIEDGES, Vertex, Edge> >
#else
    stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Vertex,
        Edge>
#endif
{

  public:

    ///@name Local Types
    ///@{

    using STAPLGraph =
#ifdef _PARALLEL
    stapl::dynamic_graph<stapl::DIRECTED,stapl::NONMULTIEDGES,Vertex, Edge>
#elif defined(VIZMO)
    stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Vertex, Edge,
        stapl::sequential::adj_map_int<stapl::DIRECTED, stapl::NONMULTIEDGES,
        Vertex, Edge>>
#else
    stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Vertex, Edge>
#endif
    ;

    typedef typename STAPLGraph::vertex_descriptor VID;
    typedef typename STAPLGraph::edge_descriptor   EID;
    typedef typename STAPLGraph::vertex_iterator   VI;
    typedef typename STAPLGraph::adj_edge_iterator EI;

#ifndef _PARALLEL
    ///TODO: Remove guard after const issue is fixed in STAPL
    typedef typename STAPLGraph::const_vertex_iterator                 CVI;
    typedef typename STAPLGraph::vertex_property&                      VP;
    typedef typename STAPLGraph::edge_property&                        EP;
    typedef stapl::sequential::vdata_iterator<VI>                      VPI;
    typedef stapl::sequential::const_vdata_iterator<VI>                CVPI;
    typedef stapl::sequential::vector_property_map<STAPLGraph, size_t> ColorMap;
#else
    typedef typename STAPLGraph::vertex_iterator                       CVI;
    typedef typename STAPLGraph::vertex_property                       VP;
    typedef stapl::sequential::vector_property_map<STAPLGraph, size_t> ColorMap;
#endif

    typedef std::function<void(VI)> VertexHook;
    typedef std::function<void(EI)> EdgeHook;
    enum class HookType : char {AddVertex, DeleteVertex, AddEdge, DeleteEdge};

    ///@}
    ///@name Construction
    ///@{

    RoadmapGraph(Robot* const _r);

    ///@}
    ///@name Modifiers
    ///@{

    /// Add a new unique vertex to the graph. If it already exists, a warning
    /// will be printed to cerr.
    /// @param _v The vertex to add.
    /// @return A new VID of the added vertex, or the VID of the existing vertex.
    virtual VID AddVertex(const Vertex& _v) noexcept;

    /// Add a new unique vertex to the graph with a designated descriptor. If it
    /// already exists or the descriptor is already in use, a warning will be
    /// printed to cerr.
    /// @param _vid The desired descriptor.
    /// @param _v The vertex property.
    /// @return A new VID of the added vertex, or the VID of the existing vertex.
    virtual VID AddVertex(const VID _vid, const Vertex& _v) noexcept;

    /// Remove a vertex (and attached edges) from the graph if it exists.
    /// @param _v The vertex descriptor.
    virtual void DeleteVertex(const VID _v) noexcept;

    /// Add an edge from source to target.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    /// @param _w  The edge property.
    virtual void AddEdge(const VID _source, const VID _target, const Edge& _w) noexcept;

    /// Add edges both ways between source and target vertices.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    /// @param _w  The edge properties (source to target first).
    virtual void AddEdge(const VID _source, const VID _target,
        const std::pair<Edge, Edge>& _w) noexcept;

    /// Remove an edge from the graph if it exists.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    virtual void DeleteEdge(const VID _source, const VID _target) noexcept;

    /// Remove an edge from the graph if it exists.
    /// @param _iterator An iterator to the edge.
    virtual void DeleteEdge(EI _iterator) noexcept;

    /// Set the robot pointer on all configurations in the map.
    void SetRobot(Robot* const _r) noexcept;

    ///@}
    ///@name Queries
    ///@{

    /// Check if a vertex is present in the graph.
    /// @param _v  The vertex property to seek.
    /// @return True if the vertex property was found in the graph.
    bool IsVertex(const Vertex& _v) noexcept;

    /// Check if a vertex is present in the graph and retrieve a const iterator
    /// to it if so.
    /// @param _v  The vertex property to seek.
    /// @param _vi A vertex iterator, set to the located vertex or end if not
    ///            found.
    /// @return True if the vertex property was found in the graph.
    bool IsVertex(const Vertex& _v, CVI& _vi) noexcept;

    /// Check if an edge is present between two vertices.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    /// @return True if an edge exists from source to target.
    bool IsEdge(const VID _source, const VID _target) noexcept;

    /// Get the descriptor of a vertex property if it exists in the graph, or
    /// INVALID_VID otherwise.
    template <typename T>
    VID GetVID(const T& _t) noexcept;
    VID GetVID(const VI& _t) noexcept;
    VID GetVID(const Vertex& _t) noexcept;

    /// Get the descriptor of the last vertex added to the graph.
    VID GetLastVID() noexcept;

#ifndef _PARALLEL
    /// Get the number of CCs in the graph.
    /// @return The CC count.
    size_t GetNumCCs() noexcept;
#endif

    /// Each time the roadmap is modified, we update the timestamp.
    size_t GetTimestamp() const noexcept;

    ///@}
    ///@name Accessors
    ///@{

    /// Retrieve a reference to a vertex property by descriptor or iterator.
    template <typename T>
    VP GetVertex(T& _t) noexcept;
    VP GetVertex(VI& _t) noexcept;
    VP GetVertex(VID _t) noexcept;

    /// Retrieve an edge from the graph.
    /// @param _source The source node VID.
    /// @param _target The target node VID.
    /// @param _ei An edge iterator, set to the specified edge if found or end
    ///            otherwise.
    /// @return True if the edge was located.
    bool GetEdge(const VID _source, const VID _target, EI& _ei) noexcept;

    EP GetEdge(const VID _source, const VID _target) noexcept;
    EP GetEdge(const EID _descriptor) noexcept;

    Robot* GetRobot() const noexcept;

    ///@}
    ///@name Hooks
    ///@{
    /// Hooks are arbitrary functions that are attached to roadmap events. I.e.,
    /// whenever a vertex is added, a set of functions should be called (hooks).
    /// There is a set of hooks for each of the four modifying actions
    /// (add/delete a vertex/edge).
    ///
    /// IMPORTANT: Hooks for 'add' events execute immediately after the event,
    ///            while hooks for 'delete' events execute immediately prior.
    ///            This ensures that the iterator and neighbor information are
    ///            valid in both cases.
    ///
    /// IMPORTANT: Dependencies between hooks create data races. To avoid
    ///            problems, any piece of data that is modified by one hook
    ///            should not be read or modified by any other hook.

    /// Check if a hook with a given type and label is installed.
    /// @param _type The hook type.
    /// @param _label The unique label.
    /// @return True if a hook of the specified type and label is present.
    bool IsHook(const HookType, const std::string& _label) const;

    /// Install a vertex hook. It will be called each time a new vertex is
    /// added.
    /// @param _type The hook type (vertex add/delete).
    /// @param _label The unique label.
    /// @param _h The hook function.
    void InstallHook(const HookType _type, const std::string& _label,
        const VertexHook& _h);

    /// Install an edge hook. It will be called each time a new edge is added.
    /// @param _type The hook type (edge add/delete).
    /// @param _label The unique label.
    /// @param _h The hook function.
    void InstallHook(const HookType _type, const std::string& _label,
        const EdgeHook& _h);

    /// Remove a hook.
    /// @param _type The hook type.
    /// @param _label The unique label.
    void RemoveHook(const HookType _type, const std::string& _label);

    /// Disable the hooks. This is useful for making temporary additions and
    /// deletions to the roadmap without triggering the hook functions. Be sure
    /// to re-enable them after, and to only use this in isolated code segments
    /// where you are sure that we won't miss any real nodes.
    void DisableHooks() noexcept;

    /// Enable the hook functions (default).
    void EnableHooks() noexcept;

    /// Uninstall all hooks. Should only be used at the end of a library run to
    /// clean the roadmap object.
    void ClearHooks() noexcept;

    ///@}

#ifdef _PARALLEL
    // Temporarily wrapper for some graph methods
    // Until full migration and change of names in STAPL is completed
    size_t get_num_edges() {return this->num_edges();}
    size_t get_num_vertices() const {return this->num_vertices();}
    size_t get_degree(const VID& _vd) {return this->find_vertex(_vd)->size();}
    size_t get_out_degree(const VID& _vd) {return this->get_degree(_vd);}
#endif

  protected:

    ///@name Base Class Unhiding
    ///@{

    /// Unhide the add and delete vertex and edge function within private
    /// encapsulation to prevent users of the RoadmapGraph from calling those
    /// functions.
    using STAPLGraph::add_vertex;
    using STAPLGraph::add_edge;
    using STAPLGraph::delete_vertex;
    using STAPLGraph::delete_edge;

    ///@}
    ///@name Helpers
    ///@{

    /// Execute the AddVertex hooks.
    /// @param _iterator An iterator to the newly added vertex.
    void ExecuteAddVertexHooks(const VI _iterator) noexcept;

    /// Execute the DeleteVertex hooks.
    /// @param _iterator An iterator to the to-be-deleted vertex.
    void ExecuteDeleteVertexHooks(const VI _iterator) noexcept;

    /// Execute the AddEdge hooks.
    /// @param _iterator An iterator to the newly added edge.
    void ExecuteAddEdgeHooks(const EI _iterator) noexcept;

    /// Execute the DeleteEdge hooks.
    /// @param _iterator An iterator to the to-be-deleted edge.
    void ExecuteDeleteEdgeHooks(const EI _iterator) noexcept;

    /// Helper for printing hook type names.
    /// @param _t A hook type.
    /// @return The string representation of _t.
    std::string ToString(const HookType& _t) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    Robot* m_robot{nullptr};       ///< The robot this roadmap is for.

    size_t m_timestamp{0};    ///< Tracks the number of changes to the graph.

    bool m_enableHooks{true}; ///< Use hook functions?

    std::unordered_map<std::string, VertexHook> m_addVertexHooks;
    std::unordered_map<std::string, VertexHook> m_deleteVertexHooks;

    std::unordered_map<std::string, EdgeHook> m_addEdgeHooks;
    std::unordered_map<std::string, EdgeHook> m_deleteEdgeHooks;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename Vertex, typename Edge>
RoadmapGraph<Vertex, Edge>::
RoadmapGraph(Robot* const _r) : m_robot(_r) { }


/*------------------------------- Modifiers ----------------------------------*/

template <typename Vertex, typename Edge>
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
AddVertex(const Vertex& _v) noexcept {
#ifndef _PARALLEL
  // Find the vertex and ensure it does not already exist.
  CVI vi;
  if(IsVertex(_v, vi)) {
  std::cerr << "\nRoadmapGraph::AddVertex: vertex " << vi->descriptor()
              << " already in graph"
              << std::endl;
    return vi->descriptor();
  }

  // The vertex does not exist. Add it now.
  const VID vid = this->add_vertex(_v);
  ++m_timestamp;

  // Execute post-delete hooks and update vizmo debug.
  ExecuteAddVertexHooks(this->find_vertex(vid));
  VDAddNode(_v);

  return vid;
#else
  return this->add_vertex(_v);
#endif
}


template <typename Vertex, typename Edge>
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
AddVertex(const VID _vid, const Vertex& _v) noexcept {
  // Find the vertex and ensure it does not already exist.
  auto iter = this->find_vertex(_vid);
  const bool exists = iter != this->end();
  if(exists or IsVertex(_v)) {
    std::cerr << "\nRoadmapGraph::AddVertex: already in graph" << std::endl;
    return iter->descriptor();
  }

  // The vertex does not exist. Add it now.
  const VID vid = this->add_vertex(_vid, _v);
  ++m_timestamp;

  // Execute post-delete hooks and update vizmo debug.
  ExecuteAddVertexHooks(this->find_vertex(vid));
  VDAddNode(_v);

  return vid;
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
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
  ExecuteDeleteVertexHooks(vi);
  VDRemoveNode(vi->property());

  // Delete the vertex.
  this->delete_vertex(vi->descriptor());
  ++m_timestamp;
}


template <class Vertex, class Edge>
void
RoadmapGraph<Vertex, Edge>::
AddEdge(const VID _source, const VID _target, const Edge& _w) noexcept {
#ifdef VIZMO
  GetVertex(_source).Lock();
#endif
  // Let the add_edge function handle checking for existance incase we are using
  // a multigraph.
  const auto edgeDescriptor = this->add_edge(_source, _target, _w);
  const bool notNew = edgeDescriptor.id() == INVALID_EID;

  if(notNew) {
    std::cerr << "\nRoadmapGraph::AddEdge: edge (" << _source << ", "
              << _target << ") already exists, not adding."
              << std::endl;
  }
  else {
    // Find the edge iterator.
    VI vi;
    EI ei;
    this->find_edge(edgeDescriptor, vi, ei);

    // Execute post-add hooks and update vizmo debug.
    ExecuteAddEdgeHooks(ei);
    VDAddEdge(vi->property(), GetVertex(_target));

    ++m_timestamp;
  }
#ifdef VIZMO
  GetVertex(_source).UnLock();
#endif
}


template <class Vertex, class Edge>
void
RoadmapGraph<Vertex, Edge>::
AddEdge(const VID _source, const VID _target, const std::pair<Edge, Edge>& _w)
    noexcept {
  AddEdge(_source, _target, _w.first);
  AddEdge(_target, _source, _w.second);
}


template <class Vertex, class Edge>
void
RoadmapGraph<Vertex, Edge>::
DeleteEdge(const VID _source, const VID _target) noexcept {
  // Find the edge and crash if it isn't found.
  const EID edgeDescriptor(_source, _target);
  VI dummy;
  EI edgeIterator;

  const bool found = this->find_edge(edgeDescriptor, dummy, edgeIterator);
  if(!found)
    throw RunTimeException(WHERE, "Edge (" + std::to_string(_source) + ", " +
        std::to_string(_target) + ") does not exist.");

  DeleteEdge(edgeIterator);
}


template <class Vertex, class Edge>
void
RoadmapGraph<Vertex, Edge>::
DeleteEdge(EI _iterator) noexcept {
  const Vertex& sourceCfg = GetVertex(_iterator->source());
#ifdef VIZMO
  sourceCfg.Lock();
#endif
  // Execute pre-delete hooks and update vizmo debug.
  ExecuteDeleteEdgeHooks(_iterator);
  VDRemoveEdge(sourceCfg, GetVertex(_iterator->target()));

  // Delete the edge.
  this->delete_edge(_iterator->descriptor());
  ++m_timestamp;
#ifdef VIZMO
  sourceCfg.UnLock();
#endif
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
SetRobot(Robot* const _r) noexcept {
  m_robot = _r;
  for(VI vi = this->begin(); vi != this->end(); ++vi)
    vi->property().SetRobot(_r);
}

/*-------------------------------- Queries -----------------------------------*/

template <typename Vertex, typename Edge>
bool
RoadmapGraph<Vertex, Edge>::
IsVertex(const Vertex& _v) noexcept {
  CVI vi;
  return IsVertex(_v, vi);
}


template <typename Vertex, typename Edge>
bool
RoadmapGraph<Vertex, Edge>::
IsVertex(const Vertex& _v, CVI& _vi) noexcept {
#ifndef _PARALLEL
  for(CVI vi = this->begin(); vi != this->end(); ++vi){
    if(vi->property() == _v){
      _vi = vi;
      return true;
    }
  }
#else
  std::cerr << "WARNING::STAPL working on fixing problem with const iterators\n";
#endif
  return false;
}


template <typename Vertex, typename Edge>
bool
RoadmapGraph<Vertex, Edge>::
IsEdge(const VID _source, const VID _target) noexcept {
  EI ei;
  return GetEdge(_source, _target, ei);
}


template <typename Vertex, typename Edge>
template<typename T>
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
GetVID(const T& _t) noexcept {
  return *_t;
}


template <typename Vertex, typename Edge>
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
GetVID(const VI& _t) noexcept {
  return _t->descriptor();
}


template <typename Vertex, typename Edge>
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
GetVID(const Vertex& _t) noexcept {
  CVI vi;
  if(IsVertex(_t, vi))
    return vi->descriptor();
  return INVALID_VID;
}


template <typename Vertex, typename Edge>
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
GetLastVID() noexcept {
  if(this->get_num_vertices() == 0)
    return INVALID_VID;

  return (--this->end())->descriptor();
}


#ifndef _PARALLEL
template <typename Vertex, typename Edge>
size_t
RoadmapGraph<Vertex, Edge>::
GetNumCCs() noexcept {
  ColorMap c;
  return get_cc_count(*this, c);
}
#endif


template <typename Vertex, typename Edge>
size_t
RoadmapGraph<Vertex, Edge>::
GetTimestamp() const noexcept {
  return m_timestamp;
}

/*------------------------------- Accessors ----------------------------------*/

template <typename Vertex, typename Edge>
template<typename T>
typename RoadmapGraph<Vertex, Edge>::VP
RoadmapGraph<Vertex, Edge>::
GetVertex(T& _t) noexcept {
  return GetVertex(VID(*_t));
}


template <typename Vertex, typename Edge>
typename RoadmapGraph<Vertex, Edge>::VP
RoadmapGraph<Vertex, Edge>::
GetVertex(VI& _t) noexcept {
  return _t->property();
}


template <typename Vertex, typename Edge>
typename RoadmapGraph<Vertex, Edge>::VP
RoadmapGraph<Vertex, Edge>::
GetVertex(VID _t) noexcept {
  auto iter = this->find_vertex(_t);
  if(iter == this->end())
    throw RunTimeException(WHERE, "Requested node '" + std::to_string(_t) +
        "' which is not in the graph.");
  return iter->property();
}


template <typename Vertex, typename Edge>
bool
RoadmapGraph<Vertex, Edge>::
GetEdge(const VID _source, const VID _target, EI& _ei) noexcept {
#ifndef _PARALLEL
  VI vi;
  return this->find_edge(EID(_source, _target), vi, _ei);
#else
  return false;
#endif
}


template <typename Vertex, typename Edge>
typename RoadmapGraph<Vertex, Edge>::EP
RoadmapGraph<Vertex, Edge>::
GetEdge(const VID _source, const VID _target) noexcept {
  EI ei;
  if(!GetEdge(_source, _target, ei)) {
    std::ostringstream oss;
    oss << "Requested non-existent edge (" << _source << ", " << _target << ").";
    throw RunTimeException(WHERE, oss.str());
  }

  return ei->property();
}


template <typename Vertex, typename Edge>
typename RoadmapGraph<Vertex, Edge>::EP
RoadmapGraph<Vertex, Edge>::
GetEdge(const EID _descriptor) noexcept {
  return GetEdge(_descriptor.source(), _descriptor.target());
}


template <typename Vertex, typename Edge>
Robot*
RoadmapGraph<Vertex, Edge>::
GetRobot() const noexcept {
  return m_robot;
}

/*--------------------------------- Hooks ------------------------------------*/

template <typename Vertex, typename Edge>
bool
RoadmapGraph<Vertex, Edge>::
IsHook(const HookType _type, const std::string& _label) const {
  switch(_type) {
    case HookType::AddVertex:
      return m_addVertexHooks.count(_label);
    case HookType::DeleteVertex:
      return m_deleteVertexHooks.count(_label);
    case HookType::AddEdge:
      return m_addEdgeHooks.count(_label);
    case HookType::DeleteEdge:
      return m_deleteEdgeHooks.count(_label);
    default:
      throw RunTimeException(WHERE, "Unrecognized hook type '" +
          ToString(_type) + "'.");
      return false;
  }
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
InstallHook(const HookType _type, const std::string& _label,
    const VertexHook& _h) {
  // Ensure that the hook does not already exist.
  if(IsHook(_type, _label))
    throw RunTimeException(WHERE, "Hook of type '" + ToString(_type) +
        "' with label '" + _label + "' already exists.");

  switch(_type) {
    case HookType::AddVertex:
      m_addVertexHooks[_label] = _h;
      break;
    case HookType::DeleteVertex:
      m_deleteVertexHooks[_label] = _h;
      break;
    case HookType::AddEdge:
    case HookType::DeleteEdge:
      throw RunTimeException(WHERE, "Edge hook type '" + ToString(_type) +
          "' used with vertex hook function labeled '" + _label + "'.");
    default:
      throw RunTimeException(WHERE, "Unrecognized hook type '" +
          ToString(_type) + "' with label '" + _label + "'.");
  }
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
InstallHook(const HookType _type, const std::string& _label, const EdgeHook& _h) {
  // Ensure that the hook does not already exist.
  if(IsHook(_type, _label))
    throw RunTimeException(WHERE, "Hook of type '" + ToString(_type) +
        "' with label '" + _label + "' already exists.");

  switch(_type) {
    case HookType::AddEdge:
      m_addEdgeHooks[_label] = _h;
      break;
    case HookType::DeleteEdge:
      m_deleteEdgeHooks[_label] = _h;
      break;
    case HookType::AddVertex:
    case HookType::DeleteVertex:
      throw RunTimeException(WHERE, "Vertex hook type '" + ToString(_type) +
          "' used with edge hook function labeled '" + _label + "'.");
    default:
      throw RunTimeException(WHERE, "Unrecognized hook type '" +
          ToString(_type) + "' with label '" + _label + "'.");
  }
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
RemoveHook(const HookType _type, const std::string& _label) {
  if(!IsHook(_type, _label))
    throw RunTimeException(WHERE, "Hook of type '" + ToString(_type) +
        "' with label '" + _label + "' does not exist.");

  switch(_type) {
    case HookType::AddVertex:
      m_addVertexHooks.erase(_label);
      break;
    case HookType::DeleteVertex:
      m_deleteVertexHooks.erase(_label);
      break;
    case HookType::AddEdge:
      m_addEdgeHooks.erase(_label);
      break;
    case HookType::DeleteEdge:
      m_deleteEdgeHooks.erase(_label);
      break;
    default:
      throw RunTimeException(WHERE, "Unrecognized hook type '" +
          ToString(_type) + "' with label '" + _label + "'.");
  }
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
DisableHooks() noexcept {
  m_enableHooks = false;
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
EnableHooks() noexcept {
  m_enableHooks = true;
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
ClearHooks() noexcept {
  m_addVertexHooks.clear();
  m_deleteVertexHooks.clear();
  m_addEdgeHooks.clear();
  m_deleteEdgeHooks.clear();
}

/*-------------------------------- Helpers -----------------------------------*/

template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
ExecuteAddVertexHooks(const VI _iterator) noexcept {
  if(m_enableHooks)
    for(auto& hook : m_addVertexHooks)
      hook.second(_iterator);
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
ExecuteDeleteVertexHooks(const VI _iterator) noexcept {
  if(m_enableHooks)
    for(auto& hook : m_deleteVertexHooks)
      hook.second(_iterator);
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
ExecuteAddEdgeHooks(const EI _iterator) noexcept {
  if(m_enableHooks)
    for(auto& hook : m_addEdgeHooks)
      hook.second(_iterator);
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
ExecuteDeleteEdgeHooks(const EI _iterator) noexcept {
  if(m_enableHooks)
    for(auto& hook : m_deleteEdgeHooks)
      hook.second(_iterator);
}


template <typename Vertex, typename Edge>
std::string
RoadmapGraph<Vertex, Edge>::
ToString(const HookType& _t) const noexcept {
  switch(_t) {
    case HookType::AddVertex:
      return "AddVertex";
    case HookType::DeleteVertex:
      return "DeleteVertex";
    case HookType::AddEdge:
      return "AddEdge";
    case HookType::DeleteEdge:
      return "DeleteEdge";
    default:
      return std::string(1, char(_t));
  }
}


/*----------------------------------------------------------------------------*/

#endif
