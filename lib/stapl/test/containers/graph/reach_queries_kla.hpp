/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_RQUERIES_H
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_RQUERIES_H

#include <stapl/containers/graph/algorithms/pscc_kla_utils.hpp>

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/skeletons/map_reduce_sched.hpp>

namespace stapl {

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex property class for the pSCC algorithm.
/// @tparam VertexGIDType The type of the vertex descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertexGIDType>
class reach_queries_vertex_property
{
 public:
  typedef VertexGIDType color_type;
  typedef std::set<color_type> reachables_type;
  typedef typename reachables_type::iterator reachables_iterator;

 private:
  /// @brief The pivot, if any, that reached this node
  size_t m_forward_level;

  /// @brief The pivot, if any, that this node reaches
  size_t m_backward_level;

  /// @brief The reverse edge list of this node.
  reachables_type m_predecessors;

 public:
  reach_queries_vertex_property(void)
    : m_forward_level(std::numeric_limits<size_t>::max()),
      m_backward_level(std::numeric_limits<size_t>::max())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief This function marks the node with a pivot and (if possible)
  /// updates the cc. If the node already has this mark, the function
  /// returns false, so that the traversal knows to stop.
  /// The forward boolean is used to denote whether this mark came from
  /// a forward or a backward traversal.
  //////////////////////////////////////////////////////////////////////
  bool set_level(color_type mark, bool forward, size_t level)
  {
    if (forward && (m_forward_level == std::numeric_limits<size_t>::max())) {
      m_forward_level = level;
      return true;
    }
    if (!forward && (m_backward_level == std::numeric_limits<size_t>::max())) {
      m_backward_level = level;
      return true;
    }
    return false;
  }

  void clear_levels(void)
  {
    m_forward_level = std::numeric_limits<size_t>::max();
    m_backward_level = std::numeric_limits<size_t>::max();
  }

  size_t level(bool forward) const
  {
    if (forward) {
      return m_forward_level;
    } else {
      return m_backward_level;
    }
  }

  void add_predecessor(color_type v)
  { m_predecessors.insert(v); }

  void remove_predecessor(color_type v)
  { m_predecessors.erase(v); }

  void clear_predecessors(void)
  { m_predecessors.clear(); }

  reachables_iterator begin(void)
  { return m_predecessors.begin(); }

  reachables_iterator end(void)
  { return m_predecessors.end(); }

  size_t size(void) const
  { return m_predecessors.size(); }

  void define_type(typer& t)
  {
    t.member(m_forward_level);
    t.member(m_backward_level);
    t.member(m_predecessors);
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref reach_queries_vertex_property.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertGID, typename Accessor>
class proxy<algo_details::reach_queries_vertex_property<VertGID>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef algo_details::reach_queries_vertex_property<VertGID> target_t;
  typedef typename target_t::color_type color_type;
  typedef typename target_t::reachables_type reachables_type;
  typedef typename target_t::reachables_iterator reachables_iterator;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  bool set_level(color_type mark, bool forward, size_t level)
  { return Accessor::invoke(&target_t::set_level, mark, forward, level); }

  void clear_levels(void)
  { Accessor::invoke(&target_t::clear_levels); }

  size_t level(bool forward) const
  { return Accessor::const_invoke(&target_t::level,forward); }

  void add_predecessor(color_type v)
  { Accessor::invoke(&target_t::add_predecessor, v); }

  void remove_predecessor(color_type v)
  { Accessor::invoke(&target_t::add_predecessor, v); }

  void clear_predecessors(void)
  { Accessor::invoke(&target_t::clear_predecessors); }

  reachables_iterator begin(void)
  { return Accessor::invoke(&target_t::begin); }

  reachables_iterator end(void)
  { return Accessor::invoke(&target_t::end); }

  size_t size(void) const
  { return Accessor::const_invoke(&target_t::size); }
}; //struct proxy


namespace algo_details {

struct reach_queries_clear
{
 public:
  typedef void result_type;

  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    v.property().clear_levels();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief This functor updates the simple descriptor of each node.
/// If the node is a pivot, this functor traverses from it.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct reach_queries_bfs_start
{
public:
  typedef bool result_type;

private:
  size_t m_pivot;

public:
  reach_queries_bfs_start(size_t pivot)
    : m_pivot(pivot)
  { }

  template <typename Vertex, typename GraphVisitor>
  result_type operator()(Vertex v, GraphVisitor graph_visitor) const
  {
    typedef typename Vertex::vertex_descriptor color_type;
    typedef pscc_async_bfs<color_type> bfs;
    typedef typename Vertex::property_type property_type;
    typedef typename property_type::reachables_iterator pred_iterator;

    size_t level = graph_visitor.level();
    if (level == 1) {
      //some marks are too high to be used
      if (v.descriptor() != m_pivot) {
        return false;
      } else {
        bfs fw_bfs(m_pivot, true, level);
        fw_bfs(v);
        bfs bw_bfs(m_pivot, false, level);
        bw_bfs(v);

        for (auto&& i : v) {
          graph_visitor.visit(i.target(), fw_bfs);
        }
        property_type p = v.property();
        for (auto&& i : p) {
          graph_visitor.visit(i, bw_bfs);
        }
        return true;
      }
    } else {
      bool result = 0;
      if (v.property().level(true) == level-1) {
        bfs fw_bfs(m_pivot, true, level);
        for (auto&& i : v) {
          graph_visitor.visit(i.target(), fw_bfs);
        }
        result = true;
      }

      if (v.property().level(false) == level-1) {
        property_type p = v.property();
        bfs bw_bfs(m_pivot, false, level);
        for (auto&& i : p) {
          graph_visitor.visit(i, bw_bfs);
        }
        result = true;
      }

      return result;
    }
  }

  void define_type(typer& t)
  { t.member(m_pivot); }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief This function executes the SCC2 algorithm on the graph @p g.
/// The algorithm works by traversing from all nodes
/// breaking the graph into subpieces, and repeating this operation
/// until every node is allocated to an SCC.
/// @param g The @ref graph_view over the input graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
void reach_queries(GraphView& g, size_t k, float degree_percentage=0.1)
{
  using namespace algo_details;

  typedef typename GraphView::vertex_descriptor           vertex_descriptor;
  typedef reach_queries_vertex_property<vertex_descriptor>   vertex_property;
  typedef graph<DIRECTED, MULTIEDGES, vertex_property>    graph_type;
  typedef graph_view<graph_type>                          gview_type;

  size_t size = g.size();

  kla_params<gview_type> p;
  p.avoid_hubs = true;
  p.degree_threshold = degree_percentage*size;

  graph_type gc(size);
  gview_type gcv(gc);
  copy_graph_struct_w_preds(g, gcv);

  srand48((get_location_id()+1)*time(0));

  size_t log_size = std::log(size);
  for (size_t i=0; i != log_size; ++i) {
    algo_details::reach_queries_bfs_start start(drand48()*size);
    kla_paradigm(start, pscc_async_bfs<vertex_descriptor>(), gcv, k, p);
    map_func(reach_queries_clear(), gcv);
  }
}

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_ALGORITHMS_RQUERIES_H

