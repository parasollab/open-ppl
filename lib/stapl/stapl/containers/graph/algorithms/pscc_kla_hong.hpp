/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_HONG_H
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_HONG_H

#include <stapl/containers/graph/algorithms/pscc_kla_utils.hpp>

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/pscc_kla_single.hpp>
#include <stapl/containers/graph/algorithms/connected_components.hpp>
#include <stapl/map.hpp>

namespace stapl {

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex property class for the DCSC algorithm.
/// @tparam VertexGIDType The type of the vertex descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class pscc_hong_edge_property
{
 private:
  bool m_temp;

 public:
  pscc_hong_edge_property(void)
    : m_temp(false)
  { }

  pscc_hong_edge_property(bool temp)
    : m_temp(temp)
  { }

  bool is_temp(void) const
  { return m_temp; }

  void define_type(typer& t)
  { t.member(m_temp); }
};

} //namespace algo_details


template <typename Accessor>
class proxy<algo_details::pscc_hong_edge_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef algo_details::pscc_hong_edge_property target_t;

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

  bool is_temp(void) const
  { return Accessor::const_invoke(&target_t::is_temp); }
};


namespace algo_details {

struct pscc_add_temp_edges
{
  typedef void result_type;

  template <typename Vertex, typename Graph>
  result_type operator()(Vertex v, Graph& g) const
  {
    if (v.property().size() == 0) {
      return;
    }

    typename Vertex::property_type prop = v.property();
    for (auto&& i : prop) {
      g.add_edge_async(typename Graph::edge_descriptor(v.descriptor(), i),
                       typename Graph::edge_property(true));
    }
  }
};

struct pscc_del_temp_edges
{
  typedef void result_type;

  template <typename Vertex, typename Graph>
  result_type operator()(Vertex v, Graph& g) const
  {
    if (v.property().size() == 0) {
      return;
    }

    auto edges = v.edges();
    for (auto&& i : edges) {
      if (i.property().is_temp()) {
        g.delete_edge(i.descriptor());
      }
    }
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// This function executes Hong's DCSC algorithm on the graph @p g. The
/// algorithm works by selecting a pivot, traversing from this pivot, breaking
/// the graph into subpieces, performing a special trim, separating weakly
/// connected components, and then running DCSC (pscc_single) on the pieces.
/// @param g The @ref graph_view over the input graph.
/// @param k The maximum amount of asynchrony for kla.
/// @param degree_percentage The threshold for a vertex's degree after which
/// it will be considered a hub (given as a percentage of total graph size).
///
/// For DCSC, it is very important the group to which each node belongs be
/// maintained constantly; the algorithm cannot use more than one pivot at
/// a time on any group. We use two different methods of distinguishing the
/// groups, depending on which part of the algorithm we are currently executing.
/// - (simple) While the traversals are being executed, the groups are
/// distinguished only by what pivot is operating on them; this is denoted by
/// the member m_group in pscc_single_vertex_property.
/// - (complex) While the new pivots are being selected, the groups are
/// distinguished by a (pivot that just operated on me, how that pivot can reach
/// me) pair.
/// This information is encapsulated in the @ref pscc_single_mark_type struct.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
void pscc_hong(GraphView& g, size_t k, float degree_percentage=0.1)
{
  using namespace algo_details;

  typedef typename GraphView::vertex_descriptor           vertex_descriptor;
  typedef pscc_single_vertex_property<vertex_descriptor>  vertex_property;
  typedef pscc_hong_edge_property                         edge_property;
  typedef graph<DIRECTED, MULTIEDGES,
                vertex_property, edge_property>           graph_type;
  typedef graph_view<graph_type>                          gview_type;

  typedef typename vertex_property::mark_signature        group_type;
  typedef group_type                                      map_key_type;
  typedef std::pair<vertex_descriptor, size_t>            map_value_type;
  typedef stapl::map<map_key_type, map_value_type>        pivots_map;
  typedef stapl::indexed_domain<map_key_type>             pivots_domain;
  typedef stapl::map_view<pivots_map>                     pivots_view;

  size_t g_sz = g.size();

  kla_params<gview_type> p;
  p.avoid_hubs = true;
  p.degree_threshold = degree_percentage*g_sz;

  graph_type gc(g_sz);
  gview_type gcv(gc);
  copy_graph_struct_w_preds(g, gcv);

  srand48((get_location_id()+1)*time(0));

  pivots_domain dom(group_type(0, 0), group_type(g_sz-1, 2));

  //HONG'S SPECIAL ADDITIONS:
  // TRIM ONCE
  map_func(pscc_trim_forward(), gcv, make_repeat_view(gcv));
  map_func(pscc_trim_backward(), gcv, make_repeat_view(gcv));

  // RUN ONE STEP
  if (count_free_nodes(gcv) != 0) {
    pivots_map pivots(dom);
    pivots_view pv(pivots);
    map_func(pscc_single_pivot_selector(), gcv, make_repeat_view(pv));

    algo_details::pscc_single_bfs_start<pivots_view> start(pv);
    kla_paradigm(start, pscc_async_bfs<vertex_descriptor>(), gcv, k, p);

    map_func(pscc_separate_disjoint_return_uncolored(), gcv,
             make_repeat_view(gcv));
  }

  // TRIM
  map_func(pscc_trim_forward(), gcv, make_repeat_view(gcv));
  map_func(pscc_trim_backward(), gcv, make_repeat_view(gcv));
  // TRIM2
  map_func(pscc_double_trim(), gcv, make_repeat_view(gcv));
  // TRIM
  map_func(pscc_trim_forward(), gcv, make_repeat_view(gcv));
  map_func(pscc_trim_backward(), gcv, make_repeat_view(gcv));


  // WCC
  map_func(pscc_add_temp_edges(), gcv, make_repeat_view(gcv));
  connected_components(gcv);
  map_func(pscc_del_temp_edges(), gcv, make_repeat_view(gcv));

  while (count_free_nodes(gcv) > 0) {
    pivots_map pivots(dom);
    pivots_view pv(pivots);
    map_func(pscc_single_pivot_selector(), gcv, make_repeat_view(pv));

    algo_details::pscc_single_bfs_start<pivots_view> start(pv);
    kla_paradigm(start, pscc_async_bfs<vertex_descriptor>(), gcv, k, p);

    map_func(pscc_separate_disjoint_return_uncolored(), gcv,
             make_repeat_view(gcv));

    map_func(pscc_trim_forward(), gcv, make_repeat_view(gcv));
    map_func(pscc_trim_backward(), gcv, make_repeat_view(gcv));
  }

  stapl::map_func(copy_sccs_wf(), gcv, g);
}

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_ALGORITHMS_HONG_H

