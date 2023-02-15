/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PRIORITY_AWARE_H
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PRIORITY_AWARE_H

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
template <typename VertexGIDType,
          typename GIDCompare = less<VertexGIDType> >
class pscc_priority_aware_vertex_property
{
 public:
  typedef VertexGIDType                         color_type;
  typedef std::set<color_type, GIDCompare>      reachables_type;
  typedef typename reachables_type::iterator    reachables_iterator;
  typedef std::pair<color_type, color_type>     mark_signature;


 private:
  /// Stores the node's color, or is invalid if it has not yet been colored.
  color_type      m_cc;

  /// @brief A strict weak ordering of color type
  /// (for deciding which colors overwrite which colors,
  /// if a node is labeled by more than one pivot).
  GIDCompare      m_overwrite;

  /// The pivot, if any, that reached this node
  color_type      m_forward_mark;
  size_t          m_forward_level;

  /// The pivot, if any, that this node reaches
  color_type      m_backward_mark;
  size_t          m_backward_level;

  /// The reverse edge list of this node.
  reachables_type m_predecessors;

 public:
  pscc_priority_aware_vertex_property(void)
    : m_cc(invalid_cc()),
      m_overwrite(GIDCompare()),
      m_forward_mark(invalid_cc()),
      m_forward_level(std::numeric_limits<size_t>::max()),
      m_backward_mark(invalid_cc()),
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
    if (forward) {
      if (m_overwrite(mark, m_forward_mark) ||
           (m_forward_level == std::numeric_limits<size_t>::max())) {
        m_forward_mark = mark;
        m_forward_level = level;
      } else {
        return false;
      }
    } else {
      if (m_overwrite(mark, m_backward_mark) ||
           (m_backward_level == std::numeric_limits<size_t>::max())) {
        m_backward_mark = mark;
        m_backward_level = level;
      } else {
        return false;
      }
    }
    if (m_forward_mark == m_backward_mark) {
      m_cc = m_forward_mark;
    } else {
      m_cc = invalid_cc();
    }
    return true;
  }

  size_t level(bool forward) const
  {
    if (forward) {
      return m_forward_level;
    } else {
      return m_backward_level;
    }
  }

  void recolor(color_type color)
  {
    //colors are the same, but doesn't get put in scc until traversal
    if (color != invalid_cc()) {
      m_cc = m_forward_mark = m_backward_mark = color;
      m_forward_level = m_backward_level = 0;
    } else {
      m_forward_mark = m_backward_mark = color;
      m_forward_level = m_backward_level = std::numeric_limits<size_t>::max();
    }
  }

  color_type get_mark(bool forward) const
  {
    if (forward) {
      return m_forward_mark;
    } else {
      return m_backward_mark;
    }
  }

  void set_cc(const color_type c)
  { m_cc = c; }

  color_type get_cc(void) const
  { return m_cc; }

  bool has_valid_cc(void) const
  { return m_cc != invalid_cc(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns some information about the node that (hopefully) allows
  /// it to be distinguished from its neighbors, so that their edges can be
  /// broken.
  ///
  /// Signature is a pair (color, color), denoting the backward and forward
  /// colors which visited this node on the last traversal.
  //////////////////////////////////////////////////////////////////////
  mark_signature get_signature(void) const
  {return std::make_pair(m_backward_mark, m_forward_mark); }

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

  static color_type invalid_cc(void)
  { return index_bounds<color_type>::invalid(); }

  static mark_signature invalid_signature(void)
  { return std::make_pair(invalid_cc(), invalid_cc()); }

  void define_type(typer& t)
  {
    t.member(m_cc);
    t.member(m_forward_mark);
    t.member(m_forward_level);
    t.member(m_backward_mark);
    t.member(m_backward_level);
    t.member(m_predecessors);
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for
/// @ref pscc_priority_aware_vertex_property.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertGID, typename Accessor>
class proxy<algo_details::pscc_priority_aware_vertex_property<VertGID>,
            Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef algo_details::pscc_priority_aware_vertex_property<VertGID> target_t;
  typedef typename target_t::color_type color_type;
  typedef typename target_t::reachables_type reachables_type;
  typedef typename target_t::reachables_iterator reachables_iterator;
  typedef typename target_t::mark_signature mark_signature;

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

  size_t level(bool forward) const
  { return Accessor::const_invoke(&target_t::level,forward); }

  void recolor(color_type c)
  { return Accessor::invoke(&target_t::recolor, c); }

  size_t get_mark(bool forward) const
  { return Accessor::const_invoke(&target_t::get_mark,forward); }

  void set_cc(color_type c)
  { return Accessor::invoke(&target_t::set_cc, c); }

  color_type get_cc(void) const
  { return Accessor::const_invoke(&target_t::get_cc); }

  bool has_valid_cc(void) const
  { return Accessor::const_invoke(&target_t::has_valid_cc); }

  mark_signature get_signature(void) const
  { return Accessor::const_invoke(&target_t::get_signature); }

  color_type get_group(void) const
  { return Accessor::const_invoke(&target_t::get_group); }

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

  static color_type invalid_cc(void)
  { return index_bounds<color_type>::invalid(); }

  static mark_signature invalid_signature(void)
  { return Accessor::invoke(&target_t::invalid_signature); }
}; //class proxy


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief This functor gives each node a unique random number.
/// It also sets (perhaps incorrectly) some ccs. Nothing should
/// be called between this functor and the traversal.
//////////////////////////////////////////////////////////////////////
class pscc_priority_aware_recolor
{
 private:
  size_t m_shift;

 public:
  typedef void result_type;

  pscc_priority_aware_recolor(size_t shift)
    : m_shift(shift)
  { }

  template<typename VertexType>
  result_type operator()(VertexType v) const
  {
    typedef typename VertexType::vertex_descriptor color_type;

    //if node is done, ignore it
    if (v.property().has_valid_cc()) {
      v.property().recolor(v.property().invalid_cc());
      return;
    }

    //otherwise, draw a random number and recolor it
    long double rnum = drand48();
    if (rnum > 1.0/(v.size()+v.property().size()+1)) {
      v.property().recolor(v.property().invalid_cc());
      return;
    }

    color_type color = rnum*(1<<m_shift);
    color = (color<<m_shift) + v.descriptor();
    v.property().recolor(color);
  }

  void define_type(typer& t)
  {
    t.member(m_shift);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief This functor updates the simple descriptor of each node.
/// If the node is a pivot, this functor traverses from it.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_priority_aware_bfs_start
{
  typedef bool result_type;

  template <typename Vertex, typename GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    typedef typename std::decay<Vertex>::type::vertex_descriptor color_type;
    typedef pscc_async_bfs<color_type> bfs;
    typedef typename std::decay<Vertex>::type::property_type property_type;
    typedef typename property_type::reachables_iterator pred_iterator;

    bool result = false;
    size_t level = graph_visitor.level();

    if (v.property().level(true) == level-1) {
      color_type color = v.property().get_mark(true);
      bfs fw_bfs(color, true, level);
      for (auto&& i : v) {
        graph_visitor.visit(i.target(), fw_bfs, default_info(color));
      }
      result = true;
    }

    if (v.property().level(false) == level-1) {
      property_type p = v.property();
      color_type color = p.get_mark(false);
      bfs bw_bfs(color, false, level);
      for (auto&& i : p) {
        graph_visitor.visit(i, bw_bfs, default_info(color));
      }
      result = true;
    }

    return result;
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// This function executes the SCC2 algorithm on the graph @p g.
/// The algorithm works by traversing from all nodes
/// breaking the graph into subpieces, and repeating this operation
/// until every node is allocated to an SCC.
/// @param g The @ref graph_view over the input graph.
/// @param k The maximum amount of asynchrony for kla.
/// @param degree_percentage The threshold for a vertex's degree after which
/// it will be considered a hub (given as a percentage of total graph size).
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
void pscc_priority_aware(GraphView& g, size_t k, float degree_percentage=0.1)
{
  using namespace algo_details;

  typedef typename GraphView::vertex_descriptor               vertex_descriptor;
  typedef pscc_priority_aware_vertex_property<vertex_descriptor>
                                                              vertex_property;
  typedef graph<DIRECTED, MULTIEDGES, vertex_property>        graph_type;
  typedef graph_view<graph_type>                              gview_type;

  size_t gsz = g.size();

  kla_params<gview_type> p;
  p.avoid_hubs = true;
  p.degree_threshold = degree_percentage*gsz;

  graph_type gc(g.size());
  gview_type gcv(gc);
  copy_graph_struct_w_preds(g, gcv);

  srand48((get_location_id()+1)*time(0));

  while (true) {
    map_func(pscc_trim_forward(),  gcv, make_repeat_view(gcv));
    map_func(pscc_trim_backward(), gcv, make_repeat_view(gcv));

    if (count_free_nodes(gcv) == 0) {
      break;
    }

    map_func(pscc_priority_aware_recolor(std::log2(gcv.size())+2), gcv);

    kla_paradigm(pscc_priority_aware_bfs_start(),
      pscc_async_bfs<vertex_descriptor>(0, true, 0),
      arbitrary_priority_scheduler<>(),
      gcv, k, p);

    map_func(pscc_separate_disjoint_return_uncolored(), gcv,
             make_repeat_view(gcv));
  }
  stapl::map_func(copy_sccs_wf(), gcv, g);
}

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_ALGORITHMS_PRIORITY_AWARE_H

