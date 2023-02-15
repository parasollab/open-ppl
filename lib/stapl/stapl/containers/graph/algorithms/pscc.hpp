/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PSCC_H
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PSCC_H

#include <stapl/containers/graph/algorithms/pscc_utils.hpp>

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/views/repeated_view.hpp>


namespace stapl {

namespace algo_details {


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property class for the pSCC algorithm.
/// @tparam VertexGIDType The type of the vertex descriptor.
/// @tparam GIDCompare The type of the comparator used to compare the
/// vertex descriptors. Defaults to less<VertexGIDType>.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertexGIDType,
          typename GIDCompare = less<VertexGIDType> >
class pscc_vertex_property
{
 public:
  typedef VertexGIDType                         color_type;
  typedef std::set<color_type, GIDCompare>      reachables_type;
  typedef typename reachables_type::iterator    reachables_iterator;
  typedef std::pair<size_t, size_t>             mark_signature;

 private:
  /// @brief Stores the node's color, or is invalid if it has not yet been
  /// colored.
  color_type      m_cc;

  /// @brief Indicates whether the node is currently a pivot.
  bool            m_is_pivot;

  /// @brief A strict weak ordering of color type
  /// (for deciding which colors overwrite which colors,
  /// if a node is labeled by more than one pivot).
  GIDCompare      m_overwrite;

  /// @brief The set of pivots that reach this node.
  reachables_type m_forward;

  /// @brief The set of pivots that this node reaches.
  reachables_type m_backward;

  /// @brief The reverse edge list of this node.
  reachables_type m_predecessors;

 public:
  pscc_vertex_property()
    : m_cc(invalid_cc()),
      m_is_pivot(false),
      m_overwrite(GIDCompare())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief This function marks the node with a pivot and (if possible)
  /// updates the cc. If the node already has this mark, the function
  /// returns false, so that the traversal knows to stop.
  /// The forward boolean is used to denote whether this mark came from
  /// a forward or a backward traversal.
  //////////////////////////////////////////////////////////////////////
  bool add_mark(color_type mark, bool forward)
  {
    if (forward) {
      if (!m_forward.insert(mark).second) {
        return false;
      }
      if (m_backward.find(mark) != m_backward.end() &&
          (m_cc==invalid_cc() || m_overwrite(mark,m_cc))) {
        m_cc = mark;
      }
    } else {
      if (!m_backward.insert(mark).second) {
        return false;
      }
      if (m_forward.find(mark) != m_forward.end() &&
          (m_cc==invalid_cc() || m_overwrite(mark,m_cc))) {
        m_cc = mark;
      }
    }
    return true;
  }

  void clear_marks()
  {
    m_is_pivot = false;
    m_forward.clear();
    m_backward.clear();
  }

  void set_pivot()
  { m_is_pivot = true; }

  bool is_pivot() const
  { return m_is_pivot; }

  void set_cc(color_type const& cc)
  { m_cc = cc; }

  color_type get_cc() const
  { return m_cc; }

  bool has_valid_cc() const
  { return m_cc != invalid_cc(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief This function returns some information about the node that
  /// (hopefully) allows it to be distinguished from its neighbors,
  /// so that their edges can be broken.
  /// In the case of DCSCMulti, this information is the size of the
  /// forward and backward marking sets.
  //////////////////////////////////////////////////////////////////////
  mark_signature get_signature() const
  {
    if (m_cc == invalid_cc()) {
      return std::make_pair(m_forward.size(), m_backward.size());
    }
    return invalid_signature();
  }

  void add_predecessor(color_type v)
  { m_predecessors.insert(v); }

  void remove_predecessor(color_type v)
  { m_predecessors.erase(v); }

  void clear_predecessors()
  { m_predecessors.clear(); }

  reachables_iterator begin()
  { return m_predecessors.begin(); }

  reachables_iterator end()
  { return m_predecessors.end(); }

  static color_type invalid_cc()
  { return index_bounds<color_type>::invalid(); }

  static mark_signature invalid_signature()
  { return std::make_pair(-1, -1); }

  void define_type(typer& t)
  {
    t.member(m_cc);
    t.member(m_is_pivot);
    t.member(m_overwrite);
    t.member(m_forward);
    t.member(m_backward);
    t.member(m_predecessors);
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for the
/// @ref algo_details::pscc_vertex_property.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertGID, typename Cmp, typename Accessor>
class proxy<algo_details::pscc_vertex_property<VertGID, Cmp>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef algo_details::pscc_vertex_property<VertGID, Cmp>  target_t;
  typedef typename target_t::color_type                     color_type;
  typedef typename target_t::reachables_type                reachables_type;
  typedef typename target_t::reachables_iterator            reachables_iterator;
  typedef typename target_t::mark_signature                 mark_signature;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  {
    return Accessor::read();
  }

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

  // adds the mark, updating the cc if necessary and
  // returning whether the mark is new
  bool add_mark(color_type mark, bool forward)
  {
    return Accessor::invoke(&target_t::add_mark, mark, forward);
  }

  void clear_marks()
  { Accessor::invoke(&target_t::clear_marks); }

  void set_pivot()
  { Accessor::invoke(&target_t::set_pivot); }

  bool is_pivot() const
  { return Accessor::const_invoke(&target_t::is_pivot); }

  void set_cc(color_type const& cc)
  { Accessor::invoke(&target_t::set_cc, cc); }

  color_type get_cc() const
  { return Accessor::const_invoke(&target_t::get_cc); }

  bool has_valid_cc() const
  { return Accessor::const_invoke(&target_t::has_valid_cc); }

  mark_signature get_signature() const
  { return Accessor::const_invoke(&target_t::get_signature); }

  void add_predecessor(color_type v)
  { Accessor::invoke(&target_t::add_predecessor, v); }

  void remove_predecessor(color_type v)
  { Accessor::invoke(&target_t::add_predecessor, v); }

  void clear_predecessors()
  { Accessor::invoke(&target_t::clear_predecessors); }

  reachables_iterator begin()
  { return Accessor::invoke(&target_t::begin); }

  reachables_iterator end()
  { return Accessor::invoke(&target_t::end); }

  static color_type invalid_cc()
  { return index_bounds<color_type>::invalid(); }

  static mark_signature invalid_signature()
  { return Accessor::invoke(&target_t::invalid_signature); }
}; //struct proxy


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief This functor randomly selects nodes as pivots and begins a
/// traversal from them.
/// It is initialized with a chance (0 to 1) that each node
/// should be selected as a pivot. Then, for each uncolored node, it
/// draws a random number; if the node is selected as a pivot, it
/// begins a traversal from that node. It also returns a count
/// of how many pivots were selected for statistics purposes.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_set_pivots_and_traverse
  : public dynamic_wf
{
 public:
  typedef size_t  result_type;

 private:
  double m_chance;

 public:
  pscc_set_pivots_and_traverse(double chance)
    : m_chance(chance)
  {}

  template <typename TGV, typename Vertex, typename GView>
  result_type operator()(TGV tgv, Vertex v, GView g) const
  {
    typedef typename Vertex::vertex_descriptor  color_type;
    typedef pscc_async_bfs<color_type>          bfs;

    //if the probability is good, set v as a pivot
    if (!v.property().has_valid_cc() && drand48() <= m_chance) {
      v.property().set_pivot();

      bfs fw_bfs(v.descriptor(), true);
      fw_bfs(tgv, v, g);
      bfs bw_bfs(v.descriptor(), false);
      bw_bfs(tgv, v, g);

      return 1;
    } else {
      return 0;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_chance);
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Strongly-Connected Components Algorithm (pSCC)
///
/// This function executes the DCSCMulti algorithm on the input graph.
/// It begins by selecting approximately pivot_count pivots and then
/// adaptively selects a new value on each iteration. The SCC color
/// is stored in the property using a set_cc() function on the graph's
/// vertex property.
/// @param g The @ref graph_view over the input graph.
/// @param pivot_count The number of pivots to select at each iteration.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
void pscc(GraphView g, double pivot_count = 1)
{
  using namespace algo_details;

  typedef typename GraphView::vertex_descriptor vertex_descriptor;
  typedef pscc_vertex_property<vertex_descriptor> vertex_property;
  typedef graph<DIRECTED, MULTIEDGES, vertex_property> graph_type;
  typedef graph_view<graph_type> fview_type;

  graph_type gc(g.size());
  fview_type gcv(gc);
  copy_graph_struct_w_preds(g, gcv);
  srand48((get_location_id()+1)*time(0));

  while (true) {
    map_func(pscc_trim_forward(), gcv, make_repeat_view(gcv));
    map_func(pscc_trim_backward(), gcv, make_repeat_view(gcv));

    size_t node_count = graph_free_node_count(gcv);
    if (node_count == 0) {
      break;
    }

    algo_details::pscc_set_pivots_and_traverse cat(pivot_count/node_count);
    plus<size_t> sum_wf;
    auto&& gcv_rv = make_repeat_view(gcv);
    while ((pivot_count = map_reduce(cat, sum_wf, gcv, gcv_rv)) == 0);

    size_t success_count =
      map_reduce(pscc_separate_disjoint_return_successful(),
                 sum_wf, gcv, gcv_rv);

    pivot_count = std::ceil((2*success_count*success_count)/pivot_count);
  }
  map_func(copy_sccs_wf(), gcv, g);
}

} //namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_ALGORITHMS_PSCC_H
