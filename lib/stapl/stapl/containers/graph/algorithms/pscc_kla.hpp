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

#include <stapl/containers/graph/algorithms/pscc_kla_utils.hpp>
#include <stapl/containers/graph/algorithms/paradigms/kla_paradigm.hpp>

#include <stapl/skeletons/map.hpp>
#include <stapl/skeletons/map_reduce.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/views/repeated_view.hpp>

namespace stapl {

namespace algo_details {


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property class for the SCCMulti algorithm.
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
  typedef VertexGIDType                           color_type;
  typedef std::set<color_type, GIDCompare>        reachables_type;
  typedef typename reachables_type::iterator      reachables_iterator;
  typedef std::map<color_type, size_t>            markings_type;
  typedef typename markings_type::const_iterator  markings_iterator;
  typedef std::pair<size_t, size_t>               mark_signature;

 private:
  /// @brief Stores the node's color,
  /// or is invalid if it has not yet been colored.
  color_type m_cc;

  /// @brief A strict weak ordering of color type
  /// (for deciding which colors overwrite which colors,
  /// if a node is labeled by more than one pivot).
  GIDCompare m_overwrite;

  /// @brief The set of pivots that reach this node.
  markings_type m_forward;

  /// @brief The set of pivots that this node reaches.
  markings_type m_backward;

  /// @brief The reverse edge list of this node.
  reachables_type m_predecessors;

  /// @brief Bitfield indicating if vertex is part of
  /// forward traversal and backward traversal.
  std::map<color_type, std::bitset<2>> m_traversal_info;

 public:
  pscc_vertex_property(void)
    : m_cc(invalid_cc()),
      m_overwrite(GIDCompare())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief marks the node as a pivot.
  /// @param mark id of the traversal, in this case it is the node descriptor.
  //////////////////////////////////////////////////////////////////////
  void set_pivot(color_type mark)
  {
    m_forward.insert(std::make_pair(mark, 0));
    m_backward.insert(std::make_pair(mark, 0));
    m_cc = mark;
    // Mark the pivot as part of forward and backward traversals, and active.
    m_traversal_info.insert(std::make_pair(mark, std::bitset<2>(3)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief This function marks the node with a pivot and (if possible)
  /// updates the cc. If the node already has this mark, the function
  /// returns false, so that the traversal knows to stop.
  ///
  /// The forward boolean is used to denote whether this mark came from
  /// a forward or a backward traversal.
  //////////////////////////////////////////////////////////////////////
  bool set_level(color_type mark, bool forward, size_t level)
  {
    typedef std::map<size_t, std::bitset<2> >::iterator tranversal_iterator;

    // Get any existing traversal info
    tranversal_iterator traversal_info = m_traversal_info.find(mark);
    if (forward) {
      if (!m_forward.insert(std::make_pair(mark, level)).second) {
        return false;
      }

      // else forward traversal should continue
      if (traversal_info != m_traversal_info.end()) {
        traversal_info->second.set(0, true);
      } else {
        m_traversal_info.insert(std::make_pair(mark, std::bitset<2>(1)));
      }
      if (m_backward.find(mark) != m_backward.end() &&
          (m_cc==invalid_cc() || m_overwrite(mark, m_cc)) ) {
        m_cc = mark;
      }
    } else {
      if (!m_backward.insert(std::make_pair(mark, level)).second) {
        return false;
      }

      // else backward traversal should continue.
      if (traversal_info != m_traversal_info.end()) {
        traversal_info->second.set(1, true);
      } else {
        m_traversal_info.insert(std::make_pair(mark, std::bitset<2>(2)));
      }
      if (m_forward.find(mark) != m_forward.end() &&
          (m_cc==invalid_cc() || m_overwrite(mark, m_cc)) ) {
        m_cc = mark;
      }
    }
    return true;
  }

  void clear_marks(void)
  {
    m_forward.clear();
    m_backward.clear();
    m_traversal_info.clear();
  }

  std::vector<color_type> marks_at_level(bool forward, size_t level) const
  {
    std::vector<color_type> marks;
    if (forward) {
      for (auto&& mi : m_forward) {
        if (mi.second == level) {
          marks.push_back(mi.first);
        }
      }
    } else {
      for (auto&& mi : m_backward) {
        if (mi.second == level) {
          marks.push_back(mi.first);
        }
      }
    }
    return marks;
  }

  bool is_pivot(void) const
  { return marks_at_level(true, 0).size() != 0; }

  bool active_forward(color_type mark) const
  {
    std::map<size_t, std::bitset<2>>::const_iterator traversal_info =
      m_traversal_info.find(mark);
    if (traversal_info == m_traversal_info.end()) {
      return false;
    } else {
      return traversal_info->second.test(0);
    }
  }

  bool active_backward(color_type mark) const
  {
    auto traversal_info = m_traversal_info.find(mark);
    if (traversal_info == m_traversal_info.end()) {
      return false;
    } else {
      return traversal_info->second.test(1);
    }
  }

  void deactivate_forward(color_type mark)
  {
    auto traversal_info = m_traversal_info.find(mark);
    stapl_assert(traversal_info != m_traversal_info.end(),
      "mark not found in pscc_property::traversal_info.");
    traversal_info->second.set(0, false);
  }

  void deactivate_backward(color_type mark)
  {
    auto traversal_info = m_traversal_info.find(mark);
    stapl_assert(traversal_info != m_traversal_info.end(),
      "mark not found in pscc_property::traversal_info.");
    traversal_info->second.set(1, false);
  }

  void set_cc(color_type cc)
  { m_cc = cc; }

  color_type get_cc(void) const
  { return m_cc; }

  bool has_valid_cc(void) const
  { return m_cc != invalid_cc() && m_forward.size() + m_backward.size() == 0; }

  //////////////////////////////////////////////////////////////////////
  /// @brief This function returns some information about the node that
  /// (hopefully) allows it to be distinguished from its neighbors,
  /// so that their edges can be broken.
  /// In the case of SCCMulti, this information is the size of the
  /// forward and backward marking sets.
  //////////////////////////////////////////////////////////////////////
  mark_signature get_signature(void) const
  { return std::make_pair(m_forward.size(), m_backward.size()); }

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

  reachables_type get_predecessors(void) const
  { return m_predecessors; }

  size_t size(void) const
  { return m_predecessors.size(); }

  static color_type invalid_cc(void)
  { return index_bounds<color_type>::invalid(); }

  static mark_signature invalid_signature(void)
  { return std::make_pair(-1, -1); }

  void define_type(typer& t)
  {
    t.member(m_cc);
    t.member(m_overwrite);
    t.member(m_forward);
    t.member(m_backward);
    t.member(m_predecessors);
    t.member(m_traversal_info);
  }
};
} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for the @ref pscc_vertex_property.
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
  typedef typename target_t::markings_type                  markings_type;
  typedef typename target_t::markings_iterator              markings_iterator;
  typedef typename target_t::reachables_type                reachables_type;
  typedef typename target_t::reachables_iterator            reachables_iterator;
  typedef typename target_t::mark_signature                 mark_signature;

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

  std::vector<color_type> marks_at_level(bool forward, size_t level) const
  { return Accessor::const_invoke(&target_t::marks_at_level, forward, level); }

  void clear_marks(void)
  { Accessor::invoke(&target_t::clear_marks); }

  void set_pivot(color_type mark)
  { Accessor::invoke(&target_t::set_pivot, mark); }

  bool is_pivot(void) const
  { return Accessor::const_invoke(&target_t::is_pivot); }

  bool active_forward(color_type mark) const
  { return Accessor::const_invoke(&target_t::active_forward, mark); }

  bool active_backward(color_type mark) const
  { return Accessor::const_invoke(&target_t::active_backward, mark); }

  void deactivate_forward(color_type mark)
  { Accessor::invoke(&target_t::deactivate_forward, mark); }

  void deactivate_backward(color_type mark)
  { return Accessor::invoke(&target_t::deactivate_backward, mark); }

  void set_cc(color_type cc)
  { return Accessor::invoke(&target_t::set_cc, cc); }

  color_type get_cc(void) const
  { return Accessor::const_invoke(&target_t::get_cc); }

  bool has_valid_cc(void) const
  { return Accessor::const_invoke(&target_t::has_valid_cc); }

  mark_signature get_signature(void) const
  { return Accessor::const_invoke(&target_t::get_signature); }

  void add_predecessor(color_type v)
  { Accessor::invoke(&target_t::add_predecessor, v); }

  void remove_predecessor(color_type v)
  { Accessor::invoke(&target_t::add_predecessor, v); }

  void clear_predecessors(void)
  { Accessor::invoke(&target_t::clear_predecessors); }

  reachables_type get_predecessors(void) const
  { return Accessor::const_invoke(&target_t::get_predecessors); }

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
}; //struct proxy


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function to randomly select nodes as pivots.
///
/// The functor is initialized with a chance (0 to 1) that each node
/// should be selected as a pivot. Then, for each uncolored node, it
/// initializes the node as a pivot from which a traversal will begin.
//////////////////////////////////////////////////////////////////////
struct pscc_set_pivots
{
  typedef size_t result_type;

private:
  double m_chance;

public:
  pscc_set_pivots(double chance)
    : m_chance(chance)
  { }

  template <typename Vertex>
  result_type operator()(Vertex&& v) const
  {
    if (v.property().has_valid_cc()) {
      return 0;
    } else if (drand48() <= m_chance) {
      v.property().set_pivot(v.descriptor());
      return 1;
    }
    return 0;
  }

  void define_type(typer& t)
  { t.member(m_chance); }
};


//////////////////////////////////////////////////////////////////////
/// @brief This functor begins a traversal from a node if it is a pivot.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_traverse
{
  typedef bool result_type;

public:
  template <typename Vertex, typename GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    typedef typename std::decay<Vertex>::type::vertex_descriptor color_type;
    typedef pscc_async_bfs<color_type> bfs;
    typedef typename std::decay<Vertex>::type::property_type property_type;
    typedef typename property_type::reachables_iterator pred_iterator;

    if (v.property().has_valid_cc()) {
      return 0;
    } else {
      size_t level = graph_visitor.level();
      // if part of forward traversal, visit successors
      std::vector<color_type> fw_colors =
        v.property().marks_at_level(true, level-1);
      for (color_type& fc : fw_colors) {
        if (v.property().active_forward(fc)) {
          bfs fw_bfs(fc, true, level);
          for (auto&& i : v) {
            graph_visitor.visit(i.target(), fw_bfs);
          }
          v.property().deactivate_forward(fc);
        }
      }

      std::vector<color_type> bw_colors =
        v.property().marks_at_level(false, level-1);
      for (color_type& bc : bw_colors) {
        if (v.property().active_backward(bc)) {
          auto&& p = v.property().get_predecessors();
          bfs bw_bfs(bc, false, level);
          for (auto&& i2 : p) {
            graph_visitor.visit(i2, bw_bfs);
          }
          v.property().deactivate_backward(bc);
        }
      }

      return (fw_colors.size() != 0) || (bw_colors.size() != 0);
    }
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief SCCMulti algorithm for identifying Strongly Connected
/// Components (SCCs).
///
/// This function executes the SCCMulti algorithm on the input graph.
/// It begins by selecting approximately pivot_count pivots and then
/// adaptively selects a new value on each iteration. The SCC color
/// is stored in the property using a set_cc() function on the graph's
/// vertex property.
/// @param g The @ref graph_view over the input graph.
/// @param k The maximum amount of asynchrony for kla.
/// @param degree_percentage The threshold for a vertex's degree after which
/// it will be considered a hub (given as a percentage of total graph size).
/// @param pivot_count The number of pivots to select at each iteration.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
void pscc(GraphView& g, size_t k,
          float degree_percentage = 0.1, double pivot_count = 1)
{
  using namespace algo_details;

  typedef typename GraphView::vertex_descriptor vertex_descriptor;
  typedef pscc_vertex_property<vertex_descriptor> vertex_property;
  typedef graph<DIRECTED, MULTIEDGES, vertex_property> graph_type;
  typedef graph_view<graph_type> fview_type;

  size_t gsz = g.size();

  kla_params<fview_type> p;
  p.avoid_hubs = true;
  p.degree_threshold = degree_percentage*gsz;

  graph_type gc(gsz);
  fview_type gcv(gc);
  copy_graph_struct_w_preds(g, gcv);
  srand48((get_location_id()+1)*time(0));

  while (true) {
    //clear marks and count
    if (count_free_nodes(gcv) == 0) {
      break;
    }

    map_func(pscc_trim_forward(), gcv, make_repeat_view(gcv));
    map_func(pscc_trim_backward(), gcv, make_repeat_view(gcv));

    size_t node_count = count_free_nodes_and_clear(gcv);
    if (node_count == 0) {
      break;
    }

    algo_details::pscc_set_pivots set_pivots(pivot_count/node_count);
    while ((pivot_count = map_reduce(set_pivots, plus<size_t>(), gcv)) == 0);

    kla_paradigm(pscc_traverse(), pscc_async_bfs<vertex_descriptor>(),
                 gcv, k, p);

    size_t success_count =
      map_reduce(pscc_separate_disjoint_return_successful(), plus<size_t>(),
                 gcv, make_repeat_view(gcv));

    pivot_count = std::ceil((2*success_count*success_count)/pivot_count);
  }
  map_func(copy_sccs_wf(), gcv, g);
}

} //namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_ALGORITHMS_PSCC_H
