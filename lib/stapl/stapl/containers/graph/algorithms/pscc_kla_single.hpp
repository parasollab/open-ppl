/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_SINGLE_H
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_SINGLE_H

#include <stapl/containers/graph/algorithms/pscc_kla_utils.hpp>
#include <stapl/containers/graph/algorithms/paradigms/kla_paradigm.hpp>

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/map.hpp>

namespace stapl {

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief This is the complex group descriptor. It is
/// a <VertexDescriptor, char> pair. Many operators are defined on it
/// so that it will work as a map key for the pivot selection map.
///
/// For the char value:
/// 0 indicates that it was not reached by m_group,
/// 1 indicates that m_group reached it backward only,
/// 2 indicates that m_group reached it forward only, and
/// 3 indicates that it was colored by m_group.
/// @tparam VertexDescriptor The type of the vertex descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertexDescriptor>
class pscc_single_mark_type
{
 public:
  typedef pscc_single_mark_type<VertexDescriptor>  this_type;
  typedef VertexDescriptor                  color_type;
  typedef char                              direction_type;

  friend struct hash<this_type>;

 private:
  color_type        m_group;
  direction_type    m_direction;

 public:
  explicit pscc_single_mark_type(void)
    : m_group(index_bounds<color_type>::lowest()),
      m_direction(0)
  { }

  explicit pscc_single_mark_type(color_type const& group)
    : m_group(group),
      m_direction(0)
  { }

  explicit pscc_single_mark_type(color_type const& group,
    direction_type const& dir)
    : m_group(group),
      m_direction(dir)
  { }

  bool operator==(this_type const& p) const
  { return p.m_group == this->m_group && p.m_direction == this->m_direction; }

  bool operator!=(this_type const& p) const
  { return !(*this == p); }

  color_type operator+(this_type const& other) const
  { return this->make_color_type() + other.make_color_type(); }

  color_type operator-(this_type const& other) const
  { return this->make_color_type() - other.make_color_type(); }

  inline bool operator<(this_type const& other) const
  {
    return m_group < other.m_group ||
      ((other.m_group == m_group) && (m_direction < other.m_direction));
  }

  inline bool operator>(this_type const& other) const
  { return !(*this == other) && !(*this < other); }

  inline bool operator<= (this_type const& other) const
  { return *this < other || *this == other; }

  this_type operator+(size_t s) const
  {
    size_t intermed = this->make_color_type() + s;
    return this_type(intermed/3, intermed%3);
  }

  this_type operator-(size_t s) const
  {
    size_t intermed = this->make_color_type() - s;
    return this_type(intermed/3, intermed%3);
  }

  const color_type make_color_type(void) const
  { return 3*m_group + m_direction; }

  void define_type(typer &t)
  {
    t.member(m_group);
    t.member(m_direction);
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief This allows pscc_single_mark_type to be used as a STAPL map key.
/// @tparam VertexDescriptor The type of the vertex descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertexDescriptor>
struct hash<algo_details::pscc_single_mark_type<VertexDescriptor> >
{
  std::size_t operator()(
    algo_details::pscc_single_mark_type<VertexDescriptor> const& t) const
  { return t.m_group*3+t.m_direction; }
};


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex property class for the pSCC algorithm.
/// @tparam VertexGIDType The type of the vertex descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertexGIDType>
class pscc_single_vertex_property
{
 public:
  typedef VertexGIDType                         color_type;
  typedef std::set<color_type>                  reachables_type;
  typedef typename reachables_type::iterator    reachables_iterator;
  typedef algo_details::pscc_single_mark_type<color_type>     mark_signature;


 private:
  /// @brief Stores the node's color, or is invalid if it has not yet been
  /// colored.
  color_type      m_cc;

  /// @brief The pivot, if any, that reached this node
  /// (either equal to m_group or invalid).
  size_t          m_forward_level;

  /// @brief The pivot, if any, that this node reaches
  /// (either equal to m_group or invalid).
  size_t          m_backward_level;

  /// @brief The reverse edge list of this node.
  reachables_type m_predecessors;

  /// @brief The pivot that may potentially reach this node.
  color_type      m_group;

  /// @brief The active mark used by the cc algorithm
  bool            m_active;

 public:
  pscc_single_vertex_property(void)
    : m_cc(invalid_cc()),
      m_forward_level(std::numeric_limits<size_t>::max()),
      m_backward_level(std::numeric_limits<size_t>::max()),
      m_group(index_bounds<color_type>::lowest())
  { }

  void cc(color_type c)
  { m_group = c; }

  color_type cc(void) const
  { return m_group; }

  void active(bool a)
  { m_active = a; }

  bool active(void) const
  { return m_active; }

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
    if (forward) {
      if (m_forward_level == std::numeric_limits<size_t>::max()) {
        m_forward_level = level;
        if (m_backward_level != std::numeric_limits<size_t>::max()) {
          m_cc = mark;
        }
      } else {
        return false;
      }
    } else {
      if (m_backward_level == std::numeric_limits<size_t>::max()) {
        m_backward_level = level;
        if (m_forward_level != std::numeric_limits<size_t>::max()) {
          m_cc = mark;
        }
      } else {
        return false;
      }
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
  //////////////////////////////////////////////////////////////////////
  /// @brief Sets a simple descriptor on this node and clears the old markings.
  //////////////////////////////////////////////////////////////////////
  void update_group(color_type group)
  {
    m_group = group;
    m_backward_level = m_forward_level = std::numeric_limits<size_t>::max();
  }

  color_type get_group(void) const
  { return m_group; }

  void set_cc(color_type c)
  { m_cc = c; }

  color_type get_cc(void) const
  { return m_cc; }

  bool has_valid_cc(void) const
  { return m_cc != invalid_cc(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns some information about the node that (hopefully) allows
  /// it to be distinguished from its neighbors, so that their edges can be
  /// broken.
  /// In the case of DCSC, this information is the complex group descriptor.
  ///
  /// Signature is a pair (mark, direction), where mark is the mark used on
  /// this node and direction is obtained by the markings on it
  /// available directions: (0 none, 1 backward, 2 forward, 3 both).
  //////////////////////////////////////////////////////////////////////
  mark_signature get_signature(void) const
  {
    size_t invalid = std::numeric_limits<size_t>::max();
    return mark_signature(m_group,
      (char)((( m_forward_level == invalid) ? 0 : 2) |
             ((m_backward_level == invalid) ? 0 : 1)));
  }

  void add_predecessor(color_type v)
  { m_predecessors.insert(v); }

  void remove_predecessor(color_type v)
  { m_predecessors.erase(v); }

  void clear_predecessors(void)
  { m_predecessors.clear(); }

  bool has_predecessor(color_type v) const
  { return (m_predecessors.find(v) != m_predecessors.end()); }

  reachables_iterator begin(void)
  { return m_predecessors.begin(); }

  reachables_iterator end(void)
  { return m_predecessors.end(); }

  size_t size(void) const
  { return m_predecessors.size(); }

  color_type first(void) const
  { return *m_predecessors.begin(); }

  static color_type invalid_cc(void)
  { return index_bounds<color_type>::invalid(); }

  static mark_signature invalid_signature(void)
  { return mark_signature(invalid_cc()); }

  void define_type(typer& t)
  {
    t.member(m_cc);
    t.member(m_forward_level);
    t.member(m_backward_level);
    t.member(m_predecessors);
    t.member(m_group);
    t.member(m_active);
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref pscc_single_vertex_property.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertGID, typename Accessor>
class proxy<algo_details::pscc_single_vertex_property<VertGID>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef algo_details::pscc_single_vertex_property<VertGID> target_t;
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

  void cc(color_type c)
  { Accessor::invoke(&target_t::cc, c); }

  color_type cc(void) const
  { return Accessor::const_invoke(&target_t::cc); }

  void active(bool a)
  { Accessor::invoke(&target_t::active, a); }

  bool active(void) const
  { return Accessor::const_invoke(&target_t::active); }

  bool set_level(color_type mark, bool forward, size_t level)
  { return Accessor::invoke(&target_t::set_level, mark, forward, level); }

  size_t level(bool forward) const
  { return Accessor::const_invoke(&target_t::level,forward); }

  void update_group(color_type v)
  { Accessor::invoke(&target_t::update_group, v); }

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

  bool has_predecessor(color_type v) const
  { return Accessor::const_invoke(&target_t::has_predecessor, v); }

  reachables_iterator begin(void)
  { return Accessor::invoke(&target_t::begin); }

  reachables_iterator end(void)
  { return Accessor::invoke(&target_t::end); }

  size_t size(void) const
  { return Accessor::const_invoke(&target_t::size); }

  color_type first(void) const
  { return Accessor::const_invoke(&target_t::first); }

  static color_type invalid_cc(void)
  { return index_bounds<color_type>::invalid(); }

  static mark_signature invalid_signature(void)
  { return Accessor::invoke(&target_t::invalid_signature); }
}; //struct proxy


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief This functor uniformly selects a pivot for each group by
/// inserting one of its nodes into a map.
///
/// The map also contains a count so that the pivot is selected uniformly.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertexDescriptor>
class pscc_single_map_insert
{
 private:
  VertexDescriptor m_vert;

 public:
  typedef void    result_type;

  pscc_single_map_insert(VertexDescriptor what)
    : m_vert(what)
  { }

  template<typename MapPair, typename T>
  result_type operator()(MapPair& where, const T&) const
  {
    ++where.second.second;
    if (drand48() < 1.0/where.second.second) {
      where.second.first = m_vert;
    }
  }

  void define_type(typer& t)
  { t.member(m_vert); }
};


//////////////////////////////////////////////////////////////////////
/// @brief This functor selects one pivot from each group by attempting
/// to insert each vertex into a map using @ref pscc_single_map_insert.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_single_pivot_selector
{
  typedef void result_type;

  template <typename Vertex, typename Map>
  void operator()(Vertex v, Map& m) const
  {
    typedef typename Vertex::vertex_descriptor        vertex_descriptor;
    typedef pscc_single_map_insert<vertex_descriptor>      insert_func;

    if (v.property().has_valid_cc()) {
      return;
    }

    m.insert(v.property().get_signature(),
      std::make_pair(v.descriptor(),1), insert_func(v.descriptor()));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This functor updates the simple descriptor of each node.
/// If the node is a pivot, this functor traverses from it.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename MView>
struct pscc_single_bfs_start
{
private:
  MView* m_mview;

public:
  typedef bool result_type;

  pscc_single_bfs_start(MView& mview)
    : m_mview(&mview)
  { }

  template <typename Vertex, typename GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    typedef typename std::decay<Vertex>::type::vertex_descriptor color_type;
    typedef pscc_async_bfs<color_type> bfs;
    typedef typename std::decay<Vertex>::type::property_type property_type;
    typedef typename property_type::reachables_iterator pred_iterator;

    //if this pivot has a mark from a previous generation, don't use it
    if (v.property().level(true) == std::numeric_limits<size_t>::max() &&
        v.property().has_valid_cc()) {
      return false;
    }

    //find the pivot for this node's group
    size_t level = graph_visitor.level();
    if (level == 1) {
      if (v.property().has_valid_cc()) {
        v.property().update_group(0);
        return false;
      }

      color_type pivot = m_mview->get(v.property().get_signature()).first;
      v.property().update_group(pivot);
      if (v.descriptor() == pivot) {
        bfs fw_bfs(pivot, true, level);
        fw_bfs(v);
        for (auto&& i : v) {
          graph_visitor.visit(i.target(), fw_bfs);
        }

        bfs bw_bfs(pivot, false, level);
        bw_bfs(v);
        property_type p = v.property();
        for (auto&& i : p) {
          graph_visitor.visit(i, bw_bfs);
        }
        return true;
      } else {
        return false;
      }
    } else {
      bool result = false;
      if (v.property().level(true) == level-1) {
        bfs fw_bfs(v.property().get_group(), true, level);
        for (auto&& i : v) {
          graph_visitor.visit(i.target(), fw_bfs);
        }
        result = true;
      }

      if (v.property().level(false) == level-1) {
        property_type p = v.property();
        bfs bw_bfs(v.property().get_group(), false, level);
        for (auto&& i : p) {
          graph_visitor.visit(i, bw_bfs);
        }
        result = true;
      }
      return result;
    }
  }

  void define_type(typer& t)
  { t.member(m_mview); }
};

} //namespace algo_details



//////////////////////////////////////////////////////////////////////
/// This function executes the DCSC algorithm on the graph @p g.
/// The algorithm works by selecting a pivot, traversing from this pivot,
/// breaking the graph into subpieces, and then repeating this operation
/// on each of the subpieces.
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
void pscc_single(GraphView& g, size_t k, float degree_percentage=0.1)
{
  using namespace algo_details;

  typedef typename GraphView::vertex_descriptor           vertex_descriptor;
  typedef pscc_single_vertex_property<vertex_descriptor>  vertex_property;
  typedef graph<DIRECTED, MULTIEDGES, vertex_property>    graph_type;
  typedef graph_view<graph_type>                          fview_type;

  typedef typename vertex_property::mark_signature        group_type;
  typedef group_type                                      map_key_type;
  typedef std::pair<vertex_descriptor, size_t>            map_value_type;
  typedef stapl::map<map_key_type, map_value_type>        pivots_map;
  typedef stapl::indexed_domain<map_key_type>             pivots_domain;
  typedef stapl::map_view<pivots_map>                     pivots_view;

  size_t g_sz = g.size();

  kla_params<fview_type> p;
  p.avoid_hubs = true;
  p.degree_threshold = degree_percentage*g_sz;

  graph_type gc(g_sz);
  fview_type gcv(gc);
  copy_graph_struct_w_preds(g, gcv);

  srand48((get_location_id()+1)*time(0));

  pivots_domain dom(group_type(0, 0), group_type(g_sz-1, 2));

  while (true) {
    map_func(pscc_trim_forward(), gcv, make_repeat_view(gcv));
    map_func(pscc_trim_backward(), gcv, make_repeat_view(gcv));

    if (count_free_nodes(gcv) == 0) {
      break;
    }

    pivots_map pivots(dom);
    pivots_view pv(pivots);
    map_func(pscc_single_pivot_selector(), gcv, make_repeat_view(pv));

    algo_details::pscc_single_bfs_start<pivots_view> start(pv);
    kla_paradigm(start, pscc_async_bfs<vertex_descriptor>(), gcv, k, p);

    map_func(pscc_separate_disjoint_return_uncolored(), gcv,
             make_repeat_view(gcv));

  }

  stapl::map_func(copy_sccs_wf(), gcv, g);
}

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_ALGORITHMS_SINGLE_H

