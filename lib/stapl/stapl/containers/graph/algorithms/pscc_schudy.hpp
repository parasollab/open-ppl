/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PSCC_SCHUDY_H
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PSCC_SCHUDY_H

#include <stapl/containers/graph/algorithms/pscc_utils.hpp>

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/map_view.hpp>

namespace stapl {

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Enum indicating a vertex's state in the binary search step
/// of Schudy's algorithm.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
enum BinarySearchFlag
{
  LOCKED_PIVOT,
  TEMPORARY_PIVOT,
  TEMPORARY_NONPIVOT,
  LOCKED_NONPIVOT
};


//////////////////////////////////////////////////////////////////////
/// @brief Enum indicating the quality of a set of pivots.
///
/// Schudy's algorithm (@ref pscc_schudy()) does a binary search on each
/// group. At each step in the search, the set of pivots is either too
/// small (it does not reach enough nodes), too big (it reaches enough
/// nodes but there is more than 1 temporary_pivot), or just right (it
/// reaches enough nodes AND there is exactly one temporary_pivot).
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
enum SearchDirection
{
  TOO_SMALL_FLAG,
  TOO_BIG_FLAG,
  JUST_RIGHT_FLAG
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property class for the @ref pscc_schudy() algorithm.
/// @tparam VertexGIDType The type of the vertex descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertexGIDType>
class pscc_schudy_vertex_property
{
 public:
  typedef VertexGIDType                         color_type;
  typedef std::set<color_type>                  reachables_type;
  typedef typename reachables_type::iterator    reachables_iterator;
  typedef size_t                                mark_signature;

 private:
  /// The node's color, or invalid if it has not yet been colored.
  color_type        m_cc;
  /// The pivot, if any, that reached this node (will always be
  /// equal to m_group or invalid).
  color_type        m_forward;
  /// The pivot, if any, that this node reaches (will always be
  /// equal to m_group or invalid).
  color_type        m_backward;
  /// This flag indicates that the node was reached by SOME node,
  /// not necessarily the pivot
  bool              m_marked;
  /// The reverse edge list on this node.
  reachables_type   m_predecessors;
  /// The pivot group of this node.
  mark_signature    m_group;
  /// Binary search flag indicating this node's state in the binary search.
  BinarySearchFlag  m_search_flag;
  /// Flag indicating the group's state in the binary search.
  SearchDirection   m_search_direction;

 public:
  pscc_schudy_vertex_property()
    : m_cc(invalid_cc()),
      m_marked(false),
      m_group(0),
      m_search_flag(TEMPORARY_NONPIVOT),
      m_search_direction(TOO_SMALL_FLAG)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Marks the node as reached from SOME node, not necessarily
  /// the pivot.
  /// @return True if node was previously marked, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool mark()
  {
    if (!m_marked) {
      m_marked = true;
      return false;
    } else {
      return true;
    }
  }

  bool marked() const
  { return m_marked; }

  void unmark()
  { m_marked = false; }

  SearchDirection direction() const
  { return m_search_direction; }

  void direction(SearchDirection s)
  { m_search_direction = s; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Marks the node with a pivot and (if possible) updates the cc.
  /// If the node already has this mark, the function returns false, so
  /// that the traversal knows to stop.
  /// The forward boolean is used to denote whether this mark came from
  /// a forward or a backward traversal.
  /// @return True if mark is from forward traversal, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool add_mark(color_type mark, bool forward)
  {
    if (forward) {
      if (m_forward != mark) {
        m_forward = mark;
        if (m_backward == mark) {
          m_cc = mark;
        }
      } else {
        return false;
      }
    } else {
      if (m_backward != mark) {
        m_backward = mark;
        if (m_forward == mark) {
          m_cc = mark;
        }
      } else {
        return false;
      }
    }
    return true;
  }

  BinarySearchFlag search_flag() const
  { return m_search_flag; }

  void search_flag(BinarySearchFlag flag)
  { m_search_flag = flag; }

  bool is_pivot() const
  { return m_search_flag == TEMPORARY_PIVOT; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the group's descriptor and clears old data.
  ///
  //////////////////////////////////////////////////////////////////////
  void update_group()
  {
    //update the group
    m_group *= 2;
    if (m_marked) {
      m_group += 1;
    }

    //update searching data if not a pivot
    m_marked = false;
    if (m_search_flag != TEMPORARY_PIVOT) {
      m_search_flag = TEMPORARY_NONPIVOT;
    }
    m_search_direction = TOO_SMALL_FLAG;

    //update scc data
    m_backward = m_forward = invalid_cc();
  }

  mark_signature get_group() const
  { return m_group; }

  void set_cc(color_type c)
  { m_cc = c; }

  color_type get_cc() const
  { return m_cc; }

  bool has_valid_cc() const
  { return m_cc != invalid_cc(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns some information about the node that (hopefully) allows
  /// it to be distinguished from its neighbors, so that their edges can be
  /// broken.
  /// In the case of Schudy's algorithm, this information is just the group
  /// descriptor.
  //////////////////////////////////////////////////////////////////////
  mark_signature get_signature() const
  {
    if (has_valid_cc()) {
      return invalid_signature();
    }
    return m_group;
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
  { return invalid_cc(); }

  void define_type(typer& t)
  {
    t.member(m_cc);
    t.member(m_marked);
    t.member(m_predecessors);
    t.member(m_forward);
    t.member(m_backward);
    t.member(m_group);
    t.member(m_search_flag);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Accumulates data about each of the groups. The three values
/// of the vector indicate:
///  0 - the number of pivots (nodes marked TEMPORARY_PIVOT)
///  1 - the number of marked nodes
///  2 - the number of nodes
/// The binary search can stop when v[1] >= v[2]/2 AND v[0] == 1.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class pscc_schudy_mark_counter
{
 private:
  std::vector<size_t> v;

 public:
  pscc_schudy_mark_counter()
    : v(3)
  { }

  void inc012()
  { ++v[0]; ++v[1]; ++v[2]; }

  void inc12()
  { ++v[1]; ++v[2]; }

  void inc2()
  { ++v[2]; }

  std::vector<size_t> counts() const
  { return v; }

  void counts(std::vector<size_t> c)
  { v = c; }

  void define_type(typer& t)
  {
    t.member(v);
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for
/// @ref algo_details::pscc_schudy_vertex_property.
/// @tparam VertGID The type of the vertex's descriptor.
/// @tparam Accessor The type of accessor for this proxy.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertGID, typename Accessor>
class proxy<algo_details::pscc_schudy_vertex_property<VertGID>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef algo_details::pscc_schudy_vertex_property<VertGID>           target_t;
  typedef typename target_t::color_type           color_type;
  typedef typename target_t::reachables_type      reachables_type;
  typedef typename target_t::reachables_iterator  reachables_iterator;
  typedef typename target_t::mark_signature       mark_signature;

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

  bool mark()
  { return Accessor::invoke(&target_t::mark); }

  bool marked() const
  { return Accessor::const_invoke(&target_t::marked); }

  void unmark()
  { Accessor::invoke(&target_t::unmark); }

  algo_details::SearchDirection direction() const
  { return Accessor::const_invoke(&target_t::direction); }

  void direction(algo_details::SearchDirection s)
  { Accessor::invoke(&target_t::direction, s); }

  // adds a mark, updating the cc, and returns whether the mark is new
  bool add_mark(color_type mark, bool forward)
  { return Accessor::invoke(&target_t::add_mark, mark, forward); }

  algo_details::BinarySearchFlag search_flag() const
  { return Accessor::const_invoke(&target_t::search_flag); }

  void search_flag(algo_details::BinarySearchFlag flag)
  { Accessor::invoke(&target_t::search_flag, flag); }

  bool is_pivot() const
  { return Accessor::const_invoke(&target_t::is_pivot); }

  void update_group()
  { Accessor::invoke(&target_t::update_group); }

  mark_signature get_group() const
  { return Accessor::const_invoke(&target_t::get_group); }

  void set_cc(color_type c)
  { Accessor::invoke(&target_t::set_cc, c); }

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


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for
/// @ref algo_details::pscc_schudy_mark_counter.
/// @tparam Accessor The type of accessor for this proxy.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename Accessor>
class proxy<algo_details::pscc_schudy_mark_counter, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef algo_details::pscc_schudy_mark_counter target_t;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
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

  void inc012()
  { Accessor::invoke(&target_t::inc012); }

  void inc12()
  { Accessor::invoke(&target_t::inc12); }

  void inc2()
  { Accessor::invoke(&target_t::inc2); }

  std::vector<size_t> counts() const
  { return Accessor::const_invoke(&target_t::counts); }

  void counts(std::vector<size_t> c)
  { Accessor::invoke(&target_t::counts, c); }
}; //struct proxy

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Does a traversal from any node that is flagged as either a
/// TEMPORARY_PIVOT or a LOCKED_PIVOT. Any node that the traversal reaches
/// is marked using <tt>property().mark()</tt>.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_schudy_marking_bfs
{
  typedef pscc_schudy_marking_bfs      this_type;
  typedef void                result_type;

  template <typename TGV, typename Vertex, typename Graph>
  result_type operator()(TGV tgv, Vertex v, Graph g)
  {
    if (v.property().mark()) {
      return;
    }

    for (auto&& i : v) {
      tgv.add_task(*this, localize_ref(tgv),
          localize_ref(g, i.target()), localize_ref(g));
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Schudy's algorithm randomly permutes the nodes and then
/// performs a binary search, looking for the minimal prefix of the
/// nodes that reaches at least half of the graph.
/// Instead of this, we perform a "randomized binary search," in
/// which we either increase or decrease the pivot nodes based on the
/// last iteration's performance.
/// If the reached set is too big, all of the nodes marked as
/// TEMPORARY_NONPIVOT are set to LOCKED_NONPIVOT. Then half of the
/// nodes set as TEMPORARY_PIVOT are switched to TEMPORARY_NONPIVOT.
/// If the reached set is too small, the nodes marked as TEMPORARY_PIVOT
/// are set to LOCKED_PIVOT, and half of the nodes that were
/// TEMPORARY_NONPIVOT are changed to TEMPORARY_PIVOT.
/// The algorithm is complete when exactly one node is marked TEMPORARY_PIVOT.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_schudy_search_step
  : public dynamic_wf
{
  typedef void result_type;

  template <typename TGV, typename Vertex, typename GView>
  void operator()(TGV tgv, Vertex v, GView g)
  {
    if (v.property().has_valid_cc() ||
        v.property().direction() == JUST_RIGHT_FLAG) {
      return;
    }

    if (v.property().direction() == TOO_SMALL_FLAG) {
      if (v.property().search_flag() == TEMPORARY_PIVOT) {
        v.property().search_flag(LOCKED_PIVOT);
      } else if (v.property().search_flag() == TEMPORARY_NONPIVOT &&
          drand48() < .5) {
        v.property().search_flag(TEMPORARY_PIVOT);
      }
    } else if (v.property().direction() == TOO_BIG_FLAG) {
      if (v.property().search_flag() == TEMPORARY_NONPIVOT){
        v.property().search_flag(LOCKED_NONPIVOT);
      } else if (v.property().search_flag() == TEMPORARY_PIVOT &&
          drand48() < .5) {
        v.property().search_flag(TEMPORARY_NONPIVOT);
      }
    }

    //bfs only if the marking is positive
    if (v.property().search_flag() <= TEMPORARY_PIVOT &&
        !v.property().marked()) {
      pscc_schudy_marking_bfs bfs;
      bfs(tgv, v, g);
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Updates the search direction and removes the reached information
/// for groups that have not finished the search.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_schudy_unmark
{
  typedef void result_type;

  template<typename Vertex, typename AView>
  void operator()(Vertex v, AView av)
  {
    if (v.property().has_valid_cc() ||
        v.property().direction() == JUST_RIGHT_FLAG) {
      return;
    }

    v.property().direction(av[v.property().get_group()]);
    if (v.property().direction() != JUST_RIGHT_FLAG) {
      v.property().unmark();
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Populates the @ref algo_details::pscc_schudy_mark_counter
/// for each group.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_schudy_count
{
  typedef void result_type;

  template<typename Vertex, typename CountVector>
  void operator()(Vertex v, CountVector cv)
  {
    if (v.property().has_valid_cc() ||
        v.property().direction() == JUST_RIGHT_FLAG) {
      return;
    }

    if (v.property().is_pivot()) {
      cv[v.property().get_group()].inc012();
    } else if (v.property().marked()) {
      cv[v.property().get_group()].inc12();
    } else {
      cv[v.property().get_group()].inc2();
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Uses the @ref algo_details::pscc_schudy_mark_counter
/// information to determine the search direction for each group. If
/// any groups are not finished, it returns false so that the binary
/// search may continue.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_schudy_evaluate_groups
{
  typedef bool    result_type;
  template<typename Ratios, typename Actions>
  result_type operator()(const Ratios r, Actions a)
  {
    std::vector<size_t> counts = r.counts();
    if ((2*counts[1]) < counts[2]) {
      a = TOO_SMALL_FLAG;
    } else if (counts[0] == 1 || counts[2] == 0) {
      a = JUST_RIGHT_FLAG;
    } else {
      a = TOO_BIG_FLAG;
    }

    return a == JUST_RIGHT_FLAG;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Populates the @ref algo_details::pscc_schudy_mark_counter
/// for each group that exists on this location. The counters are then
/// combined using @ref pscc_schudy_count_reduction. Together, these two
/// functions do the same work as @ref pscc_schudy_count, but with less
/// communication when the number of groups is very small.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_schudy_count_local
{
 public:
  typedef std::vector<pscc_schudy_mark_counter>    result_type;

 private:
  size_t m_group_ct;

 public:
  pscc_schudy_count_local(size_t gc)
    : m_group_ct(gc)
  { }

  template<typename SubGraph>
  result_type operator()(SubGraph g)
  {
    result_type cv(m_group_ct);
    for (auto&& v : g) {
      if (v.property().has_valid_cc() ||
        v.property().direction() == JUST_RIGHT_FLAG) {
        continue;
      }

      if (v.property().is_pivot()) {
        cv[v.property().get_group()].inc012();
      } else if (v.property().marked()) {
        cv[v.property().get_group()].inc12();
      } else {
        cv[v.property().get_group()].inc2();
      }
    }
    return cv;
  }

  void define_type(typer& t)
  { t.member(m_group_ct); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Combines the @ref algo_details::pscc_schudy_mark_counter information
/// from @ref pscc_schudy_count_local.
/// Together, these two functions do the same work as @ref pscc_schudy_count,
/// but with less communication when the number of groups is very small.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_schudy_count_reduction
{
  typedef pscc_schudy_count_local::result_type  result_type;

  result_type operator()(result_type t, result_type const& s)
  {
    for (size_t i=0; i<t.size(); ++i) {
      std::vector<size_t> tc = t[i].counts();
      std::vector<size_t> sc = s[i].counts();
      tc[0] += sc[0];
      tc[1] += sc[1];
      tc[2] += sc[2];
      t[i].counts(tc);
    }
    return t;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Uses the @ref algo_details::pscc_schudy_mark_counter informations
/// to determine the search direction for each group. If any groups are
/// not finished, it returns false so that the binary search may continue.
/// This does the same work as @ref pscc_schudy_evaluate_groups, but with less
/// communication when the number of groups is small.
/// @param r An std::vector of @ref algo_details::pscc_schudy_mark_counter
/// instances. It contains one @ref algo_details::pscc_schudy_mark_counter
/// for each group.
/// @param a An std::vector of SearchDirection instances.
/// @return true if more work is needed, false otherwise.
/// @see SearchDirection
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Ratios, typename Actions>
bool pscc_schudy_evaluate_groups_local(Ratios const& r, Actions& a)
{
  bool good = true;
  for (size_t i=0; i<r.size(); ++i) {
    std::vector<size_t> counts = r[i].counts();
    if ((2*counts[1]) < counts[2]) {
      a[i] = TOO_SMALL_FLAG;
      good = false;
    } else if (counts[0] == 1 || counts[2] == 0) {
      a[i] = JUST_RIGHT_FLAG;
    } else {
      a[i] = TOO_BIG_FLAG;
      good = false;
    }
  }

  return !good;
}


//////////////////////////////////////////////////////////////////////
/// @brief Updates the search direction and removes the reached information
/// for groups that have not finished the search. It does the same work as
/// @ref pscc_schudy_unmark, but is faster when the number of groups is small.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Vec>
struct pscc_schudy_unmark_local
{
 private:
  const Vec m_av;

 public:
  typedef void result_type;

  pscc_schudy_unmark_local(Vec const& av)
    : m_av(av)
  { }

  template<typename SubGraph>
  void operator()(SubGraph g)
  {
    for (auto&& i : g) {
      i.property().direction(m_av[i.property().get_group()]);
      if (i.property().direction() != JUST_RIGHT_FLAG) {
        i.property().unmark();
      }
    }
  }

  void define_type(typer& t)
  { t.member(m_av); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Runs the binary search and selects the final pivot for each
/// group, using the subfunctions above. The number of groups is 2^depth,
/// since Schudy's algorithm basically guarantees an even division of
/// nodes on each iteration.
/// The algorithm works in two different ways; one does the work locally
/// and then compiles it, while the other does all work globally. The local
/// work is more efficient when the number of groups is small, but gets
/// worse as the number of groups increases.
/// @param g The input @ref graph_view.
/// @param depth The depth.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Graph>
void pscc_schudy_pivot_selector(Graph& g, size_t depth)
{
  typedef std::vector<SearchDirection> actions_array;

  size_t num_groups = std::pow(2, depth-1);
  actions_array av(num_groups);

  size_t depth2 = 0;

  //there are two ways to count over g groups
  // a) make a vector of the values and reduce with a summation
  //    this is the best option when the number of values is small
  // b) distribute the values and have vertices call a counter function
  //    this is always bad, but will be better when the number of values is big
  if (depth < 17) {
    typedef std::vector<SearchDirection> actions_array;
    typedef std::vector<pscc_schudy_mark_counter> counts_array;

    actions_array av(num_groups);

    bool searching = true;
    while (searching) {
      ++depth2;

      if (depth2 > 1) {
        map_func(pscc_schudy_unmark_local<actions_array>(av),
                 stapl::native_view(g));
      }
      map_func(pscc_schudy_search_step(), g, make_repeat_view(g));
      counts_array rv = map_reduce(pscc_schudy_count_local(num_groups),
                                   pscc_schudy_count_reduction(),
                                   stapl::native_view(g));
      searching = pscc_schudy_evaluate_groups_local(rv, av);
    }
  } else {
    typedef array<pscc_schudy_mark_counter> counts_array;
    typedef array_view<counts_array> counts_view;
    typedef array<SearchDirection> actions_array;
    typedef array_view<actions_array> actions_view;

    actions_array actions(num_groups);
    actions_view av(actions);

    bool searching = true;
    while (searching) {
      ++depth2;

      if (depth2 > 1) {
        map_func(pscc_schudy_unmark(), g, make_repeat_view(av));
      }
      map_func(pscc_schudy_search_step(), g, make_repeat_view(g));
      counts_array ratios(num_groups);
      counts_view rv(ratios);
      map_func(pscc_schudy_count(), g, make_repeat_view(rv));
      if (rv.size() ==
          map_reduce(pscc_schudy_evaluate_groups(), plus<size_t>(), rv, av)) {
        searching = false;
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Updates the group of each node (2*old_group + 0/1) and then
/// traverses from the one node marked TEMPORARY_PIVOT in each group.
/// This traversal sets the forward/backward flag, and not just the
/// mark boolean.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_schudy_bfs_start
  : public dynamic_wf
{
  typedef void result_type;

  template <typename TGV, typename Vertex, typename GView>
  void operator()(TGV tgv, Vertex v, GView g) const
  {
    typedef typename Vertex::vertex_descriptor        color_type;
    typedef pscc_async_bfs<color_type>                bfs;

    if (v.property().has_valid_cc()) {
      return;
    }
    v.property().update_group();

    //if it's not the pivot, don't start here
    if (!v.property().is_pivot()) {
      return;
    }

    color_type vd = v.descriptor();

    bfs fw_bfs(vd, true);
    fw_bfs(tgv, v, g);
    bfs bw_bfs(vd, false);
    bw_bfs(tgv, v, g);
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief This is an implementation of Schudy's MultiPivot SCC algorithm,
///
/// This algorithm executes Schudy's SCC algorithm on the input graph.
/// It first finds a pivot such that the pivot and some other nodes reach at
/// least half of the graph. It then tests reachability from the pivot and
/// divides the graph into the unreached and reached node sets.
/// @note This algorithm is O(log n) times slower than DCSCMulti (pscc.h),
/// and so should not be used other than for comparison.
/// @param g The input @ref graph_view.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
void pscc_schudy(GraphView g)
{
  using namespace algo_details;

  typedef typename GraphView::vertex_descriptor vertex_descriptor;
  typedef pscc_schudy_vertex_property<vertex_descriptor> vertex_property;
  typedef graph<DIRECTED, MULTIEDGES, vertex_property> graph_type;
  typedef graph_view<graph_type> fview_type;

  graph_type gc(g.size());
  fview_type gcv(gc);
  copy_graph_struct_w_preds(g, gcv);

  srand48((get_location_id()+1)*time(0));

  int recursion_depth = 0;
  while (true) {
    ++recursion_depth;

    map_func(pscc_trim_forward(), gcv, make_repeat_view(gcv));
    map_func(pscc_trim_backward(), gcv, make_repeat_view(gcv));

    if (gcv.num_edges_collective() == 0) {
      break;
    }

    pscc_schudy_pivot_selector(gcv, recursion_depth);
    map_func(pscc_schudy_bfs_start(), gcv, make_repeat_view(gcv));
    map_func(pscc_separate_disjoint_return_uncolored(),
      gcv, make_repeat_view(gcv));
  }

  map_func(copy_sccs_wf(), gcv, g);
}

} // namespace stapl

#endif
