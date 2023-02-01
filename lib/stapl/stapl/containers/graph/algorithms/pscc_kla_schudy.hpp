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

#include <stapl/containers/graph/algorithms/pscc_kla_utils.hpp>
#include <stapl/containers/graph/algorithms/paradigms/kla_paradigm.hpp>

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/map_view.hpp>

namespace stapl {

namespace algo_details {

template <typename VertexDescriptor>
class pscc_schudy_mark_type;

}

template <typename VertexDescriptor>
struct hash<algo_details::pscc_schudy_mark_type<VertexDescriptor> >;

}


namespace stapl {

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief This is the complex group descriptor.
///
/// It is a <VertexDescriptor, char> pair. Many operators are defined on it
/// so that it will work as a map key for the pivot selection map. For
/// the char value (see Schudy's paper):
/// 0 indicates V\A\B,
/// 1 indicates A\B,
/// 2 indicates B\A\C, and
/// 3 indicates A-intersect-B\C.
/// @tparam VertexDescriptor The type of the vertex descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertexDescriptor>
class pscc_schudy_mark_type
{
public:
  typedef pscc_schudy_mark_type<VertexDescriptor>  this_type;
  typedef VertexDescriptor                  color_type;
  typedef char                              direction_type;

  friend struct hash<this_type>;

private:
  color_type        m_group;
  direction_type    m_direction;

public:
  explicit pscc_schudy_mark_type(void)
    : m_group(index_bounds<color_type>::lowest()),
      m_direction(0)
  { }

  explicit pscc_schudy_mark_type(color_type group,
                                 direction_type dir)
    : m_group(group),
      m_direction(dir)
  { }

  pscc_schudy_mark_type(size_t stored_mark)
    : m_group(stored_mark/4),
      m_direction(stored_mark%4)
  { }

  operator size_t(void) const
  {
    return m_group*4 + m_direction;
  }

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
    return (m_group < other.m_group) ||
           ((other.m_group == m_group) && (m_direction < other.m_direction));
  }

  inline bool operator>(this_type const& other) const
  { return !(*this == other) && !(*this < other); }

  inline bool operator<= (this_type const& other) const
  { return *this < other || *this == other; }

  this_type operator+(size_t s) const
  {
    size_t intermed = this->make_color_type() + s;
    return this_type(intermed/4, intermed%4);
  }

  this_type operator-(size_t s) const
  {
    size_t intermed = this->make_color_type() - s;
    return this_type(intermed/4, intermed%4);
  }

  const color_type make_color_type(void) const
  { return m_group*4+m_direction; }

  void define_type(typer &t)
  {
    t.member(m_group);
    t.member(m_direction);
  }
};

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
/// Schudy's algorithm (@ref pscc_schudy) does a binary search on each
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

} //algo_details

//////////////////////////////////////////////////////////////////////
/// @brief This allows pscc_schudy_mark_type to be used as a STAPL map key.
/// @tparam VertexDescriptor The type of the vertex descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertexDescriptor>
struct hash<algo_details::pscc_schudy_mark_type<VertexDescriptor> >
{
  std::size_t operator()(
    algo_details::pscc_schudy_mark_type<VertexDescriptor> const& t) const
  {
    return 4*t.m_group + t.m_direction;
  }
};

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex property class for the @ref pscc_schudy algorithm.
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
  typedef pscc_schudy_mark_type<color_type>     mark_signature;

private:
  /// The node's color, or invalid if it has not yet been colored.
  color_type        m_cc;
  /// The pivot, if any, that reached this node (will always be
  /// equal to m_group or invalid).
  size_t            m_forward_level;
  /// The pivot, if any, that this node reaches (will always be
  /// equal to m_group or invalid).
  size_t            m_backward_level;
  /// This flag indicates that the node was reached by SOME node,
  /// not necessarily the pivot
  size_t            m_level;
  /// The reverse edge list on this node.
  reachables_type   m_predecessors;
  /// The pivot group of this node.
  mark_signature    m_group;
  /// Binary search flag indicating this node's state in the binary search.
  BinarySearchFlag  m_search_flag;
  /// Flag indicating the group's state in the binary search.
  SearchDirection   m_search_direction;
  /// Pivot that reached this node (used after the first big KLA step)
  color_type        m_pivot;

public:
  pscc_schudy_vertex_property(void)
    : m_cc(invalid_cc()),
      m_forward_level(std::numeric_limits<size_t>::max()),
      m_backward_level(std::numeric_limits<size_t>::max()),
      m_level(std::numeric_limits<size_t>::max()),
      m_group(),
      m_search_flag(TEMPORARY_NONPIVOT),
      m_search_direction(TOO_SMALL_FLAG),
      m_pivot(std::numeric_limits<size_t>::max())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Marks the node as reached from SOME node, not necessarily
  /// the pivot.
  /// @return True if node was previously unmarked, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool set_mark(size_t level)
  {
    if (m_level == std::numeric_limits<size_t>::max()) {
      m_level = level;
      return true;
    } else {
      return false;
    }
  }

  size_t get_mark(void) const
  { return m_level; }

  void unmark(void)
  { m_level = std::numeric_limits<size_t>::max(); }

  SearchDirection direction(void) const
  { return m_search_direction; }

  void direction(SearchDirection s)
  { m_search_direction = s; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Marks the node with a pivot and (if possible) updates the cc.
  /// If the node already has this mark, the function returns false, so
  /// that the traversal knows to stop.
  ///
  /// The forward boolean is used to denote whether this mark came from
  /// a forward or a backward traversal.
  /// @return True if mark is from forward traversal, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool set_level(color_type mark, bool forward, size_t level)
  {
    m_pivot = mark;
    if (forward) {
      if (m_forward_level == std::numeric_limits<size_t>::max()) {
        m_forward_level = level;
        if (m_backward_level != std::numeric_limits<size_t>::max()) {
          set_cc(mark);
        }
      } else {
        return false;
      }
    } else {
      if (m_backward_level == std::numeric_limits<size_t>::max()) {
        m_backward_level = level;
        if (m_forward_level != std::numeric_limits<size_t>::max()) {
          set_cc(mark);
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

  BinarySearchFlag search_flag(void) const
  { return m_search_flag; }

  void search_flag(BinarySearchFlag flag)
  { m_search_flag = flag; }

  bool is_pivot(void) const
  { return m_search_flag == TEMPORARY_NONPIVOT; }

  color_type get_acting_pivot(void) const
  { return m_pivot; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the group's descriptor and clears old data.
  //////////////////////////////////////////////////////////////////////
  void update_group(color_type group)
  {
    size_t invalid = std::numeric_limits<size_t>::max();
    //update the group
    if (m_cc == invalid_cc()) {
      m_group = mark_signature(group,
        (char)((        (m_level == invalid) ? 0 : 1) |
               ((m_forward_level == invalid) ? 0 : 2))
      );
      m_search_direction = TOO_BIG_FLAG;
      m_search_flag = TEMPORARY_PIVOT;
    } else {
      m_group = mark_signature(invalid_cc(), 0);
      m_search_flag = LOCKED_NONPIVOT;
    }

    //update scc data
    m_backward_level = m_forward_level = m_level = invalid;
  }

  mark_signature get_group(void) const
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
  ///
  /// In the case of Schudy's algorithm, this information is just the group
  /// descriptor.
  //////////////////////////////////////////////////////////////////////
  mark_signature get_signature(void) const
  { return m_group; }

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
  { return invalid_cc(); }

  void define_type(typer& t)
  {
    t.member(m_cc);
    t.member(m_level);
    t.member(m_predecessors);
    t.member(m_forward_level);
    t.member(m_backward_level);
    t.member(m_group);
    t.member(m_search_flag);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Counter class that maintains information about each group.
/// The binary search can stop when m_reached < m_size/2 AND m_pivots == 1.
///
/// Note: Schudy's paper says to look for the smallest set of pivots such that
/// they reach at least half the graph. Instead, we look for the largest set of
/// pivots such that they reach no more than half the graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class pscc_schudy_mark_counter
{
private:
  size_t m_pivots;
  size_t m_reached;
  size_t m_size;
  algo_details::SearchDirection m_dir;

public:
  pscc_schudy_mark_counter(void)
    : m_pivots(0),
      m_reached(0),
      m_size(0),
      m_dir(TOO_SMALL_FLAG)
  { }

  pscc_schudy_mark_counter(bool pivot, bool reached)
    : m_pivots(pivot),
      m_reached(reached),
      m_size(1),
      m_dir(TOO_SMALL_FLAG)
  { }

  void count(void)
  { ++m_size; }

  void is_pivot(void)
  { ++m_pivots; ++m_size; }

  void is_reached(void)
  { ++m_reached; ++m_size; }

  void is_reached_pivot(void)
  { ++m_pivots; ++m_reached; ++m_size; }

  algo_details::SearchDirection direction(void) const
  { return m_dir; }

  bool evaluate(void)
  {
    if (2*m_reached > m_size) {
      m_dir = TOO_BIG_FLAG;
    } else if (m_pivots == 1) {
      m_dir = JUST_RIGHT_FLAG;
    } else {
      m_dir = TOO_SMALL_FLAG;
    }

    return m_dir == JUST_RIGHT_FLAG;
  }

  void update(pscc_schudy_mark_counter const& other)
  {
    m_pivots += other.m_pivots;
    m_reached += other.m_reached;
    m_size += other.m_size;
  }

  void define_type(typer& t)
  {
    t.member(m_pivots);
    t.member(m_reached);
    t.member(m_size);
    t.member(m_dir);
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref pscc_schudy_vertex_property.
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

  bool set_mark(size_t level)
  { return Accessor::invoke(&target_t::set_mark, level); }

  size_t get_mark(void) const
  { return Accessor::const_invoke(&target_t::get_mark); }

  void unmark(void)
  { Accessor::invoke(&target_t::unmark); }

  algo_details::SearchDirection direction(void) const
  { return Accessor::const_invoke(&target_t::direction); }

  void direction(algo_details::SearchDirection s)
  { Accessor::invoke(&target_t::direction, s); }

  //adds a mark, updating the cc, and returns whether the mark is new
  bool set_level(color_type mark, bool forward, size_t level)
  { return Accessor::invoke(&target_t::set_level, mark, forward, level); }

  size_t level(bool forward) const
  { return Accessor::const_invoke(&target_t::level, forward); }

  algo_details::BinarySearchFlag search_flag(void) const
  { return Accessor::const_invoke(&target_t::search_flag); }

  void search_flag(algo_details::BinarySearchFlag flag)
  { Accessor::invoke(&target_t::search_flag, flag); }

  bool is_pivot(void) const
  { return Accessor::const_invoke(&target_t::is_pivot); }

  color_type get_acting_pivot(void) const
  { return Accessor::const_invoke(&target_t::get_acting_pivot); }

  void update_group(color_type group)
  { Accessor::invoke(&target_t::update_group, group); }

  mark_signature get_group(void) const
  { return Accessor::const_invoke(&target_t::get_group); }

  void set_cc(color_type c)
  { return Accessor::invoke(&target_t::set_cc, c); }

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


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref pscc_schudy_mark_counter.
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

  void count(void)
  { Accessor::invoke(&target_t::count); }

  void is_pivot(void)
  { Accessor::invoke(&target_t::is_pivot); }

  void is_reached(void)
  { Accessor::invoke(&target_t::is_reached); }

  void is_reached_pivot(void)
  { Accessor::invoke(&target_t::is_reached_pivot); }

  algo_details::SearchDirection direction(void) const
  { return Accessor::const_invoke(&target_t::direction); }

  bool evaluate(void)
  { return Accessor::invoke(&target_t::evaluate); }
}; //struct proxy

namespace algo_details {


//////////////////////////////////////////////////////////////////////
/// @brief Does a traversal from any node that is flagged as either a
/// TEMPORARY_PIVOT or a LOCKED_PIVOT.
///
/// Any node that the traversal reaches is marked using
/// @ref property().mark().
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_schudy_marking_bfs
{
public:
  typedef bool result_type;

private:
  size_t      m_level;

public:
  pscc_schudy_marking_bfs(void)
    : m_level(std::numeric_limits<size_t>::max())
  { }

  pscc_schudy_marking_bfs(size_t level)
    : m_level(level)
  { }

  template <typename Vertex>
  result_type operator()(Vertex&& v) const
  {
    return v.property().set_mark(m_level);
  }

  void define_type(typer& t)
  {
    t.member(m_level);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Schudy's algorithm randomly permutes the nodes and then
/// performs a binary search, looking for the minimal prefix of the
/// nodes that reaches at least half of the graph.
///
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
{
public:
  typedef bool result_type;

  template <typename Vertex, typename GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    if (v.property().has_valid_cc() ||
        v.property().direction() == JUST_RIGHT_FLAG) {
      return false;
    }

    size_t level = graph_visitor.level();
    if (level == 1) {
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
          v.property().get_mark() == std::numeric_limits<size_t>::max()) {
        pscc_schudy_marking_bfs fw_bfs(level);
        fw_bfs(v);
        for (auto&& i2 : v) {
          graph_visitor.visit(i2.target(), fw_bfs);
        }
        return true;
      } else {
        return false;
      }

    } else {
      if (v.property().get_mark() == level-1) {
        pscc_schudy_marking_bfs fw_bfs(level);
        for (auto&& i2 : v) {
          graph_visitor.visit(i2.target(), fw_bfs);
        }
        return true;
      } else {
        return false;
      }
    }
  }
};


template <typename CountMap>
struct pscc_schudy_count_manager
{
  typedef CountMap result_type;

  template <typename Vertex>
  result_type operator()(Vertex v) const
  {
    CountMap result;

    if (v.property().has_valid_cc()) {
      return result;
    }

    bool pivot = v.property().is_pivot();
    bool marked =
      (v.property().get_mark() != std::numeric_limits<size_t>::max());
    typedef typename result_type::value_type value_type;
    result.insert(value_type(v.property().get_signature(),
                  pscc_schudy_mark_counter(pivot, marked)));
    return result;
  }
};


template <typename CountMap>
struct combine_count_map
{
  typedef CountMap result_type;

  template <typename MapRef0, typename MapRef1>
  result_type operator()(MapRef0&& lhs, MapRef1&& rhs) const
  {
    result_type res(lhs);
    result_type rhs_copy(rhs);

    for (auto&& count : rhs_copy) {
      auto res_count = res.find(count.first);
      if (res_count != res.end()) {
        res_count->second.update(count.second);
      } else {
        res.insert(count);
      }
    }

    return res;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Uses the @ref pscc_schudy_mark_counter information to determine
/// the search direction for each group. If any groups are not finished,
/// it returns false so that the binary search may continue.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_schudy_evaluate_groups
{
  typedef bool    result_type;

  template<typename Counter>
  result_type operator()(Counter&& c)
  { return c.second.evaluate(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Updates the search direction and removes the reached information
/// for groups that have not finished the search.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename CounterMap>
struct pscc_schudy_update_direction_and_unmark
{
private:
  CounterMap m_counters;

public:
  typedef void result_type;

  pscc_schudy_update_direction_and_unmark(CounterMap const& counters)
    : m_counters(counters)
  { }

  template<typename Vertex>
  void operator()(Vertex v)
  {
    if (v.property().has_valid_cc() ||
        v.property().direction() == JUST_RIGHT_FLAG) {
      return;
    }

    v.property().direction(m_counters[v.property().get_group()].direction());

    if (v.property().direction() != JUST_RIGHT_FLAG) {
      v.property().unmark();
    }
  }

  void define_type(typer& t)
  { t.member(m_counters); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Runs the binary search and selects the final pivot for each
/// group, using the subfunctions above. The number of groups is 2^depth,
/// since Schudy's algorithm basically guarantees an even division of
/// nodes on each iteration.
///
/// The algorithm works in two different ways; one does the work locally
/// and then compiles it, while the other does all work globally. The local
/// work is more efficient when the number of groups is small, but gets
/// worse as the number of groups increases.
/// @param g The input @ref graph_view.
/// @param depth The depth.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Graph>
void pscc_schudy_pivot_selector(Graph& g, size_t k, kla_params<Graph> const& p)
{
  typedef typename Graph::vertex_property                vertex_property;

  typedef typename vertex_property::mark_signature       map_key_type;
  typedef stapl::indexed_domain<size_t>                  map_domain;
  typedef pscc_schudy_mark_counter                       counter_map_value_type;
  typedef std::map<size_t, counter_map_value_type>       counter_map;


  while (true) {
    kla_paradigm(pscc_schudy_search_step(), pscc_schudy_marking_bfs(0), g, k,
                 p);
    counter_map counters =
      map_reduce(pscc_schudy_count_manager<counter_map>(),
                 combine_count_map<counter_map>(), g);

    size_t just_right_count(0);
    for (auto&& counter : counters) {
      just_right_count += counter.second.evaluate();
    }
    if (counters.size() != just_right_count) {
      map_func(pscc_schudy_update_direction_and_unmark<counter_map>(counters),
               g);
    } else {
      break;
    }
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Updates the group of each node (2*old_group + 0/1) and then
/// traverses from the one node marked TEMPORARY_PIVOT in each group.
///
/// This traversal sets the forward/backward flag, and not just the
/// mark boolean.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_schudy_bfs_start
{
public:
  typedef bool result_type;

  template <typename Vertex, typename GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    typedef typename std::decay<Vertex>::type::vertex_descriptor color_type;
    typedef pscc_async_bfs<color_type> bfs;
    typedef typename std::decay<Vertex>::type::property_type property_type;
    typedef typename property_type::reachables_iterator pred_iterator;

    if (v.property().level(true) == std::numeric_limits<size_t>::max() &&
        v.property().has_valid_cc()) {
      return 0;
    }

    size_t level = graph_visitor.level();
    if (level == 1) {
      if (v.property().is_pivot()) {
        color_type vd = v.descriptor();
        bfs fw_bfs(vd, true, level);
        fw_bfs(v);
        for (auto&& i2 : v) {
          graph_visitor.visit(i2.target(), fw_bfs);
        }
        bfs bw_bfs(vd, false, level);
        bw_bfs(v);
        property_type p = v.property();
        for (auto&& i2 : p) {
          graph_visitor.visit(i2, bw_bfs);
        }
        return true;
      } else {
        return false;
      }
    } else {
      bool result = false;
      if (v.property().level(true) == level-1) {
        bfs fw_bfs(v.property().get_acting_pivot(), true, level);
        for (auto&& i2 : v) {
          graph_visitor.visit(i2.target(), fw_bfs);
        }
        result = true;
      }

      if (v.property().level(false) == level-1) {
        property_type p = v.property();
        bfs bw_bfs(v.property().get_acting_pivot(), false, level);
        for (auto&& i2 : p) {
          graph_visitor.visit(i2, bw_bfs);
        }
        result = true;
      }

      return result;
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This functor selects one pivot from each group by attempting
/// to insert each vertex into a map using @ref pscc_single_map_insert.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename PivotMap>
struct pscc_schudy_find_groups
{
  typedef PivotMap result_type;

  template <typename Vertex>
  result_type operator()(Vertex v) const
  {
    PivotMap m;
    if (v.property().is_pivot()) {
      typedef typename PivotMap::value_type value_type;
      m.insert(value_type(v.property().get_group(), v.descriptor()));
    }
    return m;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This functor selects one pivot from each group by attempting
/// to insert each vertex into a map using @ref pscc_single_map_insert.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename PivotMap>
struct pscc_schudy_relabel_groups
{
private:
  PivotMap m_pivots;

public:
  typedef void result_type;

  pscc_schudy_relabel_groups(PivotMap const& pivots)
    : m_pivots(pivots)
  { }

  template <typename Vertex>
  void operator()(Vertex v) const
  {
    if (v.property().has_valid_cc()) {
      v.property().update_group(v.property().invalid_cc());
    } else {
      v.property().update_group(
        m_pivots.find(v.property().get_group())->second);
    }
  }

  void define_type(typer& t)
  { t.member(m_pivots); }
};

template <typename Result>
struct combine_pivot_map
{
  typedef Result result_type;

  template <typename MapRef0, typename MapRef1>
  result_type operator()(MapRef0&& lhs, MapRef1&& rhs) const
  {
    result_type res(lhs);
    result_type rhs_copy(rhs);
    for (auto&& pivot : rhs_copy) {
#ifndef STAPL_NDEBUG
      auto insert_res = res.insert(pivot);
      stapl_assert(insert_res.second, "pivot info was not unique");
#else
      res.insert(pivot);
#endif
    }
    return res;
  }
};
} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief This is an implementation of Schudy's MultiPivot SCC algorithm.
///
/// This algorithm executes a variant of Schudy's SCC algorithm on the input
/// graph. It first finds a pivot such that the pivot and some other nodes reach
/// at least half of the graph. It then tests reachability from the pivot and
/// divides the graph into the unreached and reached node sets.
/// @note This algorithm is O(log n) times slower than SCCMulti
/// (@ref pscc.hpp), and so should not be used other than for comparison.
/// @param g The input @ref graph_view.
/// @param k The maximum amount of asynchrony for kla.
/// @param degree_percentage The threshold for a vertex's degree after which
/// it will be considered a hub (given as a percentage of total graph size).
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
int pscc_schudy(GraphView& g, size_t k, float degree_percentage = 0.1)
{
  using namespace algo_details;

  typedef typename GraphView::vertex_descriptor vertex_descriptor;
  typedef pscc_schudy_vertex_property<vertex_descriptor> vertex_property;
  typedef graph<DIRECTED, MULTIEDGES, vertex_property> graph_type;
  typedef graph_view<graph_type> gview_type;

  typedef typename vertex_property::mark_signature        map_key_type;
  typedef stapl::indexed_domain<size_t>                   map_domain;
  typedef vertex_descriptor                               pivot_map_value_type;
  typedef std::map<size_t, pivot_map_value_type>          pivot_map;

  size_t gsz = g.size();

  kla_params<gview_type> p;
  p.avoid_hubs = true;
  p.degree_threshold = degree_percentage*gsz;

  graph_type gc(gsz);
  gview_type gcv(gc);
  copy_graph_struct_w_preds(g, gcv);

  srand48((get_location_id()+1)*time(0));

  int recursion_depth=0;
  while (true) {
    ++recursion_depth;
    map_func(pscc_trim_forward(), gcv, make_repeat_view(gcv));
    map_func(pscc_trim_backward(), gcv, make_repeat_view(gcv));

    if (count_free_nodes(gcv) == 0) {
      break;
    }

    pscc_schudy_pivot_selector(gcv, k, p);

    algo_details::pscc_schudy_bfs_start start;
    kla_paradigm(start, pscc_async_bfs<vertex_descriptor>(),
                 gcv, k, p);

    pivot_map pivots =
      map_reduce(algo_details::pscc_schudy_find_groups<pivot_map>(),
                 algo_details::combine_pivot_map<pivot_map>(), gcv);
    map_func(algo_details::pscc_schudy_relabel_groups<pivot_map>(pivots), gcv);

    map_func(pscc_separate_disjoint_return_uncolored(),
      gcv, make_repeat_view(gcv));
  }

  map_func(copy_sccs_wf(), gcv, g);
  return recursion_depth;
}

} // namespace stapl

#endif
