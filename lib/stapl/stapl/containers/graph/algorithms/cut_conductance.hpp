/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_CUT_CONDUCTANCE_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_CUT_CONDUCTANCE_HPP

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/containers/array/array.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief If Membership is true, returns the degree of a vertex if it
///  belongs to the given cut, or returns zero if it does not belong to the cut.
/// If Membership is false, returns the degree of a vertex if it does
/// not belong to the given cut, or returns zero if it does belong to the cut.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename MemberMap, bool Membership>
class membership
{
  typedef typename MemberMap::value_type value_type;

  MemberMap  m_member_map;
  value_type m_member;

public:
  membership(MemberMap const& member_map, value_type const& member)
    : m_member_map(member_map), m_member(member)
  { }

  typedef std::size_t result_type;

  template<typename VertexRef>
  std::size_t operator()(VertexRef v)
  {
    if (Membership)
      return m_member_map.get(v) == m_member ? v.size() : 0;
    else
      return m_member_map.get(v) != m_member ? v.size() : 0;
  }

  void define_type(typer& t)
  {
    t.member(m_member);
    t.member(m_member_map);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Increments a number specifying the cross-edge count.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct increment
{
  template<typename T>
  void operator()(T& x) const
  {
    ++x;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Increments the cross-edge count for a vertex if the edge
/// used to reach this vertex was a cross-edge.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename T, typename MemberMap, typename CrossMap>
class update_cross
{
  MemberMap m_member_map;
  CrossMap  m_cross_map;
  T         m_member;

public:
  typedef void result_type;

  update_cross(T const& member, MemberMap const& member_map,
               CrossMap const& cross_map)
    : m_member_map(member_map), m_cross_map(cross_map), m_member(member)
  { }

  template<typename VertexRef>
  void operator()(VertexRef v)
  {
    if (m_member_map.get(v) != m_member)
      m_cross_map.apply(v, increment());
  }

  void define_type(typer& t)
  {
    t.member(m_member);
    t.member(m_member_map);
    t.member(m_cross_map);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief If a vertex is part of a cut, this adds tasks to check if its
///   neighbors are also part of the same cut.
/// @ingroup pgraphAlgoDetails
/// @todo view data member should be passed to paragraph via repeat view.
//////////////////////////////////////////////////////////////////////
template<typename View, typename MemberMap, typename CrossMap>
class conduct_cross_edges
  : public dynamic_wf
{
  typedef typename MemberMap::value_type member_type;

  View const*   m_view_ptr;
  MemberMap     m_member_map;
  CrossMap      m_cross_map;
  member_type   m_member;

public:
  typedef void result_type;

  conduct_cross_edges(View const& view, MemberMap const& member_map,
                      CrossMap const& cross_map, member_type const& member)
    : m_view_ptr(&view), m_member_map(member_map), m_cross_map(cross_map),
      m_member(member)
  { }

  template<typename TGV, typename VertexRef>
  void operator()(TGV tgv, VertexRef v)
  {
    typedef update_cross<member_type, MemberMap, CrossMap> wf_type;

    if (m_member_map.get(v) == m_member)
    {
      typename VertexRef::adj_edge_iterator it = v.begin();
      typename VertexRef::adj_edge_iterator eit = v.end();

      wf_type wf(m_member, m_member_map, m_cross_map);

      for (; it != eit; ++it)
        tgv.add_task(wf, localize_ref(*m_view_ptr, (*it).target()));
    }
  }

  void define_type(typer& t)
  {
    t.member(m_member_map);
    t.member(m_cross_map);
    t.member(m_view_ptr);
    t.member(m_member);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns the number of cross-edges of this vertex.
/// A cross edge is such that its source and target are in two
/// different cuts.
///
/// m_cross_map is vertex property map storing the number of cross-edges
/// incident on a vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename CrossMap>
class get_cross
{
  CrossMap m_cross_map;

public:
  get_cross(CrossMap const& cross_map)
    : m_cross_map(cross_map)
  { }

  typedef std::size_t result_type;

  template<typename VertexRef>
  result_type operator()(VertexRef v)
  {
    return m_cross_map.get(v);
  }

  void define_type(typer& t)
  {
    t.member(m_cross_map);
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Compute the conductance of a cut defined to be the set of vertices
/// for which member(v) = id.
/// @param vw The input @ref graph_view.
/// @param member_map Input vertex property map specifying the cut ids of
/// vertices.
/// @param id The cut id for which to compute conductance.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename View, typename MemberMap>
double cut_conductance(View const& vw, MemberMap const& member_map,
                       typename MemberMap::value_type const& id)
{
  // create temporary map for cross edges
  typedef static_array<std::size_t> cross_storage_type;
  typedef array_view<cross_storage_type> cross_view_type;
  typedef graph_external_property_map<View, std::size_t,
                                      cross_view_type> cross_map_type;

  cross_storage_type cross_cont(vw.size());
  cross_view_type cross_vw(cross_cont);
  cross_map_type cross(vw, cross_vw);

  // call signature expecting cross property map
  return cut_conductance(vw, member_map, cross, id);
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute the conductance of a cut defined to be the set of vertices
/// for which member(v) = id.
/// @param vw The input @ref graph_view.
/// @param member_map Input vertex property map specifying the cut ids of
/// vertices.
/// @param cross_map Temporary vertex property map to store the number of
/// cross edges
/// @param id The cut id for which to compute conductance.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename View, typename MemberMap, typename CrossMap>
double cut_conductance(View const& vw, MemberMap const& member_map,
                       CrossMap const& cross_map,
                       typename MemberMap::value_type const& id)
{
  using std::size_t;

  // compute out degree of vertices in the set
  size_t in = map_reduce(detail::membership<MemberMap, true>(member_map, id),
    stapl::plus<size_t>(), vw);

  // compute out degree of vertices not in the set
  size_t out = map_reduce(detail::membership<MemberMap, false>(member_map, id),
    stapl::plus<size_t>(), vw);

  // push membership status to outgoing edges
  map_func(detail::conduct_cross_edges<
             View, MemberMap, CrossMap
           >(vw, member_map, cross_map, id), vw);

  // compute number of edges (v, u) s.t. v is in the set and u is not in the set
  size_t cross = map_reduce(detail::get_cross<CrossMap>(cross_map),
    stapl::plus<size_t>(), vw);

  double m = std::min(in, out);

  if (cross == 0 && m == 0)
    return 0.0;

  if (m == 0)
    return std::numeric_limits<double>::max();

  return cross / m;
}

} // namespace stapl

#endif
