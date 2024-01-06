/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_GAP_SOURCES_HPP
#define STAPL_BENCHMARKS_GAP_SOURCES_HPP

#include <stapl/algorithms/algorithm.hpp>

#include <random>

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Work function to create a vector with vertices' IDs, if
///        the vertex has any outgoing edges.
/// @tparam VD Vertex descriptor type
//////////////////////////////////////////////////////////////////////
template<typename VD>
struct select_source_map
{
  using result_type = std::vector<VD>;

  template<typename Vertex>
  result_type operator()(Vertex&& v) const
  {
    return v.size() > 0 ? result_type{1, v.descriptor()} : result_type{};
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function to combine vectors up to a limit.
/// @tparam VD Vertex descriptor type
//////////////////////////////////////////////////////////////////////
template<typename VD>
class select_source_choose
{
  std::size_t m_limit;
public:

  using result_type = std::vector<VD>;

  select_source_choose(std::size_t limit)
    : m_limit(limit)
  { }

  template<typename U, typename V>
  result_type operator()(U&& u, V&& v) const
  {
    result_type u_vec{u.begin(), u.end()};
    result_type v_vec{v.begin(), v.end()};
    result_type out;

    std::sort(u_vec.begin(), u_vec.end());
    std::sort(v_vec.begin(), v_vec.end());
    std::merge(u_vec.begin(), u_vec.end(), v_vec.begin(), v_vec.end(),
      std::back_inserter(out));

    out.erase(std::unique(out.begin(), out.end()), out.end());

    if (out.size() > m_limit)
      out.erase(out.begin()+m_limit, out.end());

    return out;
  }

  void define_type(typer& t)
  {
    t.member(m_limit);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Return the descriptors for vertices to use as sources for
///        benchmarks. This will only select
///        vertices that have at least 1 outgoing edge.
///
/// @param g The input graph
/// @param num_sources The number of sources to select
//////////////////////////////////////////////////////////////////////
template<typename G>
std::vector<typename G::vertex_descriptor>
select_sources(G const& g, std::size_t num_sources)
{
  using vertex_descriptor = typename G::vertex_descriptor;

  auto sources = map_reduce(
    select_source_map<vertex_descriptor>{},
    select_source_choose<vertex_descriptor>{num_sources}, g
  );

  if (sources.size() < num_sources)
    stapl::abort("Not enough connected vertices to use as sources");

  return sources;
}


#endif
