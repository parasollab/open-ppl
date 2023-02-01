/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_TEST_CONTAINERS_GRAPH_GENERATORS_UTILS_HPP
#define STAPL_TEST_CONTAINERS_GRAPH_GENERATORS_UTILS_HPP

#include <stapl/algorithms/algorithm.hpp>
#include <boost/unordered_map.hpp>

struct degree_distribution_wf
{
  typedef boost::unordered_map<std::size_t, std::size_t> result_type;

  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    result_type dist;
    dist[v.size()] += 1;

    return dist;
  }
};


struct accumulate_degree
{
  typedef boost::unordered_map<std::size_t, std::size_t> result_type;

  template<typename T, typename U>
  result_type operator()(T a, U b)
  {
    result_type x = a;
    result_type y = b;

    for (result_type::iterator it = y.begin(); it != y.end(); ++it)
      x[it->first] += it->second;

    return x;
  }
};


template<typename View>
boost::unordered_map<std::size_t, std::size_t>
degree_distribution(View const& vw)
{
  return stapl::map_reduce(degree_distribution_wf(), accumulate_degree(), vw);
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function that checks if a vertex's degree is a specific value
//////////////////////////////////////////////////////////////////////
class degree_equal_to
{
  std::size_t m_n;

public:
  using result_type = bool;

  degree_equal_to(std::size_t n)
    : m_n(n)
  { }

  template<typename T>
  bool operator()(T&& vertex)
  {
    return vertex.size() == m_n;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_n);
  }
};

#endif
