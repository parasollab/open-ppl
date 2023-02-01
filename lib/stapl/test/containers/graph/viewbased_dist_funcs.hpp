/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_TEST_CONTAINERS_GRAPH_VIEWBASED_DIST_FUNCS_HPP
#define STAPL_TEST_CONTAINERS_GRAPH_VIEWBASED_DIST_FUNCS_HPP

#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/numeric.hpp>
#include <stapl/runtime/counter/default_counters.hpp>
#include "../shared/viewbased_dist_funcs.hpp"

typedef stapl::counter<stapl::default_timer> counter_t;


struct get_property
{
  typedef long int result_type;

  template <typename Vertex>
  result_type operator()(Vertex v)
  { return v.property(); }
};


struct set_property
{
  typedef void result_type;

  template <typename Vertex, typename Value>
  result_type operator()(Vertex v, Value val)
  { v.property() = val; }
};


template <typename Container>
std::tuple<double, double, bool>
compute(Container& c, std::stringstream& o, bool default_value = false)
{
  stapl::graph_view<Container> cv(c);
  long int n = c.size();

  // if list constructed with a default value, verify correctness of
  // constructor
  bool def_ctor = !default_value;

  if (default_value)
  {
    typename Container::vertex_property res =
      stapl::map_reduce(get_property(), stapl::plus<long int>(), cv);
    def_ctor = res == n*99 ? true : false;
  }

  counter_t gen_timer, acc_timer;
  gen_timer.reset();
  acc_timer.reset();

  gen_timer.start();
  stapl::map_func(set_property(), cv, stapl::counting_view<long int>(n));
  gen_timer.stop();

  acc_timer.start();
  typename Container::vertex_property res =
    stapl::map_reduce(get_property(), stapl::plus<long int>(), cv);
  acc_timer.stop();

  // check that all locations received the same value.
  bool all_eq = all_locs_equal()(res);

  // check that the value received is correct.
  if (!def_ctor || !all_eq || res != (n-1)*n/2)
  {
    o << "Status: FAIL\n";
    o << "Note: Error in computation. Expected " << (n-1)*n/2
      << " Computed " << res << " All loc equal " << all_eq
      << "Default value constructor " << def_ctor << "\n";
    return std::make_tuple(gen_timer.value(), acc_timer.value(), false);
  }
  return std::make_tuple(gen_timer.value(), acc_timer.value(), true);
}


struct graph_mid_init
{
  unsigned int m_outer_size;
  unsigned int m_mid_size;
  unsigned int m_inner_size;
  unsigned int m_outer_index;

  typedef void result_type;

  graph_mid_init(unsigned int outer_size, unsigned int mid_size,
                 unsigned int inner_size, unsigned int outer_index)
    : m_outer_size(outer_size), m_mid_size(mid_size), m_inner_size(inner_size),
      m_outer_index(outer_index)
  { }

  template <typename View, typename Index>
  void operator()(View vw, Index i)
  {
    long int start = m_outer_index*m_mid_size*m_inner_size + i*m_inner_size;
    stapl::map_func(set_property(), vw.property(),
                    stapl::counting_view<long int>(m_inner_size, start));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_outer_size);
    t.member(m_mid_size);
    t.member(m_inner_size);
    t.member(m_outer_index);
  }
};


struct graph_outer_init
{
  unsigned int m_outer_size;
  unsigned int m_mid_size;
  unsigned int m_inner_size;

  typedef void result_type;

  graph_outer_init(unsigned int outer_size, unsigned int mid_size,
                   unsigned int inner_size)
    : m_outer_size(outer_size), m_mid_size(mid_size), m_inner_size(inner_size)
  { }

  template <typename View, typename Index>
  void operator()(View vw, Index i)
  {
    unsigned int ndx = i;

    map_func(graph_mid_init(m_outer_size, m_mid_size, m_inner_size, ndx),
             vw.property(),
             stapl::counting_view<unsigned int>(m_mid_size));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_outer_size);
    t.member(m_mid_size);
    t.member(m_inner_size);
  }
};


struct graph_mid_sum
{
  typedef long int result_type;

  template <typename View>
  result_type operator()(View vw)
  {
    return stapl::map_reduce(get_property(), stapl::plus<long int>(),
             vw.property());
  };
};


struct graph_outer_sum
{
  typedef long int result_type;

  template <typename View>
  result_type operator()(View vw)
  {
    return stapl::map_reduce(graph_mid_sum(), stapl::plus<long int>(),
             vw.property());
  };
};


template <typename Container>
std::tuple<double, double, bool>
compute_composed(Container& c, std::stringstream& o, unsigned int n1,
                 unsigned int n2, unsigned int n3)
{
  stapl::graph_view<Container> cv(c);

  counter_t gen_timer, acc_timer;
  gen_timer.reset();
  acc_timer.reset();

  gen_timer.start();
  stapl::map_func(graph_outer_init(n1, n2, n3), cv,
                  stapl::counting_view<long int>(n1));
  gen_timer.stop();

  acc_timer.start();
  long int res =
    stapl::map_reduce(graph_outer_sum(), stapl::plus<long int>(), cv);
  acc_timer.stop();

  // check that all locations received the same value.
  bool all_eq = all_locs_equal()(res);

  // check that the value received is correct.
  if (!all_eq || res != (n1*n2*n3-1)*n1*n2*n3/2)
  {
    o << "Status: FAIL\n";
    o << "Note: Error in computation. Expected " << (n1*n2*n3-1)*n1*n2*n3/2
      << " Computed " << res << " All loc equal " << all_eq << "\n";
    return std::make_tuple(gen_timer.value(), acc_timer.value(), false);
  }
  return std::make_tuple(gen_timer.value(), acc_timer.value(), true);
}
#endif // STAPL_TEST_CONTAINERS_GRAPH_VIEWBASED_DIST_FUNCS_HPP
