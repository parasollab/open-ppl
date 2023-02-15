/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/skeletons/explicit/map_prototype.hpp>
#include <stapl/paragraph/factory_wf.hpp>

#include "../shared/nested_factories.hpp"
#include "../../test_report.hpp"

using namespace stapl;

typedef counter<stapl::default_timer> counter_t;

struct loc_reporter
  : public p_object
{
  unsigned int m_location;

  loc_reporter(void)
    : m_location(get_location_id())
  { }
};


struct print1
{
  loc_reporter* m_loc_report;

  typedef void result_type;

  print1(loc_reporter* loc_report)
    : m_loc_report(loc_report)
  { }

  template <typename T>
  void operator()(T const& t) const
  {
    std::stringstream out;
    out << "(" << t << ", " << m_loc_report->m_location << ") ";
    std::cout << out.str() << " ";
  }

  void define_type(typer& t)
  {
    t.member(m_loc_report);
  }
};


struct print2
{
  loc_reporter* m_loc_report;

  typedef void result_type;

  print2(loc_reporter* loc_report)
    : m_loc_report(loc_report)
  { }

  template <typename View>
  void operator()(View const& v) const
  {
    stapl::map_func(print1(m_loc_report), v);

    std::cout << std::endl;
  }

  void define_type(typer& t)
  {
    t.member(m_loc_report);
  }
};


struct min_row
{
  typedef int result_type;

  template <typename View>
  result_type operator()(View const& v)
  {
    return min_element(v);
  }
};


template<typename View>
double global_nested_min(View& vw, int val, std::string msg)
{
  counter_t timer;

  timer.reset();
  timer.start();

  int min_value = nested_reduce(min<int>(), vw);

  double time = timer.stop();

  STAPL_TEST_REPORT(min_value == val, msg);

  return time;
};


template<typename View>
double global_min(View& vw, int val, std::string msg)
{
  loc_reporter lr;

  if (vw.size() < 20)
    stapl::map_func(print2(&lr), vw);

  counter_t timer;

  timer.reset();
  timer.start();

  int min_value = stapl::map_reduce(min_row(), min<int>(), vw);

  double time = timer.stop();

  STAPL_TEST_REPORT(min_value == val, msg);

  return time;
}


struct gen_spec_wf
{
  size_t m_outer_size;
  size_t m_nested_size;

  typedef stapl::distribution_spec<> result_type;

  gen_spec_wf(size_t outer_size, size_t nested_size)
    : m_outer_size(outer_size), m_nested_size(nested_size)
  { }

  result_type operator()(std::vector<size_t> const& index) const
  {
    if (index.empty())
    {
      // return the distribution of the outer container.
      return stapl::cyclic(m_outer_size);
    }
    else if (index.back()%2 == 0)
    {
      // Even indices are balanced.
      return balance(m_nested_size);
    }
    else
    {
      // Odd indices are block-cyclic.
      size_t block_size = m_nested_size / 4;
      return block_cyclic(m_nested_size, block_size);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_outer_size);
    t.member(m_nested_size);
  }
};


struct init_wf
{
  typedef void result_type;

  template <typename View, typename Index>
  result_type operator()(View const& vw, Index i) const
  { generate(vw, sequence<int>(i*10+4, 1)); }
};


stapl::exit_code stapl_main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr << "usage: exe n m" << std::endl;
    exit(1);
  }

  const size_t n = atoi(argv[1]);
  const size_t m = atoi(argv[2]);

  typedef array<int,
                view_based_partition<distribution_spec<>>,
                view_based_mapper<distribution_spec<>>>    in_arr_tp;
  typedef array<in_arr_tp,
                view_based_partition<distribution_spec<>>,
                view_based_mapper<distribution_spec<>>>    spec_array_type;


  // Work function to generate distributions of the containers.
  // The outer container will be cyclic across all locations.
  gen_spec_wf gen_wf(n, m);

  // Create the composed specification
  composed_dist_spec comp_spec(gen_wf);

  // Construct the composed container.
  spec_array_type s(comp_spec);
  array_view<spec_array_type> vs(s);

  // Initialize the containers.
  map_func(init_wf(), vs, counting_view<int>(n));

  // Find the minimum element.
  global_min(vs,4,
             "Testing min_row (array<array<T>> composed distribution spec)");

  std::vector<distribution_spec<>> dists(2);
  dists[0] = stapl::cyclic(n);
  dists[1] = stapl::block_cyclic(m, m/(2*stapl::get_num_locations()));
  spec_array_type t(dists);
  array_view<spec_array_type> vt(t);

  // Initialize the containers.
  map_func(init_wf(), vt, counting_view<int>(n));

  // Find the minimum element.
  global_min(vt, 4,
    "Testing min_row (array<array<T>> uniform composed distribution spec)");

  array<int> c0(m);

  typedef array<array<int>> array_type;

  // Inner container distributed across the system
  array_type c(n,c0);

  array_view<array_type> vc(c);

  // Initialize the containers.
  map_func(init_wf(), vc, counting_view<int>(n));

  global_nested_min(vc, 4,
                    "Testing min_row (array<array<T>> inner distributed)");

  // Inner in one location (here)
  array_type d(n, c0, policy::here());

  array_view<array_type> vd(d);

  // Initialize the containers.
  map_func(init_wf(), vd, counting_view<int>(n));

  global_min(vd, 4, "Testing min_row (array<array<T>> inner in one location)");

  return EXIT_SUCCESS;
}
