/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/algorithms/algorithm.hpp>

#include <stapl/containers/map/map.hpp>

#include <stapl/views/counting_view.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/skeletons/utility/tags.hpp>

#include "../../test_report.hpp"

using namespace stapl;

struct mapwf
{
  typedef size_t result_type;

  template <typename U>
  result_type operator()(U const& x)
  {
    return x.second;
  }
};


struct map_assign_wf
{
  typedef void result_type;

  template<typename View, typename Reference>
  result_type operator()(View v, Reference elem)
  {
    v[elem] = elem;
  }
};


struct map_equal_wf
{
  typedef bool result_type;

  template<typename Reference>
  result_type operator()(Reference elem)
  {
    return elem.first == elem.second;
  }
};


struct coarse_map_equal_wf
{
  typedef bool result_type;

  template<typename View>
  result_type operator()(View v)
  {
    size_t idx            = v.domain().first();
    const size_t last_idx = v.domain().last();

    for (; idx != last_idx; ++idx)
      if (v[idx] != idx)
        return false;

    // else
    return true;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  const size_t n = atoi(argv[1]);

  typedef stapl::map<size_t, char> container_type;
  typedef indexed_domain<size_t>   domain_type;
  typedef stapl::map_view<container_type> view_type;

  domain_type dom(0, n-1);

  container_type m(dom);
  STAPL_TEST_REPORT(true, "Testing map(domain) constructor");

  size_t block = n / get_num_locations();
  size_t start = block * get_location_id();
  size_t end   = start + block;

  for (size_t i = start; i < end; ++i)
    m.insert(i, static_cast<char>(i % std::numeric_limits<char>::max()));
  rmi_fence();
  STAPL_TEST_REPORT((m.size() == n), "Testing insert");

  char y = m[n/2];
  char x = m.apply_get(n/2,
                       stapl::identity<container_type::value_type>()).second;
  stapl::rmi_fence();
  STAPL_TEST_REPORT((x==y && x==char(n/2 % std::numeric_limits<char>::max())),
                    "Testing apply_get and operator[]");

  view_type v(m);
  size_t xx = stapl::map_reduce(mapwf(), stapl::plus<size_t>(), v);
  STAPL_TEST_REPORT((xx == n*(n-1)/2), "Testing map_reduce over stapl::map");

  typedef stapl::map<size_t, size_t>       container1_type;
  typedef stapl::map_view<container1_type> view1_type;

  container1_type m1(dom);
  view1_type      v1(m1);

  map_func(map_assign_wf(), make_repeat_view(v1), counting_view<size_t>(n));

  bool passed1 = map_reduce(map_equal_wf(), stapl::logical_and<bool>(), v1);
  bool passed2 = map_reduce<skeletons::tags::with_coarsened_wf>
                   (coarse_map_equal_wf(), stapl::logical_and<bool>(), v1);

  STAPL_TEST_REPORT(passed1 && passed2,
                    "Testing map/map_view operator[] with a simple algorithm");

  return EXIT_SUCCESS;
}
