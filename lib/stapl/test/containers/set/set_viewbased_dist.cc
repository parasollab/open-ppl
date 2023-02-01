/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/algorithms/algorithm.hpp>

#include <stapl/containers/set/set.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/views/system_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/skeletons/utility/tags.hpp>

#include "../../test_report.hpp"

using namespace stapl;

template <typename Set>
void compute(size_t n, Set& m)
{
  typedef Set                              container_type;
  typedef stapl::set_view<container_type>  view_type;

  location_type lid = get_location_id();
  location_type nlocs = get_num_locations();

  size_t num_big_blocks = n % nlocs;
  size_t block = n / nlocs + (lid < num_big_blocks ? 1 : 0);
  size_t start = block * lid + (lid >= num_big_blocks ? num_big_blocks : 0);
  size_t end   = start + block-1;

  for (size_t i = start; i < end+1; ++i)
    m.insert(i);
  rmi_fence();
  STAPL_TEST_REPORT((m.size() == n), "Testing insert");

  char y = m[n/2];
  char x = m.apply_get(n/2,
             stapl::identity<typename container_type::value_type>());
  stapl::rmi_fence();
  STAPL_TEST_REPORT((x==y && x==char(n/2 % std::numeric_limits<char>::max())),
                    "Testing apply_get and operator[]");

  view_type v(m);
  size_t xx = stapl::map_reduce(stapl::identity<size_t>(),
                                stapl::plus<size_t>(), v);
  STAPL_TEST_REPORT((xx == n*(n-1)/2), "Testing map_reduce over stapl::map");
  stapl::rmi_fence();
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  const size_t n = atoi(argv[1]);

  typedef stapl::distribution_spec<>                       distribution_spec;
  typedef stapl::view_based_partition<distribution_spec> vb_partition;
  typedef stapl::view_based_mapper<distribution_spec>    vb_mapping;
  typedef stapl::set<size_t, stapl::less<size_t>,
                     vb_partition, vb_mapping>           container_type;
  typedef stapl::set_view<container_type>                view_type;

  distribution_spec bc_spec = stapl::block_cyclic(n, 2);
  container_type m_bc(bc_spec);
  compute(n, m_bc);

  distribution_spec bl_spec = stapl::block(n, 2);
  container_type m_bl(bl_spec);
  compute(n, m_bl);

  return EXIT_SUCCESS;
}
