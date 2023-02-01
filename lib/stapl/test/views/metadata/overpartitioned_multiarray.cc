/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/numeric.hpp>

#include <stapl/array.hpp>

#include "../../test_report.hpp"
#include "utils.hpp"

using namespace stapl;


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 4)
  {
    std::cout<< "usage: exe n m r" <<std::endl;
    exit(1);
  }

  size_t n = atol(argv[1]);
  size_t m = atol(argv[2]);
  size_t r = atol(argv[3]);

  // create different distribution for second multiarray
  typedef indexed_domain<size_t>                     linear_domain_type;
  typedef balanced_partition<linear_domain_type>     balanced_type;
  typedef default_traversal<3>::type                 traversal_type;
  typedef nd_partition<
            stapl::tuple<balanced_type, balanced_type, balanced_type>,
            traversal_type>                          partition_type;
  typedef multiarray<3, int, traversal_type,
          partition_type> multiarray_type;
  typedef multiarray_view<
            multiarray_type
          > view_type;

  // overpartition this multiarray into p^3 parts
  balanced_type p0(linear_domain_type(0, n-1, true), get_num_locations());
  balanced_type p1(linear_domain_type(0, m-1, true), get_num_locations());
  balanced_type p2(linear_domain_type(0, r-1, true), get_num_locations());
  partition_type part(p0, p1, p2);

  multiarray_type d(part);
  view_type w(d);

  auto c = coarsen_views(w);
  auto& coarsened = get<0>(c);

  bool passed = coarsening_covers_space(w, coarsened);
  STAPL_TEST_REPORT(passed, "Testing overpartitioned 3d multiarray");

  return EXIT_SUCCESS;
}
