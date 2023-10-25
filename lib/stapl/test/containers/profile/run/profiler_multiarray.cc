/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/multiarray/multiarray.hpp>

#include "../p_container_profiler.hpp"
#include "../profiler_util.h"
#include "../value_type_util.h"
#include "matrix_index_generator.hpp"

using namespace stapl;

exit_code stapl_main(int argc, char** argv)
{
  if (argc < 3) {
    if (get_location_id() == 0)
      std::cout << "Usage: exe n m\n";
    exit(1);
  }

  size_t n = atoi(argv[1])*get_num_locations();
  size_t m = atoi(argv[2])*get_num_locations();
  size_t blockn = n / get_num_locations();
  size_t blockm = m / get_num_locations();
  size_t premote(0);
  size_t next_only(0);

  if (get_location_id() == 0) {
    std::cout << "n: " << n;
    std::cout << "\nm: " << m;
    std::cout << "\nblockn: " << blockn;
    std::cout << "\nblockm: " << blockm;
    std::cout << "\npremote: " << premote;
    std::cout << "\nnext_only: " << next_only << "\n\n";
  }

  typedef int                                        value_type;
  typedef indexed_domain<size_t>                     vector_domain_type;
  typedef balanced_partition<vector_domain_type>     balanced_type;
  typedef default_traversal<2>::type                 traversal_type;
  typedef nd_partition<
            stapl::tuple<balanced_type, balanced_type>,
            traversal_type>                          partition_type;
  typedef tuple<size_t, size_t> gid_type;
  typedef multiarray<2, value_type, traversal_type,
                     partition_type>                 multiarray_type;

  balanced_type p0(vector_domain_type(0, n-1), get_num_locations());
  balanced_type p1(vector_domain_type(0, m-1), get_num_locations());
  partition_type part(p0, p1);

  multiarray_type c(gid_type(n, m), part);

  std::vector<gid_type> indices;

  matrix_index_generator::generate(indices, blockn*get_location_id(),
                            blockm*get_location_id(), blockn, blockm,
   premote, next_only);

  rmi_fence();

  set_element_profiler<multiarray_type, counter<default_timer>, gid_type>
    sep("multiarray<int>", &c, indices, argc, argv);
  sep.collect_profile();
  sep.report();

  get_element_profiler<multiarray_type, counter<default_timer>, gid_type>
    gep("multiarray<int>", &c, indices, argc, argv);
  gep.collect_profile();
  gep.report();

  get_element_split_profiler<multiarray_type, counter<default_timer>, gid_type>
    gesp("multiarray<int>", &c, indices, argc, argv);
  gesp.collect_profile();
  gesp.report();

  operator_square_bracket_lhs_profiler<multiarray_type, counter<default_timer>,
                                       gid_type>
    osblp("multiarray<int>", &c, indices, argc, argv);
  osblp.collect_profile();
  osblp.report();

  operator_square_bracket_rhs_profiler<multiarray_type, counter<default_timer>,
                                       gid_type>
    osbrp("multiarray<int>", &c, indices, argc, argv);
  osbrp.collect_profile();
  osbrp.report();

  return EXIT_SUCCESS;
}
