/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <utility>
#include <vector>

#include <stapl/containers/graph/multigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/connected_components.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/array/static_array.hpp>

#include "test_util.h"

using namespace stapl;

struct size_wf
{
  typedef size_t result_type;
  template <typename T>
  result_type operator() (T t)
  {
    return t.size();
  }
};


template <typename G>
bool auxillary_algorithms(G& g)
{
  // make sure 0, 1, 4, 5, and last are part of the same CC.
  if ( !(is_same_cc(g, 0, 1) &&
         is_same_cc(g, 0, int(g.size()-1)) &&
         is_same_cc(g, 0, 4) &&
         is_same_cc(g, 0, 5) &&
         is_same_cc(g, 0, 1) ) ) {
    return false;
  }

  // make sure none of the other vertices are reported as part of some CC:
  for (size_t i=2; i<g.size()-1; ++i) {
    if (i != 4 && i != 5) {
      static_array<std::vector<size_t> > out_cont(get_num_locations());
      array_view<static_array<std::vector<size_t> > > output_vw(out_cont);
      cc_stats(g, i, output_vw);
      if (stapl::map_reduce(size_wf(), stapl::plus<size_t>(), output_vw)
          != size_t(0))
        return false;
    }
  }

  // everything seems okay...
  return true;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " n" << std::endl;
    exit(1);
  }

  size_t n = atol(argv[1]);

  typedef multigraph<properties::cc_property> graph_type;
  typedef graph_view<graph_type> view_type;

  graph_type p(n);
  view_type vw(p);

  if (get_location_id() == 0) {
    p.add_edge_async(0, 1);
    p.add_edge_async(0, n-1);

    p.add_edge_async(0, 4);
    p.add_edge_async(4, 5);
  }
  rmi_fence();

  one_print("Testing Connected Components...\t\t");
  connected_components(vw);

  bool failed = auxillary_algorithms(vw);

  one_print(!failed);

  return EXIT_SUCCESS;
}
