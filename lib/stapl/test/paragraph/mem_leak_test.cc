/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/paragraph/paragraph.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/partitions/blocked_partition.hpp>
#include <stapl/views/array_view.hpp>

using namespace stapl;

struct dummy_wf
{
  typedef void result_type;

  template <typename ViewA>
  void operator()(ViewA const&) const
  { }
};


stapl::exit_code stapl_main(int, char*[])
{
  typedef indexed_domain<size_t>         dom_t;
  typedef block_partitioner<dom_t>       part_t;
  typedef array<size_t, part_t>          cnt_t;
  typedef array_view<cnt_t>              vw_t;

  part_t part(dom_t(0, 159999), 100);

  cnt_t ct(part);
  vw_t  vw(ct);

  for (int i = 0; i != 100; ++i)
    map_func(dummy_wf(), vw);

  return EXIT_SUCCESS;
}
