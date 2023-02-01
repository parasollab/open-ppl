/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdlib>
#include <iostream>

#include <stapl/runtime.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/partitions/splitter.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/segmented_view.hpp>

#include "../../test_report.hpp"

using namespace stapl;

struct print_size
{
  typedef void result_type;

  template<typename ViewType, typename DomainType>
  void operator()(ViewType const& view, DomainType dom)
  {
    dom = view.domain();
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef array<size_t>           array_type;
  typedef array_view<array_type>  View;

  typedef View::gid_type                        gid_t;
  typedef View::domain_type                     dom_t;
  typedef array<gid_t>                          indices_t;
  typedef array_view<indices_t>                 indices_v;
  typedef splitter_partition<dom_t, indices_v>  part_t;
  typedef segmented_view<View, part_t>        part_view_t;

  typedef array<dom_t>                          out_array_t;
  typedef array_view<out_array_t>               out_view_t;

  array_type  arry(1024);
  View        view(arry);

  //test1
  indices_t   ixs(4);
  indices_v   ixv(ixs);
  ixv[0] = 0;
  ixv[1] = 100;
  ixv[2] = 924;
  ixv[3] = 1024;

  part_t        part1(view.domain(), ixv, false);
  part_view_t   part_view1(view, part1);

  out_array_t  out1(part_view1.size());
  out_view_t   out_view1(out1);

  map_func(print_size(), part_view1, out_view1);

  bool res = true;
  if (part_view1.size() != 3)
    res = false;
  res &= ((out1.get_element(0).first() == 0   &&
           out1.get_element(0).last()  == 99) &&
          (out1.get_element(1).first() == 100 &&
           out1.get_element(1).last()  == 923) &&
          (out1.get_element(2).first() == 924 &&
           out1.get_element(2).last() == 1023) );

  //test2
  ixv[0] = 0;
  ixv[1] = 512;
  ixv[2] = 512;
  ixv[3] = 1024;

  part_t        part2(view.domain(), ixv, true);
  part_view_t   part_view2(view, part2);

  out_array_t  out2(part_view2.size());
  out_view_t   out_view2(out2);

  map_func(print_size(), part_view2, out_view2);

  bool res2 = true;
  if (part_view2.size() != 5)
    res2 = false;

  res2 &= ( (out2.get_element(0).size() == 0)   &&
            (out2.get_element(1).first() == 0    &&
             out2.get_element(1).last()  == 511)   &&
            (out2.get_element(2).size() == 0)   &&
            (out2.get_element(3).first() == 512  &&
             out2.get_element(3).last()  == 1023)  &&
            (out2.get_element(4).size() == 0) );

  STAPL_TEST_REPORT(res && res2, "Testing splitter partition:")

  return EXIT_SUCCESS;
}
