/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"
#include <stapl/containers/sequential/graph/vertex_iterator_adaptor.h>
#include <cstdio>

using namespace stapl::sequential;

int main(int, char**)
{
  typedef graph<stapl::DIRECTED, stapl::NONMULTIEDGES, int, int> graph_type;
  graph_type g;

  g.add_vertex(0, 1);
  g.add_vertex(1, 2);
  g.add_vertex(2, 3);
  g.add_vertex(3, 4);
  g.add_vertex(4, 5);
  g.add_vertex(5, 6);
  g.add_vertex(6, 7);
  g.add_vertex(7, 8);
  g.add_vertex(8, 9);
  g.add_vertex(9, 10);
  g.add_vertex(10, 11);

  vd_iterator<graph_type::vertex_iterator> di = g.begin();
  vd_iterator<graph_type::vertex_iterator> d_end = g.end();
  graph_type::vertex_descriptor descriptor_sum(0);
  for (; di != d_end; ++di)
    descriptor_sum += (*di);

  vdata_iterator<graph_type::vertex_iterator> pi = g.begin();
  vdata_iterator<graph_type::vertex_iterator> p_end = g.end();
  int property_sum(0);
  for (; pi != p_end; ++pi)
    property_sum += *pi;

  const_vdata_iterator<graph_type::vertex_iterator> cpi = g.begin();
  const_vdata_iterator<graph_type::vertex_iterator> cp_end = g.end();
  int const_property_sum(0);
  for (; cpi != cp_end; ++cpi)
    const_property_sum += *cpi;

  if (descriptor_sum == 55)
    printf("sequential graph descriptor iterator adaptor PASSED.\n");
  else
    printf("sequential graph descriptor iterator adaptor FAILED (%zu).\n",
           descriptor_sum);

  if (property_sum == 66)
    printf("sequential graph property iterator adaptor PASSED.\n");
  else
    printf("sequential graph property iterator adaptor FAILED (%d).\n",
           property_sum);


  if (const_property_sum == 66)
    printf("sequential graph const property iterator adaptor PASSED.\n");
  else
    printf("sequential graph const property iterator adaptor FAILED (%d).\n",
           const_property_sum);

  return 0;
}
