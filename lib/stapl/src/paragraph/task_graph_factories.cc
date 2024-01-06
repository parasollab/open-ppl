/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cmath>
#include <vector>
#include <stapl/skeletons/explicit/tree_info.h>

namespace stapl {

namespace paragraph_impl {

int compute_combine_trees(float const& n, std::vector<tree_info>& trees)
{
  int elem_remaining = int(n);
  int leaf_offset    = 0;
  int id_offset      = 0;

  for (int curr_width = int(std::pow(2.0,std::floor(std::log2(n))));
       curr_width != 0 && elem_remaining != 0;
       curr_width >>= 1)
  {
    if (curr_width <= elem_remaining)
    {
      tree_info t;

      t.width = curr_width;
      t.height = int(std::log2(curr_width));
      t.leaf_offset = leaf_offset;
      t.id_offset = id_offset;
      t.root_task_id = id_offset + 2*curr_width - 2;
      t.curr_id = id_offset + curr_width; //curr_id is not used for map tasks
      t.elements_remaining = 0; // elements_remaining is not used for map_reduce

      trees.push_back(t);

      elem_remaining -= curr_width;
      leaf_offset += curr_width;
      id_offset += 2*curr_width - 1;
    }
  }
  return int(std::ceil(std::log2(n)));
}

} // namespace paragraph_impl

} // namespace stapl
