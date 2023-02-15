/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/properties.hpp>

namespace stapl {

namespace properties {

std::ostream& operator<<(std::ostream& os, triangle_count_property const& p)
{
  os << p.num_triangles() << ", " << p.num_connected_triplets() << ", {";

  for (auto waiting : p.waiting_queue())
    os << waiting << " ";

  os << "}";

  return os;
}

} // namespace properties

} // namespace stapl
