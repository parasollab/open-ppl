/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/gang_description.hpp>
#include <ostream>

namespace stapl {

namespace runtime {

// Output operator
std::ostream& operator<<(std::ostream& os, gang_description const& gd)
{
  os << "Processes: " << gd.get_num_processes()
     << " { ";
  for (const auto pid : gd.get_processes())
    os << pid << ' ';
  os << "}\n"
     << "Logical: " << gd.get_num_locations() << " { ";
  for (auto i = 0u; i < gd.get_num_locations(); ++i)
    os << i << "->" << gd.get_process_id(i) << ' ';
  os << "}\n"
     << "Grouped: ";
  for (const auto pid : gd.get_processes()) {
    const auto v = gd.get_location_ids(pid);
    os << "{[" << pid << "] ";
    for (auto lid : v)
      os << lid << ' ';
    os << "} ";
  }
  return os << std::endl;
}

} // namespace runtime

} // namespace stapl
