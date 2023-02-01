/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/rmi_handle.hpp>
#include <ostream>

namespace stapl {

namespace runtime {

// Outputs an object_virtual_address object
std::ostream& operator<<(std::ostream& os, object_virtual_address const& h)
{
  switch (h.m_type) {
    case object_virtual_address::UNKNOWN:
      return os << "NA";
    case object_virtual_address::DIRECT:
      return os << "&" << h.direct();
    case object_virtual_address::INDIRECT:
      return os << "A" << h.indirect();
    default:
      STAPL_RUNTIME_ERROR("Incorrect handle type.");
    }
    return os;
}


// Outputs an rmi_handle_info object
std::ostream& operator<<(std::ostream& os, rmi_handle_info const& h)
{
  return os << "{ gang="               << h.m_gid
            << ", handle="             << h.m_internal_handle
            << ", registration_epoch=" << h.m_registration_epoch
            << ", current_epoch="      << h.m_epoch
            << '}';
}

} // namespace runtime

} // namespace stapl
