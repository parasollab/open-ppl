/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_NON_RMI_ABORT_RMI_HPP
#define STAPL_RUNTIME_NON_RMI_ABORT_RMI_HPP

#include <sstream>
#include <string>

namespace stapl {

////////////////////////////////////////////////////////////////////
/// @brief Displays the given @c std::string and aborts execution.
///
/// @ingroup ARMI
////////////////////////////////////////////////////////////////////
void abort(std::string const&);


////////////////////////////////////////////////////////////////////
/// @brief Aborts execution.
///
/// @ingroup ARMI
////////////////////////////////////////////////////////////////////
inline void abort(void)
{
  abort(std::string{});
}


////////////////////////////////////////////////////////////////////
/// @brief Outputs the given object to @c std::cerr as a string and aborts
///        execution.
///
/// @ingroup ARMI
////////////////////////////////////////////////////////////////////
template<typename T>
void abort(T const& t)
{
  std::ostringstream oss;
  oss << t;
  abort(oss.str());
}

} // namespace stapl

#endif
