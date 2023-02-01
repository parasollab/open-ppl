/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONSTANTS_HPP
#define STAPL_RUNTIME_CONSTANTS_HPP

#include "config/types.hpp"
#include <limits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Invalid process id.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
constexpr process_id invalid_process_id = -1;


//////////////////////////////////////////////////////////////////////
/// @brief Invalid location id.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
constexpr unsigned int invalid_location_id =
  std::numeric_limits<unsigned int>::max();


namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Invalid gang id.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
constexpr gang_id invalid_gang_id = std::numeric_limits<gang_id>::max();


//////////////////////////////////////////////////////////////////////
/// @brief Invalid @ref object_id.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
constexpr object_id invalid_object_id = std::numeric_limits<object_id>::max();

} // namespace runtime

} // namespace stapl

#endif
