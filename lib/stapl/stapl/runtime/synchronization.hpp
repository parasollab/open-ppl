/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SYNCHRONIZATION_HPP
#define STAPL_RUNTIME_SYNCHRONIZATION_HPP

#include <functional>

namespace stapl {

namespace runtime {

class context;


//////////////////////////////////////////////////////////////////////
/// @brief Ensures that all outstanding RMI requests have been completed.
///
/// This function also is an implicit barrier.
///
/// @warning This is an SPMD function.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
void rmi_fence(context&);

//////////////////////////////////////////////////////////////////////
/// @brief Ensures that all outstanding RMI requests have been completed and
///        the return value of @p f is @c true.
///
/// This function also is an implicit barrier.
///
/// @warning This is an SPMD function.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
void rmi_fence(context&, std::function<bool(void)> f);

} // namespace runtime

} // namespace stapl

#endif
