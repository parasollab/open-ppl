/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_TYPE_TRAITS_IS_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_TYPE_TRAITS_IS_BASE_CONTAINER_HPP

#include <type_traits>

namespace stapl {

struct bc_base;

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a given type is a STAPL base container.
///
/// @tparam T The type in question
//////////////////////////////////////////////////////////////////////
template<typename T>
using is_base_container = std::is_base_of<bc_base, T>;

} // namespace stapl

#endif
