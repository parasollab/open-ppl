/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_IGNORE_INDEX_HPP
#define STAPL_UTILITY_TUPLE_IGNORE_INDEX_HPP

#include <cstddef>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper function that returns the value passed to it and ignores
///        the index template parameter.
//////////////////////////////////////////////////////////////////////
template<std::size_t I, typename T>
T ignore_index(T const& t)
{
  return t;
}

} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_IGNORE_INDEX_HPP
