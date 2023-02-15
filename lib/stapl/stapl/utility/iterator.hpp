/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_ITERATOR_HPP
#define STAPL_UTILITY_ITERATOR_HPP

namespace stapl {
//////////////////////////////////////////////////////////////////////
/// @defgroup iteratorSpecializations Specializations for stand-alone begin.
/// @brief The specializations for stand-alone iterator operations such as
/// begin and end.
/// @{
/// @brief General implementation of stand-alone begin that redirects
/// it to type.begin().
//////////////////////////////////////////////////////////////////////
template <typename T>
auto begin(T&& t) -> decltype(t.begin())
{
  return t.begin();
}

//////////////////////////////////////////////////////////////////////
/// @brief General implementation of end that redirects it to type.end().
//////////////////////////////////////////////////////////////////////
template <typename T>
auto end(T&& t) -> decltype(t.end())
{
  return t.end();
}

/// @}

} // namespace stapl

#endif // STAPL_UTILITY_ITERATOR_HPP
