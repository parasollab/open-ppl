/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_MAKE_VECTOR_HPP
#define STAPL_RUNTIME_UTILITY_MAKE_VECTOR_HPP

#include <iterator>
#include <vector>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Returns a @c std::vector<T>.
///
/// @param v The vector to return.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T>
std::vector<T> make_vector(std::vector<T> v)
{
  return v;
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a @c std::vector<T>.
///
/// @param first Iterator to the start of the range.
/// @param last  Iterator to the end of the range.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T, typename InputIterator>
std::vector<T> make_vector(InputIterator first, InputIterator last)
{
  return std::vector<T>(first, last);
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a @c std::vector<T>.
///
/// @param u Value range.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T,
         typename U,
         typename = typename std::enable_if<
                      !std::is_same<
                        std::vector<T>, typename std::decay<U>::type
                      >::value>::type>
std::vector<T> make_vector(U&& u)
{
  return std::vector<T>(std::begin(u), std::end(u));
}

} // namespace runtime

} // namespace stapl

#endif
