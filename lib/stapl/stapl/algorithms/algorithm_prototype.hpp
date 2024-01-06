/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_ALGORITHM_PROTOTYPE_HPP
#define STAPL_ALGORITHMS_ALGORITHM_PROTOTYPE_HPP

#include <numeric>
#include "functional.hpp"

#include <stapl/skeletons/explicit/map_reduce_prototype.hpp>
#include <stapl/skeletons/explicit/map_prototype.hpp>

namespace stapl {

namespace prototype {

//////////////////////////////////////////////////////////////////////
/// @brief Assigns the given value to the elements of the input view.
/// @param vw One-dimensional view of the input.
/// @param value The value to fill into the input.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the input view.
//////////////////////////////////////////////////////////////////////
template<typename View>
void fill(View& vw, typename View::value_type value)
{
  ::stapl::prototype::map_func(
      stapl::bind1st(assign<typename View::value_type>(), value), vw);
}

//////////////////////////////////////////////////////////////////////
/// @brief Applies the given functor to all of the elements in the input.
/// @param vw0 One-dimensional view over the input.
/// @param func Unary functor to apply to the elements.
/// @return The functor that was passed as input.
/// @ingroup generatingAlgorithms
///
/// This algorithm will mutate the input view.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename Function>
Function
for_each(View0 const& vw0, Function func)
{
  ::stapl::prototype::map_func(func, vw0);

  // kind of pointless...
  return func;
}

namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Defines the return type of the @ref count function.
/// @tparam View Type of the input view.
/// @tparam T Type of the element to count.
///
/// @todo the following bind2nd needs typename ::type but causes compile error
//////////////////////////////////////////////////////////////////////
template<typename View, typename T>
struct count
  : public ::stapl::prototype::result_of::map_reduce<

      ::stapl::result_of::bind2nd<equal_to<T>, T>,         // map
      plus<typename View::iterator::difference_type>,      // reduce
      true,
      View
    >
{ };

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief Computes the number of elements in the input view which compare
///   equal to the given value.
/// @param view One-dimensional view over the input elements.
/// @param value Value to count the occurrences of in the input.
/// @return The number of occurrences of the given element in the input.
/// @ingroup countingAlgorithms
///
/// This algorithm is non-mutating.
///
/// @todo - track down why std::bind1st seems to be seeping into
/// this namespace (icc)
//////////////////////////////////////////////////////////////////////
template<typename View, typename T>
typename result_of::count<View, T>::type
count(View const& view, T const& value)
{

  typedef typename View::iterator::difference_type result_t;
  return ::stapl::prototype::map_reduce(
           stapl::bind2nd(equal_to<T>(), value), plus<result_t>(), view);
}

} // namespace prototype
} //namespace stapl

#endif // STAPL_ALGORITHMS_ALGORITHM_PROTOTYPE_HPP

