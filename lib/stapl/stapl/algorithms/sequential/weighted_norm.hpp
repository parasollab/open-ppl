/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_SEQUENTIAL_WEIGHTED_NORM_HPP
#define STAPL_ALGORITHMS_SEQUENTIAL_WEIGHTED_NORM_HPP

#include <cmath>

namespace stapl {
namespace sequential {

//////////////////////////////////////////////////////////////////////
/// @brief Compute a weighted normal of the elements of a view.
/// @param first1 One-dimensional view Iterator at the beginning of the
///   sequence of input elements.
/// @param last1 One-dimensional view Iterator at the end of the sequence of
///   input elements.
/// @param first2 One-dimensional view Iterator at the beginning of the
///   sequence of input elements (weights).
/// @return The square root of the sum of weighted squares of the input
///   elements.
///
/// The views must have the same size.
//////////////////////////////////////////////////////////////////////
template<class Iter1, class Iter2, class T>
T weighted_norm(Iter1 first1, Iter1 last1, Iter2 first2, T value)
{
    for(; first1 != last1; ++first1, ++first2) {
         value += *first1 * *first1 * *first2;
    }
    return std::sqrt(value);
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute a weighted normal of the elements of a view.
/// @param first1 One-dimensional view Iterator at the beginning of the
///   sequence of input elements.
/// @param last1 One-dimensional view Iterator at the end of the sequence of
///   input elements.
/// @param first2 One-dimensional view Iterator at the beginning of the
///   sequence of input elements (weights).
/// @param op1 Binary functor implementing addition.
/// @param op2 Binary functor implementing multiplication used to compute the
///   product of an element with itself and the product of the weight and the
///   element's square.
/// @return The square root of the sum of weighted squares of the input
///   elements.
///
/// The views must have the same size.
//////////////////////////////////////////////////////////////////////
template<class Iter1, class Iter2, class T, class Op1, class Op2>
T weighted_norm(Iter1 first1, Iter1 last1, Iter2 first2, T value,
                Op1 op1, Op2 op2)
{
    for(; first1 != last1; ++first1, ++first2) {
         value = op1(value, op2( op2(*first1, *first1) , *first2));
    }
    return std::sqrt(value);
}

} } // namespace stapl::sequential

#endif

