/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_SEQUENTIAL_WEIGHTED_INNER_PRODUCT_HPP
#define STAPL_ALGORITHMS_SEQUENTIAL_WEIGHTED_INNER_PRODUCT_HPP


namespace stapl {
namespace sequential {

template<class Iter1, class Iter2, class Iter3, class T>
T weighted_inner_product(Iter1 first1, Iter1 last1, Iter2 first2, Iter3 first3,
                         T value)
{
    for(; first1 != last1; ++first1, ++first2, ++first3) {
         value += *first1 * *first2 * *first3;
    }
    return value;
}

template<class Iter1, class Iter2, class Iter3, class T, class Op1, class Op2>
T weighted_inner_product(Iter1 first1, Iter1 last1, Iter2 first2, Iter3 first3,
                         T value, Op1 op1, Op2 op2)
{
    for(; first1 != last1; ++first1, ++first2, ++first3) {
         value = op1(value, op2( op2(*first1, *first2) , *first3));
    }
    return value;
}

} } // namespace stapl::sequential

#endif

