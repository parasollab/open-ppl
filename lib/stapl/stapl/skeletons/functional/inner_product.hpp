/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_INNER_PRODUCT_HPP
#define STAPL_SKELETONS_FUNCTIONAL_INNER_PRODUCT_HPP

#include "zip_reduce.hpp"

namespace stapl {
namespace skeletons {
namespace result_of {

template <typename ValueType>
using inner_product = result_of::zip_reduce<
                        2,
                        stapl::multiplies<ValueType>,
                        stapl::plus<ValueType>>;

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief Inner product skeleton specified in terms of the @c zip_reduce
/// skeleton.
///
/// @tparam ValueType the type of input elements to the inner product
///
/// @return an inner product skeleton
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename ValueType>
result_of::inner_product<ValueType>
inner_product()
{
  return skeletons::zip_reduce<2>(
           stapl::multiplies<ValueType>(),
           stapl::plus<ValueType>());
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_INNER_PRODUCT_HPP
