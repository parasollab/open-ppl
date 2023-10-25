/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_TUPLE_CAT_UNIQUE_HPP
#define STAPL_UTILITY_TUPLE_TUPLE_CAT_UNIQUE_HPP

#include "fold_type.hpp"
#include "insert_type.hpp"
namespace stapl {
namespace tuple_ops {

//////////////////////////////////////////////////////////////////////
/// @brief Concatenate tuples types discarding copies.
//////////////////////////////////////////////////////////////////////
template<class... Tuple>
using tuple_cat_unique = fold_types<insert_type, tuple<>,
      typename stapl::result_of::tuple_cat<Tuple...>::type>;

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_TUPLE_CAT_UNIQUE_HPP
