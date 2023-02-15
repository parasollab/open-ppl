/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_FOLD_HPP
#define STAPL_UTILITY_TUPLE_FOLD_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/utility.hpp>

namespace stapl {
namespace tuple_ops {

namespace detail {

template<typename BinaryOp, typename T, int N, typename ...Elements>
struct fold_impl
{
  auto operator()(tuple<Elements...> const& elements,
                  T const& init,
                  BinaryOp const& binop) const
  STAPL_AUTO_RETURN(
    binop(
      fold_impl<BinaryOp, T, N-1, Elements...>()(elements, init, binop),
      get<N-1>(elements)
    )
  )
};


template<typename BinaryOp, typename T, typename ...Elements>
struct fold_impl<BinaryOp, T, 0, Elements...>
{
  T operator()(tuple<Elements...> const&,
               T const& init, BinaryOp const&) const
  {
    return init;
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Returns the result of the repeated application of binary
/// operation @c binop to the result of the previous @c binop invocation,
/// (or @c init if it is the first call) and each element of the tuple.
///
/// @param elements tuple to be folded
/// @param init     initial value
/// @param binop    binary operation
//////////////////////////////////////////////////////////////////////
template<typename BinaryOp, typename T, typename ...Elements>
inline auto
fold(tuple<Elements...> const& elements,
     T const& init, BinaryOp const& binop)
STAPL_AUTO_RETURN((
  detail::fold_impl<
    BinaryOp, T, sizeof...(Elements), Elements...
  >()(elements, init, binop)
))

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_FOLD_HPP
