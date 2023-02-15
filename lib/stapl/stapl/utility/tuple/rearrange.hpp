/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_REARRANGE_HPP
#define STAPL_UTILITY_TUPLE_REARRANGE_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>

#include <boost/mpl/find_if.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/distance.hpp>

#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Boost.MPL conforming metafunction to check that the passed
///   compile-time constant is equal to @p N, regardless of whether it is
///   a boost::mpl::int_ or std::integral_constant.
///
/// @todo Remove when we have completely moved to std::integral_constant.
//////////////////////////////////////////////////////////////////////
template<int N>
struct type_constant_equal
{
  template<typename Arg1>
  struct apply
  {
    using type = boost::mpl::bool_<Arg1::value == N>;
  };
};

namespace tuple_ops {
namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Rearrange the contents of a compile-time tuple based on the
///        indicies specified in another compile-time tuple.
///
/// For example, given the tuple <0, 1, 2> and the order <1, 2, 0>,
/// output the tuple <2, 0, 1>.
///
/// @tparam Tuple Type of the compile-time tuple to reorder
/// @tparam Order Tuple which represents the indices of the other tuple
//////////////////////////////////////////////////////////////////////
template<typename Tuple, typename Order,
         typename Indices = make_index_sequence<tuple_size<Order>::value>>
struct rearrange;


template<typename Tuple, typename... IntTypes, std::size_t... Indices>
struct rearrange<Tuple, tuple<IntTypes...>, index_sequence<Indices...>>
{
private:
  using vector_t = boost::mpl::vector<IntTypes...>;

  using indices_t = tuple<
    typename boost::mpl::distance<
      typename boost::mpl::begin<vector_t>::type,
      typename boost::mpl::find_if<
        vector_t, type_constant_equal<Indices>>::type
    >::type...>;

public:
  using type = tuple<
    std::integral_constant<
      std::size_t,
      tuple_element<
        tuple_element<Indices, indices_t>::type::value,
        Tuple
      >::type::value
    >
  ...>;
};

} // namespace result_of
} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_REARRANGE_HPP
