/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_FILTER_HPP
#define STAPL_UTILITY_TUPLE_FILTER_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/tuple/discard.hpp>
#include <stapl/utility/tuple/homogeneous_tuple.hpp>

namespace stapl {
namespace tuple_ops {

namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that reflects the type of the @see stapl::filter
///        function.
//////////////////////////////////////////////////////////////////////
template<typename Slices, typename Tuple>
struct filter
{
  using result_size_t = typename stapl::tuple_size<Slices>::type ;
  using scalar_t      = typename stapl::tuple_element<
                          0, typename std::decay<Tuple>::type>::type;

  // the result type should be a tuple of size |Slices|.
  // but if this value is 1, we should just use a scalar instead
  // of a tuple of size 1
  using type = typename std::conditional<
    result_size_t::value == 1,          // if
    scalar_t,                           // then
    typename homogeneous_tuple_type<    // else
     result_size_t::value, scalar_t
    >::type
  >::type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that filters a d-dimensional tuple by selecting out
///        only the indices specified in a tuple of compile-time integer
///        constants.
///
/// For example, heterogeneous_filter<<0,2>, <4,5,6>> should return <4,6>.
///
/// @tparam Index The tuple of indices
/// @tparam Tuple The tuple of elements from which to select.
//////////////////////////////////////////////////////////////////////
template<typename Index, typename Tuple>
struct heterogeneous_filter;

template<std::size_t... IndexIndices, std::size_t... TupleIndices>
struct heterogeneous_filter<
  tuple<std::integral_constant<std::size_t, IndexIndices>...>,
  tuple<std::integral_constant<std::size_t, TupleIndices>...>>
{
  using as_tuple = tuple<std::integral_constant<std::size_t, TupleIndices>...>;

  using type = tuple<
    typename tuple_element<IndexIndices, as_tuple>::type...
  >;
};

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Filter a d-dimensional tuple by selecting out only the indices
///        specified in a tuple of compile-time integer constants. This
///        function provides the inverse behavior of @see discard
///
///        Note that this function currently only supports tuples
///        of homogeneous types.
///
/// @tparam Slices A tuple of std::integral_constants which specifies
///                which indices in the tuple to keep.
/// @param  t The tuple to filter
/// @return A tuple of size |Slices| where the rest of the elements are
///         filtered out
//////////////////////////////////////////////////////////////////////
template<typename Slices, typename Tuple>
typename result_of::filter<Slices, typename std::decay<Tuple>::type>::type
filter(Tuple&& t)
{
  using tuple_t  = typename std::decay<Tuple>::type;

  using result_t = typename result_of::filter<Slices, tuple_t>::type;

  using filter_t = detail::discard_impl<
                     0,
                     stapl::tuple_size<tuple_t>::type::value,
                     tuple_t, result_t, Slices, true>;

  return filter_t::apply(std::forward<Tuple>(t));
}

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_FILTER_HPP
