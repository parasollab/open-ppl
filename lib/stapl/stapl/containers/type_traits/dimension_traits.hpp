/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DIMENSION_TRAITS_HPP
#define STAPL_CONTAINERS_DIMENSION_TRAITS_HPP

#include <boost/mpl/int.hpp>
#include <boost/mpl/has_xxx.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/tuple/ensure_tuple.hpp>

namespace stapl { namespace view_impl {

BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF(has_dimension_type, dimension_type, false)
BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF(has_dimensions_type, dimensions_type, false)

//////////////////////////////////////////////////////////////////////
/// @brief Helper metafunction to return a nested dimension_type trait,
/// if it exists.
///
/// @tparam T The type for which to compute the number of dimensions
//////////////////////////////////////////////////////////////////////
template<typename T>
struct dimension_helper
{
  using type = typename T::dimension_type;
};

template<typename T>
struct dimensions_helper
{
  using type = typename tuple_size<
                 tuple_ops::ensure_tuple_t<typename T::dimensions_type>
               >::type;
};

//////////////////////////////////////////////////////////////////////
/// @copydoc dimension_helper
//////////////////////////////////////////////////////////////////////
template<>
struct dimension_helper<void>
{
  using type = std::integral_constant<std::size_t, 1>;
};

} // end namespace view_impl

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute a compile-time constant representing
/// the number of dimensions of a particular type (container, domain, size,
/// etc.)
///
/// @tparam T The type for which to compute the number of dimensions
//////////////////////////////////////////////////////////////////////
template<typename T>
struct dimension_traits
  : std::conditional<
      view_impl::has_dimension_type<T>::value,
      view_impl::dimension_helper<T>,
      typename std::conditional<
        view_impl::has_dimensions_type<T>::value,
        view_impl::dimensions_helper<T>,
        view_impl::dimension_helper<void>
      >::type
    >::type
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for tuples
/// @see dimension_traits
//////////////////////////////////////////////////////////////////////
template<typename... T>
struct dimension_traits<tuple<T...> >
  : tuple_size<tuple<T...> >
{};

} // namespace stapl

#endif //
