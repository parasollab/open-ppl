/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_INFINITE_HELPERS_HPP
#define STAPL_VIEWS_METADATA_INFINITE_HELPERS_HPP

#include <type_traits>
#include <stapl/utility/tuple.hpp>
#include <stapl/utility/pack_ops.hpp>
#include <stapl/domains/infinite.hpp>
#include <stapl/views/type_traits/has_domain.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Implementation metafunction for @ref has_finite_domain, which
///   guards inspection of  a domain_type typedef to views that define it.
///
/// @todo Remove when all views define domain_type (i.e., inherit from
///   @ref core_view)
//////////////////////////////////////////////////////////////////////
template<typename View, bool = has_domain_type<View>::value>
struct has_finite_domain_impl
  : public std::integral_constant<
             bool,
             !(std::is_base_of<infinite_impl::infinite_base,
                               typename View::domain_type>::value)
    >
{ };


template<typename View>
struct has_finite_domain_impl<View, false>
  : public std::true_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction returning true if View parameter has a
///   domain type other than @ref infinite.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct has_finite_domain
  : public has_finite_domain_impl<View>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction returning the index of the first element
///   (i.e., view) of the tuple parameter @p Views which has a finite
///   domain.
//////////////////////////////////////////////////////////////////////
template<typename Views>
using first_finite_domain_index =
  stapl::find_first_index<Views, has_finite_domain>;


//////////////////////////////////////////////////////////////////////
/// @brief Reflects constexpr value which is the sum of views in the
/// given viewset that have finite domains.
//////////////////////////////////////////////////////////////////////
template<typename ViewSet>
struct count_finite_domains;


template<typename ...Views>
struct count_finite_domains<tuple<Views...>>
{
  static constexpr int value =
    pack_ops::functional::plus_(
      0, has_finite_domain<Views>::value ? 1 : 0 ...);
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_INFINITE_HELPERS_HPP
