/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_UTILITY_HPP
#define STAPL_SKELETONS_UTILITY_UTILITY_HPP

#include <type_traits>
#include <stapl/utility/use_default.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/views/type_traits/has_domain.hpp>
#include <boost/mpl/has_xxx.hpp>
#include <stapl/skeletons/spans/spans_fwd.hpp>

namespace stapl {

BOOST_MPL_HAS_XXX_TRAIT_DEF(result_type)
BOOST_MPL_HAS_XXX_TRAIT_DEF(metadata_type)


template <typename C>
class repeat_view;


template <typename V, typename P, typename CC>
class partitioned_mix_view;


namespace skeletons {

struct no_filter
{
  template <typename Arg>
  Arg operator()(Arg&& arg)
  {
    return arg;
  }
};

struct no_mapper
{
  no_filter get_out_to_in_mapper(void) const
  {
    return no_filter();
  }

  no_filter get_in_to_in_mapper(void) const
  {
    return no_filter();
  }

  no_filter get_out_to_out_mapper(void) const
  {
    return no_filter();
  }
};


template <typename In, int i>
using flow_value_type = typename tuple_element<i, In>::type::flow_value_type;

template <typename V>
struct has_finite_domain
  : public std::true_type
{ };


template <typename C>
struct has_finite_domain<repeat_view<C>>
  : public std::false_type
{ };


template <typename C, typename P, typename CC>
struct has_finite_domain<partitioned_mix_view<repeat_view<C>, P, CC>>
  : public std::false_type
{ };


template <typename D, bool is_finite = true>
struct domain_type
  : public D
{
  using is_finite_type = std::integral_constant<bool, is_finite>;

  template<typename... Args>
  explicit domain_type(Args&&... args)
    : D(std::forward<Args>(args)...)
  { }
};


template <typename D>
using is_finite_domain = typename D::is_finite_type;

//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction returning the index of the first element
/// (i.e., domain) of the tuple parameter @p View domains which is
/// finite.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template<typename VDomains>
using first_finite_domain_index = stapl::find_first_index<
                                    VDomains, is_finite_domain>;


namespace flows {

template <typename View>
class view_flow;

} // namespace flows


template <typename ViewsTuple>
struct find_flows_type;

template <typename... Views>
struct find_flows_type<tuple<Views...>>
{
  using type =
    typename std::decay<std::tuple<flows::view_flow<Views>...>>::type;
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Finding the domain type of a factory
///
/// @tparam Sk        the factory type looking for it's domain type
/// @tparam ViewTuple tuple of views that passed as input to the factory
///
/// @ingroup skeletonsUtilities
///////////////////////////////////////////////////////////////////////////////
template <typename Factory, typename ViewsTuple, bool is_sk>
struct is_domain_scalar;

///////////////////////////////////////////////////////////////////////////////
/// @brief specialization for the case that factory is a skeleton. In this case
///        we get the domain type from the output port of the skeleton.
///
///////////////////////////////////////////////////////////////////////////////
template <typename Sk, typename ViewSet>
struct is_domain_scalar<Sk, ViewSet, true>
{

  using view_flows_tuple_t =
    typename find_flows_type<
      decltype(std::declval<typename Sk::coarsener_type>()(
        std::declval<ViewSet>()))
    >::type;

  using out_port_t = typename std::decay<Sk>::type::template out_port_type<
    typename std::decay<view_flows_tuple_t>::type>::type;

  using is_scalar_type =
    typename tuple_element<0, out_port_t>::type::domain_type::is_scalar_type;
};

///////////////////////////////////////////////////////////////////////////////
/// @brief specialization for the non-skeleton factory. In this case we check
///        only if the factory has a domain type or not.
///////////////////////////////////////////////////////////////////////////////
template <typename Factory, typename ViewSet>
struct is_domain_scalar<Factory, ViewSet, false>
{
  using is_scalar_type =
    std::integral_constant<bool, !has_domain_type<Factory>::value>;
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Finding if the output of a factory is reduced to an scalar
///        value or is an aggregated value.
///
/// @tparam Sk        the factory type looking for it's output type
/// @tparam ViewTuple tuple of views that passed as input to the factory
///
/// @ingroup skeletonsUtilities
///////////////////////////////////////////////////////////////////////////////
template <typename Factory,
          typename ViewSet,
          bool is_sk = is_skeleton<Factory>::value>
struct is_output_scalar
  : std::integral_constant<
      bool,
      !is_domain_scalar<Factory, ViewSet, is_sk>::is_scalar_type::value>
{ };

///////////////////////////////////////////////////////////////////////////////
/// @brief compile time max function
///////////////////////////////////////////////////////////////////////////////
template <std::size_t... Values>
struct find_max_value;

template <std::size_t i>
struct find_max_value<i>
{
  using type = std::integral_constant<std::size_t, i>;
};

template <std::size_t Value, std::size_t... Values>
struct find_max_value<Value, Values...>
{
  using type =
    std::integral_constant<std::size_t,
                           (Value > find_max_value<Values...>::type::value)
                             ? Value
                             : find_max_value<Values...>::type::value>;
};

///////////////////////////////////////////////////////////////////////////////
/// @brief unwraps the underlying skeleton in a wrapped skeleton
///////////////////////////////////////////////////////////////////////////////
template <typename Skeleton,
          bool isNested = is_nested_skeleton<Skeleton>::value>
struct unwrap
{
  using type = typename Skeleton::wrapped_skeleton_type;
};

template <typename Skeleton>
struct unwrap<Skeleton, false>
{
  using type = Skeleton;
};

BOOST_MPL_HAS_XXX_TRAIT_DEF(nested_p_type)

///////////////////////////////////////////////////////////////////////////////
/// @brief Traverses the skeleton tree to find the depth of tree. It is used for
///        finding the number of levels to pass to the @ref nest_blocked span.
///
/// @tparam Root       specifies type of the skeleton at the root of tree.
/// @tparam isCompose  denotes if it's the composition of skeletons or not.
/// @tparam isSkeleton denotes if it's the leaf of the tree or not.
///////////////////////////////////////////////////////////////////////////////
template <typename Root,
          bool isCompose = has_nested_p_type<Root>::value,
          bool isSkeleton = is_skeleton<Root>::value>
struct Traverse;

template <typename Root>
struct Traverse<Root, true, true>
{
  using dims = typename std::conditional<
    is_nested_skeleton<typename Root::op_type>::value,
    std::integral_constant<
      size_t,
      find_max_value<
        tuple_size<tuple_ops::ensure_tuple_t<typename Root::dims_type>>::value,
        Traverse<typename unwrap<typename Root::op_type>::type>::dims::value>::
        type::value>,
    std::integral_constant<size_t,
                           tuple_size<tuple_ops::ensure_tuple_t<
                             typename Root::dims_type>>::value>>::type;

  using type =
    typename std::conditional<
      is_nested_skeleton<typename Root::op_type>::value,
      std::integral_constant<
        size_t,
        1 + Traverse<
              typename unwrap<typename Root::op_type>::type>::type::value>,
      std::integral_constant<size_t, 0>
    >::type;
};

template <typename Root>
struct Traverse<Root, false, true>
{
  using dims = typename Traverse<typename Root::skeletons_type>::dims;
  using type = typename Traverse<typename Root::skeletons_type>::type;
};

template <typename... Root>
struct Traverse<std::tuple<Root...>, false, false>
{
  using dims = typename find_max_value<Traverse<Root>::dims::value...>::type;
  using type = typename find_max_value<Traverse<Root>::type::value...>::type;
};

template <typename Root, bool isCompose>
struct Traverse<Root, isCompose, false>
{
  using dims = std::integral_constant<size_t, 0>;
  using type = std::integral_constant<size_t, 0>;
};

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_UTILITY_HPP
