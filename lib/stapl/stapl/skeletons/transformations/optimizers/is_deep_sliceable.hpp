/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_OPTIMIZERS_IS_DEEP_SLICEABLE_HPP
#define STAPL_SKELETONS_OPTIMIZERS_IS_DEEP_SLICEABLE_HPP

#include <stapl/containers/multiarray/deep_slice.hpp>
#include <stapl/containers/type_traits/is_base_container.hpp>
#include <stapl/views/type_traits/is_identity.hpp>
#include <stapl/views/type_traits/is_segmented_view.hpp>


namespace stapl {

namespace paragraph_impl {
template <typename Sched>
class paragraph_view;
} // namespace paragraph_impl

template<class T, class Domain, class CID, class Traits>
class multiarray_base_container;

template<int D, typename T>
class basic_multiarray_base_container;

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a type is a multiarray base container.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct is_multiarray_bc
: public std::false_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for multiarray base container
//////////////////////////////////////////////////////////////////////
template<class T, class Domain, class CID, class Traits>
struct is_multiarray_bc<multiarray_base_container<T, Domain, CID, Traits>>
: public std::true_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for basic multiarray base container
//////////////////////////////////////////////////////////////////////
template<int D, typename T>
struct is_multiarray_bc<basic_multiarray_base_container<D, T>>
: public std::true_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a view can be deep sliced.
//////////////////////////////////////////////////////////////////////
template<typename View,
         bool IsView = stapl::is_view<typename std::decay<View>::type>::value>
struct is_deep_sliceable
{
  using view_t = typename std::decay<View>::type;

  static constexpr bool value = std::integral_constant<bool,
    has_identity_mf<view_t>::value &&
    !is_segmented_view<view_t>::value && (
    is_multiarray_bc<
      typename underlying_container<view_t>::type
    >::value ||
    is_deep_slice<
      typename underlying_container<view_t>::type
    >::value)
 >::value;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for paragraph_view. It is deep sliceable
//////////////////////////////////////////////////////////////////////
template<typename Sched>
struct is_deep_sliceable<paragraph_impl::paragraph_view<Sched>, false>
 : public std::true_type
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for other non-views. They are not deep sliceable
//////////////////////////////////////////////////////////////////////
template<typename View>
struct is_deep_sliceable<View, false>
 : public std::false_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute whether a pack of views is deep sliceable.
//////////////////////////////////////////////////////////////////////
template<typename... V>
struct pack_is_deep_sliceable;

template<typename V0, typename... V>
struct pack_is_deep_sliceable<V0, V...>
 : public std::integral_constant<bool,
            is_deep_sliceable<V0>::value && pack_is_deep_sliceable<V...>::value
          >
{ };

template<>
struct pack_is_deep_sliceable<>
 : public std::true_type
{ };

} // namespace stapl


#endif // STAPL_SKELETONS_OPTIMIZERS_IS_DEEP_SLICEABLE_HPP