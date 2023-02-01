/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_UPCAST_HPP
#define STAPL_VIEWS_TYPE_TRAITS_UPCAST_HPP

#include <stapl/utility/use_default.hpp>
#include <stapl/views/mapping_functions/mapping_functions.hpp>
#include <stapl/views/type_traits/is_segmented_view.hpp>
#include <stapl/views/type_traits/is_sliced_view.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper class to change the domain and mapping function type
///        of the given @p View to the specified new domain type
///        @p Dom and mapping function type @p MF.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Dom, typename MF>
struct upcast_view
{
  typedef View type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref upcast_view when the given view is
///        defined with three template parameters (container @p OldC,
///        domain @p Dom and mapping function @p Mf).
//////////////////////////////////////////////////////////////////////
template<template<typename,typename,typename> class View,
         typename C,
         typename OldDom,
         typename OldMf,
         typename Dom,
         typename MF>
struct upcast_view<View<C,OldDom,OldMf>, Dom, MF>
{
  typedef View<C,Dom,MF> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref upcast_view when the given view
///        (container @p C, domain @p OldDom, mapping
///        function @p OldMf and most derived class @p Der).
//////////////////////////////////////////////////////////////////////
template<template<typename,typename,typename, typename> class View,
         typename C,
         typename OldDom,
         typename OldMf,
         typename Der,
         typename Dom,
         typename MF>
struct upcast_view<View<C,OldDom,OldMf,Der>, Dom, MF>
{
  typedef View<C,Dom,MF,Der> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper metafunction for @ref new_upcast_view that dispatches
///   based on presence of optional parameters and domain / mapping function
///   changes to compute a view with possibly different domain and mapping
///   function type parameters.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Dom, typename MF,
  bool = std::is_same<typename view_traits<View>::domain_type, Dom>::value,
  bool = std::is_same<typename view_traits<View>::map_function, MF >::value
>
struct upcast_view_helper;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization handling case that no domain or mapping function
///   provided in original's view instantiation and new domain and mapping
///   function are the same.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Dom, typename MF>
struct upcast_view_helper<View, Dom, MF, true, true>
{ typedef View type; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization handling case that no domain or mapping function
///   provided in original's view instantiation and new domain is not the
///   same but the mapping function is the same.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename...> class View,
         typename C, typename Dom, typename MF>
struct upcast_view_helper<View<C>, Dom, MF, false, true>
{ typedef View<C, Dom> type; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization handling case that no domain or mapping
///   function provided in original's view instantiation and new
///   domain and the mapping fucntion are the same.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename...> class View,
        typename C, typename Dom, typename MF>
struct upcast_view_helper<View<C>, Dom, MF, false, false>
{ typedef View<C, Dom, MF> type; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization handling case that domain is
///   provided in original's view instantiation and new domain is not the
///   same but the mapping function is the same.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename, typename...> class View, typename C,
         typename OldDom, typename Dom, typename MF>
struct upcast_view_helper<View<C, OldDom>, Dom, MF, false, true>
{ typedef View<C, Dom> type; };



//////////////////////////////////////////////////////////////////////
/// @brief Specialization handling case that domain and mapping function are
///   provided in original's view instantiation.
//////////////////////////////////////////////////////////////////////
template<template<typename...> class View, typename C,
         typename OldDom, typename Dom, typename OldMF, typename MF,
         bool same_dom, bool same_mf>
struct upcast_view_helper<View<C, OldDom, OldMF>, Dom, MF, same_dom, same_mf>
{ typedef View<C, Dom, MF> type; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization handling case that domain and mapping function are
///   provided in original's view instantiation and match the new domain and
///   mapping function.
///
/// @note This specialization is provided to resolve ambiguous class template
///   instantiation errors when the mapping function is not identity. The
///   ambiguity is between specializations
///   upcast_view_helper<View, Dom, MF, true, true> and
///   upcast_view_helper<View<C, OldDom, OldMF>, Dom, MF, same_dom, same_mf>
//////////////////////////////////////////////////////////////////////
template<template<typename...> class View, typename C,
         typename OldDom, typename Dom, typename OldMF, typename MF>
struct upcast_view_helper<View<C, OldDom, OldMF>, Dom, MF, true, true>
{ typedef View<C, OldDom, OldMF> type; };


//////////////////////////////////////////////////////////////////////
/// @brief Domain and mapping function transformation metafunction with
///   purpose same as @ref cast_container_view but employing variadic
///   based optional parameter support.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Dom, typename MF>
struct new_upcast_view;


template<template<typename...> class V, typename Dom, typename MF,
         typename... Args>
struct new_upcast_view<V<Args...>, Dom, MF>
  : upcast_view_helper<V<Args...>, Dom, MF>
{ };


template<typename MapFunc, typename Container,
         bool iv = is_view<Container>::value>
struct compose_map_func
{
  typedef MapFunc type;

  static type apply(MapFunc const& f, Container const&)
  {
    return f;
  }
};


template<typename MapFunc, typename Container>
struct compose_map_func<MapFunc, Container, true>
{
  typedef compose_map_func<
    typename Container::map_function,
    typename Container::view_container_type
  >                                            nested_composer;

  typedef compose_func<
    typename nested_composer::type, MapFunc
  >                                            local_composer;

  typedef typename local_composer::type        type;

  static type apply(MapFunc const& f, Container const& view)
  {
    return local_composer::apply(
      nested_composer::apply(view.mapfunc(), view.container()), f
    );
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction which is implementation of @ref retype_ct_mf.
///   Receives original view, new container type, and new mapping function
///   type. A boolean stating whether mapping function type has changed
///   and a count of the number of optional params passed to view are used
///   to dispatch to an appropriate partial specialization of the class
///   template.
//////////////////////////////////////////////////////////////////////
template<class View, typename NewC, typename NewMF,
         bool b_same_mf, int NumOptionalParams>
struct retype_ct_mf_impl;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for case when only container view type parameter was
///  specified and new mapping function is the same as old mapping function.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename...> class View, typename OldC,
         typename NewC, typename NewMF>
struct retype_ct_mf_impl<View<OldC>, NewC, NewMF, true, 0>
{ typedef View<NewC> type; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for case when only container view type parameter was
///  specified and new mapping function is not the same as old mapping function.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename...> class View, typename OldC,
         typename NewC, typename NewMF>
struct retype_ct_mf_impl<View<OldC>, NewC, NewMF, false, 0>
{
  typedef View<NewC, typename view_traits<View<OldC>>::domain_type, NewMF> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for case when container and domain view type
///   parameter were specified and new mapping function is the same
///   as old mapping function.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename...> class View,
         typename OldC, typename OldDom,
         typename NewC, typename NewMF>
struct retype_ct_mf_impl<View<OldC, OldDom>, NewC, NewMF, true, 1>
{ typedef View<NewC, OldDom> type; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for case when container and domain view type
///   parameter were specified and new mapping function is not the same
///   as old mapping function.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename...> class View,
         typename OldC, typename OldDom,
         typename NewC, typename NewMF>
struct retype_ct_mf_impl<View<OldC, OldDom>, NewC, NewMF, false, 1>
{ typedef View<NewC, OldDom, NewMF> type; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for case when at least container, domain, and
///  mapping function view type parameter were specified.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename...> class View,
         typename OldC, typename OldDom, typename OldMF,
         typename ...OptionalArgs,
         typename NewC, typename NewMF, bool b_same_mf, int num_optional>
struct retype_ct_mf_impl<View<OldC, OldDom, OldMF, OptionalArgs...>,
                    NewC, NewMF, b_same_mf, num_optional>
{ typedef View<NewC, OldDom, NewMF, OptionalArgs...> type; };


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction used by @ref new_cast_container_view to
///   substitute a new container and mapfunc type in a given view type.
//////////////////////////////////////////////////////////////////////
template<class View, typename NewC, typename NewMF>
struct retype_ct_mf;


template<template<typename, typename...> class View,
         typename T, typename NewC, typename NewMF, typename...Params>
struct retype_ct_mf<View<T, Params...>, NewC, NewMF>
  : retype_ct_mf_impl<
      View<T, Params...>, NewC, NewMF,
      std::is_same<
        typename view_traits<View<T, Params...>>::map_function, NewMF
      >::value,
      sizeof...(Params)
    >
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Helper class to change the container type and mapping function
///        of the given @p View to the base container type @p C and the
///        composition of all mapping functions encountered in the @p View
///        containers.
///
///        This is used during the creation of @ref localized_view for a task
///        in a PARAGRAPH.
//////////////////////////////////////////////////////////////////////
template<typename View, typename C>
struct cast_container_view;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref cast_container_view when the given
///        view is tagged to be default.
/// @todo Verify if this specialization is still needed.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct cast_container_view<use_default, C>
{
  typedef use_default type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref cast_container_view when the given
///        view is defined with only one template parameter.
//////////////////////////////////////////////////////////////////////
template<template<typename> class View, typename OldC, typename NewC>
struct cast_container_view<View<OldC>, NewC>
{
  typedef View<NewC> type;

  typename view_traits<type>::map_function
  mapfunc(View<OldC> const& view)
  {
    return view.mapfunc();
  }
};

template<typename View, typename NewC>
struct cast_container_view_preserve_mapfunc;

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref cast_container_view that preserves
///        the mapping function of the original view (doesn't compose
///        it with the mapping functions of the underlying views).
//////////////////////////////////////////////////////////////////////
template<template<typename, typename...> class View,
         typename OldC, typename NewC, typename ...OptionalParams>
struct cast_container_view_preserve_mapfunc<
  View<OldC, OptionalParams...>, NewC
>
{
  typedef View<NewC, OptionalParams...> type;

  typename view_traits<type>::map_function
  mapfunc(View<OldC, OptionalParams...> const& view)
  {
    return view.mapfunc();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref cast_container_view when the given
///        view is defined with three template parameters (container
///        @p OldC, domain @p Dom and mapping function @p Mf).
//////////////////////////////////////////////////////////////////////
template<template<typename, typename, typename> class View,
         typename OldC,
         typename Dom,
         typename Mf,
         typename NewC>
struct cast_container_view<View<OldC, Dom, Mf>, NewC>
{
  typedef compose_map_func<Mf,OldC> mapfunc_composer;
  typedef View<NewC, Dom, typename mapfunc_composer::type> type;

  typename mapfunc_composer::type mapfunc(View<OldC, Dom, Mf> const& view)
  {
    return mapfunc_composer::apply(view.mapfunc(), view.container());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref cast_container_view when the given
///        view (container @p OldC, domain @p Dom, mapping
///        function @p Mf and most derived class @p Der).
//////////////////////////////////////////////////////////////////////
template<template<typename,typename,typename,typename> class View,
         typename OldC,
         typename Dom,
         typename Mf,
         typename Der,
         typename NewC>
struct cast_container_view<View<OldC,Dom,Mf,Der>, NewC>
{
  typedef typename view_traits<View<OldC,Dom,Mf,Der> >::map_function old_mf;
  typedef compose_map_func<old_mf,OldC> mapfunc_composer;
  typedef View<NewC,Dom,typename mapfunc_composer::type,Der> type;

  typename mapfunc_composer::type mapfunc(View<OldC,Dom,Mf,Der> const& view)
  {
    return mapfunc_composer::apply(view.mapfunc(), view.container());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Container transformation meta-function with same purpose as
///   @ref cast_container_view but employing variadic based optional
///   parameter support.
///
/// Also allows preservation of composed views instead of flattening
/// them, i.e. cast<view1<view2<cont>>> produces view1<cast<view2<cont>>>
/// and eventually view1<view2<base_cont>>. Composition types that
/// should be preserved are defined by the meta-function
/// @ref preserve_composition.
//////////////////////////////////////////////////////////////////////
template<typename View, typename C, typename Enable = void>
struct new_cast_container_view;

//////////////////////////////////////////////////////////////////////
/// @brief Helper meta-function providing a public member @p value which is
///   true if the composition containing the provided view type @p View
///   should be preserved during view casting.
//////////////////////////////////////////////////////////////////////
template<typename View,
           bool = is_view<View>::value,
           bool = is_segmented_view<View>::value || is_SLICED_view<View>::value>
struct should_preserve
  : std::false_type
{ };

template<typename View>
struct should_preserve<View, true, true>
  : std::true_type
{ };

template<typename View>
struct should_preserve<View, true, false>
  : should_preserve<typename View::view_container_type>
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Meta-function providing a public member @p value which is
///   true if given view composition @p ViewComp should be preserved during
///   view casting.
///
/// Currently, any composition that contains a @ref segmented_view or a
/// @ref SLICED_view is preserved. The reason for perserving compositions
/// with the @p SLICED_view is technical: the implementation of
/// @ref new_cast_container_view that flattens the views requires each
/// view to be of type <tt>View<Cont, Params...></tt>, where
/// <tt>SLICED_view<Slices, Cont, Params...></tt> doesn't fit.
///
/// @todo Allow compositions containing a @ref SLICED_view be flattened
///   (see details above).
//////////////////////////////////////////////////////////////////////
template<typename ViewComp>
struct preserve_composition
{
  static_assert(is_view<ViewComp>::value,
    "Not a view or a composition of views.");

  using type = should_preserve<typename ViewComp::view_container_type>;
  static constexpr bool value = type::value;
};

//////////////////////////////////////////////////////////////////////
/// @brief @ref new_cast_container_view specialization for the case
///   when view composition should be flattened.
///
/// The type member will be the outer-most view over the underlying
/// base container, with mapping function composed of mapping functions
/// of all the intermediate views.
///
/// Note that views deriving from @ref segmented_view have their own
/// specialization of @ref cast_container_view, hence the case of the
/// outer-most view being one of these is excluded from this specialization.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename...> class View,
         typename OldC, typename NewC, typename ...Params>
struct new_cast_container_view<
  View<OldC, Params...>, NewC,
  typename std::enable_if<
             !is_segmented_view<View<OldC, Params...>>::value &&
             !preserve_composition<View<OldC, Params...>>::value
           >::type >
{
  typedef compose_map_func<
    typename view_traits<View<OldC, Params...>>::map_function,
    typename view_traits<View<OldC, Params...>>::container
  >                                                         mapfunc_composer;

  typedef typename retype_ct_mf<
    View<OldC, Params...>, NewC, typename mapfunc_composer::type
  >::type                                                   type;

  typename mapfunc_composer::type mapfunc(View<OldC, Params...> const& view)
  {
    return mapfunc_composer::apply(view.mapfunc(), view.container());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief @ref new_cast_container_view specialization for the case
///   when view composition should be preserved.
///
/// The type member will be the outer-most view over the casted view at
/// the next level, with its original mapping function.
///
/// Note that views deriving from @ref segmented_view have their own
/// specialization of @ref cast_container_view, hence the case of the
/// outer-most view being one of these is excluded from this specialization.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename...> class View,
         typename OldC, typename NewC, typename ...Params>
struct new_cast_container_view<
  View<OldC, Params...>, NewC,
  typename std::enable_if<
             !is_segmented_view<View<OldC, Params...>>::value &&
             preserve_composition<View<OldC, Params...>>::value
           >::type >
  : cast_container_view_preserve_mapfunc<
      View<OldC, Params...>,
      typename cast_container_view<OldC, NewC>::type
    >
{ };

} // namespace stapl


#endif // STAPL_VIEWS_TYPE_TRAITS_UPCAST_HPP
