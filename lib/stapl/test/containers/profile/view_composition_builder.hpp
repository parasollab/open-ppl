/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_TEST_VIEWS_PROFILE_VIEW_COMPOSITION_BUILDER_HPP
#define STAPL_TEST_VIEWS_PROFILE_VIEW_COMPOSITION_BUILDER_HPP

#include <stapl/views/segmented_view.hpp>
#include <stapl/views/type_traits/is_segmented_view.hpp>
#include <stapl/views/type_traits/is_view.hpp>
#include <stapl/views/type_traits/underlying_container.hpp>
#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/apply.hpp>
#include <stapl/utility/pack_ops.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction used to build a persistent composed views
///    hierarchy by recursively constructing the views at each level
///    on heap.
///
/// The view at each level takes ownership of the view constructed at the
/// previous level and possibly also of the bottom-most container, depending
/// on whether a pointer or a reference to it has been passed to the member
/// function @ref view_composition_impl::construct.
//////////////////////////////////////////////////////////////////////
template<
  typename T,
  bool ViewContainerIsView = is_view<
    typename T::view_container_type
  >::type::value,
  bool IsSegmented = is_segmented_view<T>::type::value
>
struct view_composition_impl
{
  using cont_t = underlying_container_t<T>;
  using recursive = view_composition_impl<typename T::view_container_type>;

  static T* construct(cont_t& cont)
  { return new T(recursive::construct(cont)); }

  static T* construct(cont_t* cont)
  { return new T(recursive::construct(cont)); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for composed segmented_views
//////////////////////////////////////////////////////////////////////
template<typename T>
struct view_composition_impl<T, true, true>
{
  using cont_t = underlying_container_t<T>;
  using recursive = view_composition_impl<
    typename T::view_container_type::view_container_type
  >;

  static T* construct(cont_t& cont)
  { return new T(view_impl::store_in_frame(), recursive::construct(cont)); }

  static T* construct(cont_t* cont)
  { return new T(view_impl::store_in_frame(), recursive::construct(cont)); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization when we've reached the bottom.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct view_composition_impl<T, false, false>
{
  using cont_t = underlying_container_t<T>;

  static T* construct(cont_t& cont)
  { return new T(cont); }

  static T* construct(cont_t* cont)
  { return new T(cont); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction used to build the tuple of boost::optional
///   objects by successively calling the methods from the tuple
///   @p MakeFnsTuple on the result of the previous call.
///
/// Used to build the levels of the hierarchy of composed views in
/// @ref functional_view_composition.
//////////////////////////////////////////////////////////////////////
template<size_t idx, size_t depth>
struct create_composed_views_tuple
{
  template<typename ComposedViews, typename MakeFnsTuple>
  static void apply(ComposedViews& composed_views, MakeFnsTuple const& make_fns)
  {
    get<idx>(composed_views) =
      get<idx-1>(make_fns)(get<idx-1>(composed_views).get());

    create_composed_views_tuple<idx+1, depth>::apply(composed_views, make_fns);
  }
};

template<size_t depth>
struct create_composed_views_tuple<depth, depth>
{
  template<typename ComposedViews, typename MakeFnsTuple>
  static void apply(ComposedViews& composed_views, MakeFnsTuple const& make_fns)
  {
    get<depth>(composed_views) =
      get<depth-1>(make_fns)(get<depth-1>(composed_views).get());
  }
};

/// Clear an intermediate view wrapped in a boost::optional.
struct clear_view
{
  template<typename View>
  void operator()(boost::optional<View>& vw) const
  { vw = boost::none; }
};

//////////////////////////////////////////////////////////////////////
/// @brief Clear all intermediate views from the tuple of boost::optional
/// objects, except for the first one (which represents the base underlying
/// container for the whole hierarchy and has been passed from outside).
//////////////////////////////////////////////////////////////////////
struct clear_composed_views
{
  template<typename Base, typename... Views>
  void operator()(Base&& base, Views&&... views) const
  {
    pack_ops::functional::for_each_(clear_view(),std::forward<Views>(views)...);
  }
};

} // namespace detail

template<typename Base, typename MakeFnsTuple,
         size_t idx = tuple_size<MakeFnsTuple>::value>
struct view_composition_traits
{
  using make_fn_type = typename tuple_element<idx-1, MakeFnsTuple>::type;
  using next = view_composition_traits<Base, MakeFnsTuple, idx-1>;
  using view_type =
    typename std::result_of<make_fn_type(typename next::view_type&)>::type;

  using tuple_type = typename result_of::tuple_cat<
    typename next::tuple_type, tuple<boost::optional<view_type>>
  >::type;
};

template<typename Base, typename MakeFnsTuple>
struct view_composition_traits<Base, MakeFnsTuple, 0>
{
  using view_type = Base;
  using tuple_type = tuple<boost::optional<Base>>;
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction used to build a persistent composed views
///    hierarchy by recursively constructing the views at each level
///    on heap.
///
/// Uses the @p view(underlying*) constructor at each level.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct view_composition
  : public detail::view_composition_impl<T>
{
  static_assert(is_view<T>::type::value, "View required for view composition.");
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction used to build a persistent composed views
///    hierarchy by recursively constructing the views at each level
///    and storing them in a member tuple of boost::optionals.
///
/// A function from the tuple @p MakeFnsTuple is called at each level
/// to obtain the view at that level -- typically, these are the
/// @p make_xxx_views functions that facilitate constructing the
/// parameterized views.
///
/// The member tuple #m_composed_views contains all the intermediate views,
/// starting from the bottom-most underlying container of type @p Base.
/// E.g., for the composition view1(view2(view3(cont))), #m_composed_views
/// will be {cont, view3(cont), view2(view3(cont)), view1(view2(view3(cont)))}.
/// Once the views are not needed the method clear() should be called to
/// clear the intermediate boost::optional objects.
//////////////////////////////////////////////////////////////////////
template<typename Base, typename MakeFnsTuple>
class functional_view_composition
{
  static size_t constexpr depth = tuple_size<MakeFnsTuple>::value;

  using comp_traits = view_composition_traits<Base, MakeFnsTuple>;

  typename comp_traits::tuple_type m_composed_views;

public:
  using view_type = typename comp_traits::view_type;

  functional_view_composition(Base const& base)
  { get<0>(m_composed_views) = base; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the intermediate views using the "make_xxx_view"
  ///   functions from the @a make_fns tuple and return a pointer to the
  ///   outer-most view representing the whole hierarchy.
  //////////////////////////////////////////////////////////////////////
  view_type* create(MakeFnsTuple const& make_fns)
  {
    detail::create_composed_views_tuple<1, depth>::apply(
      m_composed_views, make_fns);

    return get<depth>(m_composed_views).get_ptr();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears all the intermediate views in the hierarchy, except
  ///   for the base underlying container for the whole hierarchy that
  ///   has been passed from outside.
  //////////////////////////////////////////////////////////////////////
  void clear()
  { tuple_ops::apply(detail::clear_composed_views(), m_composed_views); }
};

} // namespace stapl

#endif // STAPL_TEST_VIEWS_PROFILE_VIEW_COMPOSITION_BUILDER_HPP
