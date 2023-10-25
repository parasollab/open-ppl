/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_MULTILEVEL_SLICES_VIEW_HPP
#define STAPL_VIEWS_MULTILEVEL_SLICES_VIEW_HPP

#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/slices_view.hpp>

namespace stapl {

template <typename Container, bool isContainer = is_view<Container>::value>
struct wrap_with_view
{
  using type = multiarray_view<Container>;
};

template <typename View>
struct wrap_with_view<View, true>
{
  using type = View;
};

template <typename C, typename InnerSlice, typename OuterSlice>
struct multilevel_slices_view;

template <typename C, std::size_t... InnerIndices, std::size_t... OuterIndices>
struct view_traits<multilevel_slices_view<C,
                                          index_sequence<InnerIndices...>,
                                          index_sequence<OuterIndices...>>>
{
  using domain_type = typename container_traits<C>::domain_type;
  using index_type = typename domain_type::index_type;
  using map_function =
    f_ident<typename container_traits<C>::domain_type::index_type>;
  using container = C;

  using inner_slice_t = index_sequence<InnerIndices...>;
  using outer_slice_t = index_sequence<OuterIndices...>;
  using derived_type = multilevel_slices_view<C, inner_slice_t, outer_slice_t>;

  using value_type = typename container_traits<C>::value_type;

  using base_reference = typename extract_reference_type<C>::type;

  using base_reference_view = typename wrap_with_view<base_reference>::type;

  using middle_reference =
    decltype(make_slices_view<base_reference_view, InnerIndices...>(
      view_impl::store_in_frame(), std::declval<base_reference_view>()));

  using reference =
    decltype(make_slices_view<middle_reference, OuterIndices...>(
      view_impl::store_in_frame(), std::declval<middle_reference>()));

  using const_reference = reference;
}; // struct view_traits<multilevel_slices_view<>>

namespace detail {

template<typename C, std::size_t ...InnerIndices, std::size_t ...OuterIndices>
struct make_reference<
  multilevel_slices_view<
    C, index_sequence<InnerIndices...>, index_sequence<OuterIndices...>>>
{
private:
  using View = multilevel_slices_view<
    C, index_sequence<InnerIndices...>, index_sequence<OuterIndices...>>;

public:
  using container  = typename view_traits<View>::container;
  using index_t    = typename view_traits<View>::index_type;
  using reference  = typename view_traits<View>::reference;
  using base_ref   = typename C::reference;
  using middle_ref = typename view_traits<View>::middle_reference;

  template<typename... Indices>
  reference operator()(View const& view, Indices const&... indices) const
  {
    return make_ref_helper<base_ref>()(view, indices...);
  }

private:
  template<typename V>
  struct make_ref_helper
  {
    //////////////////////////////////////////////////////////////////////
    /// @brief creates two nested levels of slices_view over the each element of
    ///        the nested view, @c view indexed by the @c index.
    ///        created by referencing the @c view
    /// @tparam View nested view which its elements will be sliced in two level.
    /// @tparam InnerSlice the tuple of indices for sliced dimensions in the
    ///         inner slices_view
    /// @tparam OuterSlice the tuple of indices for sliced dimensions in the
    ///         outer slices_view
    //////////////////////////////////////////////////////////////////////
    reference operator()(View const& view, index_t const& index) const
    {
      return make_slices_view<middle_ref, OuterIndices...>(
        view_impl::store_in_frame(),
        make_slices_view<base_ref, InnerIndices...>(
          view_impl::store_in_frame(),
          view.container()[view.mapfunc()(index)]));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief creates two nested levels of slices_view over the each element of
    ///        the nested view, @c view indexed by the @c indices.
    ///        created by referencing the @c view
    /// @tparam View nested view which its elements will be sliced in two level.
    /// @tparam InnerSlice the tuple of indices for sliced dimensions in the
    ///         inner slices_view
    /// @tparam OuterSlice the tuple of indices for sliced dimensions in the
    ///         outer slices_view
    //////////////////////////////////////////////////////////////////////
    template <typename... Indices>
    reference operator()(View const& view, Indices... indices) const
    {
      return make_slices_view<middle_ref, OuterIndices...>(
        view_impl::store_in_frame(),
        make_slices_view<base_ref, InnerIndices...>(
          view_impl::store_in_frame(),
          view.container()[view.mapfunc()(index_t(indices...))]));
    }
  };

  template<typename Q>
  struct make_ref_helper<proxy<Q, view_impl::gid_accessor<View>>>
  { };

  template<typename Q>
  struct make_ref_helper<proxy<Q, index_accessor<View>>>
  { };

  template<typename Q,typename A>
  struct make_ref_helper<proxy<Q,A>>
  { };

  template<typename Q,typename C1>
  struct make_ref_helper<proxy<Q, local_accessor<C1>>>
  { };

  template<typename Q,typename R>
  struct make_ref_helper<proxy<Q, trivial_accessor<R>>>
  { };

  template<typename Q, typename C1>
  struct make_ref_helper<proxy<Q, stl_vector_accessor<C1>>>
  { };
}; // struct make_reference<multilevel_slices_view<>>

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Defines a view over the passed view, which accessing each
///        elements results a two nested level of slices_view, over that
///        accessed element according to the indices passed for outer and inner
///        @slices_view.
///
/// @tparam C Container type.
/// @tparam InnerSlice the tuple of indices for sliced dimensions in the inner
///         slices_view
/// @tparam OuterSlice the tuple of indices for sliced dimensions in the outer
///         slices_view
//////////////////////////////////////////////////////////////////////
template<typename C, typename InnerSlice, typename OuterSlice>
struct multilevel_slices_view
  : public core_view<
      C,
      typename view_traits<
        multilevel_slices_view<C, InnerSlice, OuterSlice>>::domain_type,
      typename view_traits<
        multilevel_slices_view<C, InnerSlice, OuterSlice>>::map_function
    >
{
public:
  STAPL_VIEW_REFLECT_TRAITS(multilevel_slices_view)

  typedef index_type                              dimensions_type;
  typedef typename dimension_traits<C>::type      dimension_type;

private:
  typedef core_view<C, domain_type, map_function> base_type;

public:
  multilevel_slices_view(view_container_type& vcont,
                         domain_type const& dom,
                         map_function mfunc = map_function())
    : base_type(vcont, dom, mfunc)
  { }

  multilevel_slices_view(view_container_type const& vcont,
                         domain_type const& dom,
                         map_function mfunc = map_function())
    : base_type(vcont, dom, mfunc)
  { }

  template <typename... Optional>
  multilevel_slices_view(view_container_type* vcont, Optional&&...)
    : base_type(vcont, stapl::get_domain(*vcont))
  { }

  template <typename... Optional>
  multilevel_slices_view(view_container_type& vcont, Optional&&...)
    : base_type(vcont, stapl::get_domain(vcont))
  { }

  multilevel_slices_view(view_container_type* vcont,
                         domain_type const& dom,
                         map_function mfunc = map_function())
    : base_type(vcont, dom, mfunc)
  { }

  multilevel_slices_view(multilevel_slices_view const& other)
    : base_type(other.container(), other.domain(), other.mapfunc())
  { }

  multilevel_slices_view(view_container_type& vcont,
                         domain_type const& dom,
                         map_function mfunc,
                         multilevel_slices_view const&)
    : base_type(vcont, dom, mfunc)
  { }

  dimensions_type dimensions(void) const
  { return this->domain().dimensions(); }

  void define_type(typer& t)
  { t.base<base_type>(*this); }

  reference operator[](index_type const& index) const
  { return make_reference(index); }

  template<typename ...Indices>
  reference operator()(Indices... indices) const
  { return make_reference(indices...); }

  reference make_reference(index_type const& index) const
  {
    stapl_assert(this->domain().contains(index),
                 "index out of view domain boundary\n");

    return detail::make_reference<multilevel_slices_view>()(*this, index);
  }

  template<typename Index0, typename Index1, typename ...Indices>
  reference
  make_reference(Index0 index0, Index1 index1, Indices... indices) const
  {
    stapl_assert(this->domain().contains(index0, index1, indices...),
                 "index out of view domain boundary\n");

    return detail::make_reference<multilevel_slices_view>()(
      *this, index0, index1, indices...);
  }

  std::vector<domain_type>
  local_domain(runtime::location_id loc_id, std::size_t num_locs) const
  {
    std::vector<domain_type> v;

    auto const& cm = this->container().distribution().container_manager();

    if (cm.size() == 0)
      v.push_back(domain_type());
    else
    {
      v.reserve(cm.size());

      for (auto&& bc : cm)
        v.push_back(bc.domain());
    }

    return v;
  }
}; // class multilevel_slices_view

} // namespace stapl

#endif // ifndef STAPL_VIEWS_MULTILEVEL_SLICES_VIEW_HPP
