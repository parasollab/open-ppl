/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef SIGT_VIEW_HPP
#define SIGT_VIEW_HPP

#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/views/multilevel_slices_view.hpp>
#include <stapl/views/extended_view.hpp>

namespace stapl {

template <typename C, typename InnerSlice, typename OuterSlice>
struct sigt_view;

template <typename C, std::size_t... InnerIndices, std::size_t... OuterIndices>
struct view_traits<sigt_view<C,
                             index_sequence<InnerIndices...>,
                             index_sequence<OuterIndices...>>>
{
  using domain_type = typename container_traits<C>::domain_type;
  using index_type  = typename domain_type::index_type;
  using map_function =
    f_ident<typename container_traits<C>::domain_type::index_type>;
  using container = C;

  using inner_slice_t = index_sequence<InnerIndices...>;
  using outer_slice_t = index_sequence<OuterIndices...>;
  using derived_type  = sigt_view<C, inner_slice_t, outer_slice_t>;

  using value_type = typename container_traits<C>::value_type;

  using base_reference = typename extract_reference_type<C>::type;

  using base_reference_view = typename wrap_with_view<base_reference>::type;

  using extended_base_reference_view =
    typename result_of::make_extended_view<
      3, typename base_reference_view::view_container_type>::type;

  using middle_reference =
    decltype(make_slices_view<extended_base_reference_view, InnerIndices...>(
          view_impl::store_in_frame(),
          std::declval<extended_base_reference_view>()));

  using reference =
    decltype(make_slices_view<middle_reference, OuterIndices...>(
      view_impl::store_in_frame(), std::declval<middle_reference>()));

  using const_reference = reference;
}; // struct view_traits<sigt_view<>>

namespace detail {

template <typename C, std::size_t... InnerIndices, std::size_t... OuterIndices>
struct make_reference<sigt_view<C,
                                index_sequence<InnerIndices...>,
                                index_sequence<OuterIndices...>>>
{
private:
  using View = sigt_view<
    C, index_sequence<InnerIndices...>, index_sequence<OuterIndices...>>;

public:
  using container  = typename view_traits<View>::container;
  using index_t    = typename view_traits<View>::index_type;
  using reference  = typename view_traits<View>::reference;
  using base_ref   = typename C::reference;
  using middle_ref = typename view_traits<View>::middle_reference;
  using extended_base_reference_view =
    typename view_traits<View>::extended_base_reference_view;

  using base_reference = typename view_traits<View>::base_reference;
  using base_reference_cont =  typename base_reference::view_container_type;

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
        make_slices_view<extended_base_reference_view, InnerIndices...>(
          view_impl::store_in_frame(),
          make_extended_view<3>(
            new base_reference_cont(
              view.container()[view.mapfunc()(index)].container()),
            view.num_directions())));
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
        make_slices_view<extended_base_reference_view, InnerIndices...>(
          view_impl::store_in_frame(),
          make_extended_view<3>(
            new base_reference_cont(
              view.container()[view.mapfunc()(index_t(indices...))]
                .container()),
            view.num_directions())));
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
}; // struct make_reference<sigt_view<>>

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
struct sigt_view
  : public core_view<
      C,
      typename view_traits<
        sigt_view<C, InnerSlice, OuterSlice>>::domain_type,
      typename view_traits<
        sigt_view<C, InnerSlice, OuterSlice>>::map_function
    >
{
  size_t m_num_directions;
public:
  STAPL_VIEW_REFLECT_TRAITS(sigt_view)

  typedef index_type                              dimensions_type;
  typedef typename dimension_traits<C>::type      dimension_type;

private:
  typedef core_view<C, domain_type, map_function> base_type;

public:
  sigt_view(view_container_type& vcont,
            domain_type const& dom,
            map_function mfunc = map_function())
    : base_type(vcont, dom, mfunc),
      m_num_directions(0)
  { }

  sigt_view(view_container_type const& vcont,
            domain_type const& dom,
            map_function mfunc = map_function())
    : base_type(vcont, dom, mfunc),
      m_num_directions(0)
  { }

  sigt_view(view_container_type* vcont, size_t num_directions)
    : base_type(vcont, stapl::get_domain(*vcont)),
      m_num_directions(num_directions)
  { }

  sigt_view(view_container_type& vcont, size_t num_directions)
    : base_type(vcont, stapl::get_domain(vcont)),
      m_num_directions(num_directions)
  { }

  sigt_view(view_container_type* vcont,
                         domain_type const& dom,
                         map_function mfunc = map_function())
    : base_type(vcont, dom, mfunc),
      m_num_directions(0)
  { }

  sigt_view(sigt_view const& other)
    : base_type(other.container(), other.domain(), other.mapfunc()),
      m_num_directions(other.num_directions())
  { }

  sigt_view(view_container_type& vcont,
            domain_type const& dom,
            map_function mfunc,
            sigt_view const&)
    : base_type(vcont, dom, mfunc),
      m_num_directions(0)
  { }

  size_t num_directions() const
  { return this->m_num_directions; }

  dimensions_type dimensions(void) const
  { return this->domain().dimensions(); }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_num_directions);
  }

  reference operator[](index_type const& index) const
  { return make_reference(index); }

  template<typename ...Indices>
  reference operator()(Indices... indices) const
  { return make_reference(indices...); }

  reference make_reference(index_type const& index) const
  {
    stapl_assert(this->domain().contains(index),
                 "index out of view domain boundary\n");

    return detail::make_reference<sigt_view>()(*this, index);
  }

  template<typename Index0, typename Index1, typename ...Indices>
  reference
  make_reference(Index0 index0, Index1 index1, Indices... indices) const
  {
    stapl_assert(this->domain().contains(index0, index1, indices...),
                 "index out of view domain boundary\n");

    return detail::make_reference<sigt_view>()(
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
}; // class sigt_view

} // namespace stapl


#endif // SIGT_VIEW_HPP
