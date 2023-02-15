/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_EXTENDED_VIEW_HPP
#define STAPL_VIEWS_EXTENDED_VIEW_HPP

#include <stapl/views/view_traits.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/metadata/coarsening_traits.hpp>
#include <stapl/views/metadata/projection/extended_view.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Static function that is identity operation unless
/// @p Index and @p FixedIndex are the same value.
//////////////////////////////////////////////////////////////////////
template <size_t Index, size_t FixedIndex, typename T>
struct rewire_check
{
  static T apply(T index)
  { return index; }
};


template <size_t Index, typename T>
struct rewire_check<Index, Index, T>
{
  static T apply(T)
  { return T{0}; }
};

template <size_t FixedIndex, typename Tuple>
struct less_num;

template <size_t FixedIndex, typename E0, typename... E>
struct less_num<FixedIndex, tuple<E0, E...>>
{
  static constexpr size_t value =
    ((E0::value < FixedIndex) ? 1 : 0) +
    less_num<FixedIndex, tuple<E...>>::value;
};

template <size_t FixedIndex>
struct less_num<FixedIndex, tuple<>>
{
  static constexpr size_t value = 0;
};

//////////////////////////////////////////////////////////////////////
/// @brief Mapping function used for @see make_extended_view, which
///        rewires the index at a single dimension to be 0.
///
/// @tparam FixedIndex Which dimension is being rewired.
/// @tparam T The gid type
//////////////////////////////////////////////////////////////////////
template<size_t FixedIndex, typename T,
         typename = make_index_sequence<tuple_size<T>::value>>
struct rewire_dimension;


template<size_t FixedIndex, typename T, std::size_t... GIDIndices>
struct rewire_dimension<FixedIndex, T, index_sequence<GIDIndices...>>
{
  using index_type = T;
  using gid_type   = T;

  using fixed_index_t = std::integral_constant<size_t, FixedIndex>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the mapping function type of a view after
  ///        deep slice on the view
  ///
  /// @tparam Slices The indices to slice on this deep slice
  /// @tparam Fixed The type of element that will be used to specify the
  ///               fixed values. Typically a tuple of size |Slices|.
  //////////////////////////////////////////////////////////////////////
  template<typename Slices, typename Fixed>
  struct slice_type
  {
    static constexpr size_t new_fixed_index =
      FixedIndex - less_num<FixedIndex, Slices>::value;

    using new_gid_type =
      homogeneous_tuple_type_t<tuple_size<T>::value - tuple_size<Slices>::value,
                               size_t>;

    using type = rewire_dimension<new_fixed_index, new_gid_type>;
  };

  gid_type operator()(index_type const& idx) const
  {
    return gid_type(
      rewire_check<
        GIDIndices, FixedIndex,
        typename tuple_element<GIDIndices, gid_type>::type
      >::apply(get<GIDIndices>(idx))...
    );
  }

  template<typename RefGetter, typename ...Indices>
  typename std::result_of<
    RefGetter(typename tuple_element<GIDIndices, gid_type>::type...)
  >::type
  apply_get(RefGetter const& get_reference, Indices... indices) const
  {
    return get_reference(
      rewire_check<
        GIDIndices, FixedIndex,
        typename tuple_element<GIDIndices, gid_type>::type
      >::apply(indices)...
    );
  }
};  // struct rewire_dimension

} // namespace detail

namespace result_of {

template <size_t D, typename Container>
struct make_extended_view
{
  using domain_type = typename Container::domain_type;
  using gid_type = typename domain_type::gid_type;
  using mapping_function_type = detail::rewire_dimension<D, gid_type>;
  using type = multiarray_view<Container, domain_type, mapping_function_type>;
};

} // namespace result_of


namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of the coarsening_traits struct for extended_view.
///
/// The specialization defines the correct metadata extraction policy
/// for extended_view.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename Domain, size_t FixedDim, typename Index>
struct coarsening_traits<multiarray_view<Container, Domain,
                           stapl::detail::rewire_dimension<FixedDim, Index>>>
{
  using View = multiarray_view<Container, Domain,
                 stapl::detail::rewire_dimension<FixedDim, Index>>;

  template<typename Part>
  struct construct_projection
  {
    typedef extended_view_metadata_projection<View, Part> type;
  };
};

} // namespace metadata


namespace impl {

//////////////////////////////////////////////////////////////////////
/// @brief implementation used when the extended view doesn't take
///        the ownership of the @c ct.
///
/// @tparam D the dimension to extend
/// @param  ct The multiarray view
/// @param  size The size to extend dimension D
//////////////////////////////////////////////////////////////////////
template <size_t D, typename Container>
typename result_of::make_extended_view<D, Container>::type
make_extended_view_impl(std::false_type, Container const& ct, std::size_t size)
{
  using view_type = typename result_of::make_extended_view<D, Container>::type;
  using domain_type = typename view_traits<view_type>::domain_type;

  static_assert(dimension_traits<Container>::type::value > D,
    "The dimension to extend along should be less than the total number "
    "of dimensions");

  stapl_assert(get<D>(ct.domain().dimensions()) == 1,
    "Extended view can only extend a dimension that has 1 element.");

  // Create a new domain that is the same as ct's domain, but
  // with the D'th dimension replaced with size (using the domain constructor
  // that receives first and last so the flag can be set to indicate the domain
  // is not the same size as the underlying container).
  auto new_size = ct.domain().last();
  get<D>(new_size) = size-1;
  domain_type dom(ct.domain().first(), new_size, false);

  return view_type(ct, dom);
}

//////////////////////////////////////////////////////////////////////
/// @brief implementation used when the extended view takes the ownership
///        of the @c ct.
///
/// @tparam D the dimension to extend
/// @param  ct The multiarray view
/// @param  size The size to extend dimension D
//////////////////////////////////////////////////////////////////////
template <size_t D, typename Container>
typename result_of::make_extended_view<
  D, typename std::decay<Container>::type>::type
make_extended_view_impl(std::true_type, Container* ct, std::size_t size)
{
  using view_type = typename result_of::make_extended_view<D, Container>::type;
  using domain_type = typename view_traits<view_type>::domain_type;

  static_assert(dimension_traits<Container>::type::value > D,
    "The dimension to extend along should be less than the total number "
    "of dimensions");

  stapl_assert(get<D>(ct->domain().dimensions()) == 1,
    "Extended view can only extend a dimension that has 1 element.");

  // Create a new domain that is the same as ct's domain, but
  // with the D'th dimension replaced with size (using the domain constructor
  // that receives first and last so the flag can be set to indicate the domain
  // is not the same size as the underlying container).
  auto new_size = ct->domain().last();
  get<D>(new_size) = size-1;
  domain_type dom(ct->domain().first(), new_size, false);

  return view_type(ct, dom);
}

} // namespace impl

//////////////////////////////////////////////////////////////////////
/// @brief Make an extended view, which extends a single dimension
///        of a multiarray view to a given size.
///
///        For example, if a multiarray view has dimensions (a,b,1,d),
///        then an extended view in the dimension 2 to size c, would
///        produced a view of dimensions (a,b,c,d) and where every
///        access from (-,-,[0...c-1],-) will be mapped to (-,-,0,-).
///
///        This view assumes that the dimension to be extended initially
///        is of size 1.
///
/// @tparam D the dimension to extend
/// @param  ct The multiarray view
/// @param  size The size to extend dimension D
//////////////////////////////////////////////////////////////////////
template <size_t D, typename Container>
auto make_extended_view(Container&& ct, std::size_t size)
  -> decltype(impl::make_extended_view_impl<D>(std::is_pointer<Container>(),
                                 std::forward<Container>(ct), size))
{
  return impl::make_extended_view_impl<D>(std::is_pointer<Container>(),
                                       std::forward<Container>(ct), size);
}

} // namespace stapl

#endif
