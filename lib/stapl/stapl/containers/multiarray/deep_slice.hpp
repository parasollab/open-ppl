/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// base_container of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIARRAY_DEEP_SLICE_HPP
#define STAPL_CONTAINERS_MULTIARRAY_DEEP_SLICE_HPP

#include <stapl/utility/tuple.hpp>
#include <stapl/utility/pack_ops.hpp>
#include <stapl/views/sliced_view.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/containers/distribution/base_container_metadata.hpp>
#include <stapl/utility/tuple/ensure_tuple.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to compute the last element of a domain based on
///        its start and size.
///
/// @ptparam T The size type (either tuple or std::size_t)
//////////////////////////////////////////////////////////////////////
template<typename T>
struct last_of
{
  T operator()(T const& first, T const& size) const
  {
    return tuple_ops::transform(first, size,
      [](std::size_t start, std::size_t sz) { return start + sz - 1; }
    );
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for std::size_t.
//////////////////////////////////////////////////////////////////////
template<>
struct last_of<std::size_t>
{
  std::size_t operator()(std::size_t first, std::size_t size) const
  {
    return first + size - 1;
  }
};

} // namespace detail

template<typename Slices, typename BC, typename Linearizer>
struct multiarray_bc_slice;

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute whether or not a type is a deep slice.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct is_deep_slice : std::false_type {};

template<typename... Args>
struct is_deep_slice<multiarray_bc_slice<Args...>> : std::true_type {};


//////////////////////////////////////////////////////////////////////
/// @brief A lightweight wrapper around a multiarray base container's
///   sequential storage pointer, with some of the dimensions hoisted out.
///
/// @tparam Slices All dimensions that are being sliced
/// @tparam BC The base container type
/// @tparam Linearizer The linearizer that maps from n-d to n-|slices|-d
//////////////////////////////////////////////////////////////////////
template<typename Slices, typename BC, typename Linearizer>
struct multiarray_bc_slice
{
  using container_type  = typename BC::container_type;
  using iterator        = typename container_type::iterator;
  using accessor_type   = typename BC::accessor_type;
  using reference       = typename BC::reference;
  using const_reference = typename BC::const_reference;
  using value_type      = typename BC::value_type;
  using traversal_type  = typename Linearizer::traversal_type;

  using loc_dist_metadata = base_container_metadata<multiarray_bc_slice>;

  /// There are as many dimensions as there are in the index of the new
  /// Linearizer
  using dimension_type = std::integral_constant<int,
    dimension_traits<typename Linearizer::index_type>::type::value
  >;

  using domain_type = typename detail::SLICED_view_domain<
    dimension_type::value, std::size_t
  >::type;
  using gid_type = typename domain_type::gid_type;

  using cid_type = gid_type;

private:
  iterator   m_start;
  Linearizer m_linear_mf;
  BC*        m_ct_ptr;

  template<typename Index, std::size_t... Indices>
  reference make_reference_impl(Index const& gid, index_sequence<Indices...>)
  {
    return make_reference(stapl::get<Indices>(gid)...);
  }

public:
  STAPL_USE_MANAGED_ALLOC(multiarray_bc_slice)

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a deep slice.
  ///
  /// @param start An iterator to the new start in the sequential storage
  /// @param linear_mf A new linearization for the reduced dimensionality
  ///                  base container
  /// @param ct_ptr Pointer to the original base_container
  //////////////////////////////////////////////////////////////////////
  multiarray_bc_slice(iterator start, Linearizer const& linear_mf, BC* ct_ptr)
    : m_start(start), m_linear_mf(linear_mf), m_ct_ptr(ct_ptr)
  { }

  multiarray_bc_slice(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief @copybrief multiarray_base_container::local_position
  //////////////////////////////////////////////////////////////////////
  template<typename... Indices>
  size_t local_position(Indices const&... i) const
  {
    return m_linear_mf(i...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief @copybrief multiarray_base_container::make_reference
  //////////////////////////////////////////////////////////////////////
  template<typename... Indices>
  reference make_reference(Indices const&... i)
  {
    return reference(accessor_type(m_ct_ptr, m_start + local_position(i...)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief @copybrief multiarray_base_container::operator()
  //////////////////////////////////////////////////////////////////////
  template<typename... Indices>
  reference operator()(Indices const&... i)
  {
    return make_reference(i...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief @copybrief multiarray_base_container::operator[]
  ///
  /// @todo A specialization for gid_type of size_t is needed to eliminate
  /// the construction of a tuple in this method.
  //////////////////////////////////////////////////////////////////////
  reference operator[](gid_type const& gid)
  {
    return make_reference_impl(tuple_ops::ensure_tuple(gid),
             make_index_sequence<dimension_type::value>());
  }

  reference get_element(gid_type const& gid)
  {
    return make_reference_impl(tuple_ops::ensure_tuple(gid),
             make_index_sequence<dimension_type::value>());
  }

  void set_element(gid_type const& gid, value_type const& value)
  {
    make_reference_impl(tuple_ops::ensure_tuple(gid),
             make_index_sequence<dimension_type::value>()) = value;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Domain of this deep slice. It has the same dimensionality as
  ///        the new linearizer.
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    auto const first = m_linear_mf.m_original_first;
    auto const last = detail::last_of<typename domain_type::index_type>()(
      first, m_linear_mf.m_original_size
    );

    return {first, last, true};
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Number of elements represented by this slice
  //////////////////////////////////////////////////////////////////////
  std::size_t size() const
  {
    return this->domain().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief @copybrief multiarray_base_container::apply_get
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  auto apply_get(gid_type const& gid, F&& f)
    -> decltype(f(std::declval<value_type>()))
  {
    value_type& ref = *(m_start + m_linear_mf(gid));
    return f(ref);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief @copybrief multiarray_base_container::apply_set
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void apply_set(gid_type const& gid, F&& f)
  {
    value_type& ref = *(m_start + m_linear_mf(gid));
    f(ref);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the type of a new slice on this deep
  ///         slice.
  ///
  /// @tparam Slices The indices to slice on this deep slice
  /// @tparam Fixed The type of element that will be used to specify the
  ///               fixed values. Typically a tuple of size |Slices|.
  //////////////////////////////////////////////////////////////////////
  template<typename NewSlices, typename Fixed>
  struct slice_type
  {
    using type = typename std::decay<
      decltype(std::declval<
        multiarray_bc_slice<NewSlices, BC,
          decltype(m_linear_mf.template slice<NewSlices>(
            std::declval<Fixed>()).second
          )
        >>
      ())>::type;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Create another deep slice by slicing off dimensions specified
  ///        in NewSlices.
  ///
  /// @tparam Slices The indices to slice on this deep slice
  /// @tparam Fixed The type of element that will be used to specify the
  ///               fixed values. Typically a tuple of size |Slices|.
  //////////////////////////////////////////////////////////////////////
  template<typename NewSlices, typename Fixed>
  typename slice_type<NewSlices, Fixed>::type
  slice(Fixed const& fixed)
  {
    auto lin_pair = m_linear_mf.template slice<NewSlices>(fixed);

    using slice_t = multiarray_bc_slice<
      NewSlices, BC, decltype(lin_pair.second)
    >;

    return slice_t(m_start + lin_pair.first, lin_pair.second, m_ct_ptr);
  }

  void define_type(typer& t)
  {
    stapl_assert(0, "Attempting to pack a deep slice.");
  }

}; // struct multiarray_bc_slice

} // namespace stapl

#endif // STAPL_CONTAINERS_MULTIARRAY_DEEP_SLICE_HPP
