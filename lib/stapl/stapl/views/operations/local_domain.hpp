/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_OPERATIONS_COMPUTE_LOCAL_DOMAIN_HPP
#define STAPL_VIEWS_OPERATIONS_COMPUTE_LOCAL_DOMAIN_HPP

#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/containers/partitions/block_partition.hpp>

namespace stapl {

namespace view_operations {

//////////////////////////////////////////////////////////////////////
/// @brief Return a sub domain of the view's domain taken by indexing
/// into a volumetric partitioning with the current location_ids for
/// non 1D views.
//////////////////////////////////////////////////////////////////////
template <typename View>
typename View::domain_type default_local_domain(std::true_type,
                                                View const& view,
                                                runtime::location_id loc_id,
                                                std::size_t num_locs)
{
  static constexpr size_t ndims =
    tuple_size<typename tuple_ops::result_of::ensure_tuple<
      typename View::index_type>::type>::value;

  using domain_t    = typename View::domain_type;
  using traversal_t = typename domain_t::traversal_type;

  using partition_type = multiarray_impl::block_partition<traversal_t>;

  partition_type part(domain_t(view.domain().dimensions()),
                        multiarray_impl::make_multiarray_size<ndims>()(
                          num_locs));

  auto idx = part.domain().first();
  idx      = part.domain().advance(idx, loc_id);
  return part.domain().contains(idx) ? part[idx] : domain_t();
}

//////////////////////////////////////////////////////////////////////
/// @brief Return a sub domain of the view's domain taken by indexing
/// into a volumetric partitioning with the current location_id for
/// 1D views.
//////////////////////////////////////////////////////////////////////
template <typename View>
typename View::domain_type default_local_domain(std::false_type,
                                                View const& view,
                                                runtime::location_id loc_id,
                                                std::size_t num_locs)
{
  using domain_t = typename View::domain_type;
  using partition_type = balanced_partition<domain_t>;

  partition_type part = balanced_partition<domain_t>(
    domain_t(view.domain().dimensions()), num_locs);

  auto idx = part.domain().first();
  idx      = part.domain().advance(idx, loc_id);
  return part.domain().contains(idx) ? part[idx] : domain_t();
}

//////////////////////////////////////////////////////////////////////
/// @brief Helper function for computing the local domain that by default
/// is partition agnostic of the underlying container distribution.
/// Specializations of the class template addresses cases where we do better
/// given what we know about the distribution (ideally we would like the
/// local domain to be that of the elements stored on this affinity).
//////////////////////////////////////////////////////////////////////
template <typename View>
struct compute_local_domain
{
  static std::vector<typename View::domain_type>
  apply(View const& view, runtime::location_id loc_id, std::size_t num_locs)
  {
    static constexpr size_t ndims =
      tuple_size<typename tuple_ops::result_of::ensure_tuple<
        typename View::index_type>::type>::value;

    std::vector<typename View::domain_type> v;

    v.emplace_back(default_local_domain(
      std::integral_constant<bool, (ndims > 1)>(), view, loc_id, num_locs));

    return v;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Performs pieceise comparison of first and last elements of
/// two @ref indexed_domain objects to determine if they are equivalent.
//////////////////////////////////////////////////////////////////////
template<int N, typename = make_index_sequence<N>>
struct domain_equal;

template<int N, std::size_t ...Indices>
struct domain_equal<N, index_sequence<Indices...>>
{
  template<typename D1, typename D2>
  static bool apply(D1 const& lhs, D2 const& rhs)
  {
    return pack_ops::functional::and_(
             (get<Indices>(lhs.first()) == get<Indices>(rhs.first()))...)
           && pack_ops::functional::and_(
             (get<Indices>(lhs.last()) == get<Indices>(rhs.last()))...);
  }
};

template<std::size_t ...Indices>
struct domain_equal<1, index_sequence<Indices...>>
{
  template<typename D1, typename D2>
  static bool apply(D1 const& lhs, D2 const& rhs)
  {
    return lhs.first() == rhs.first() && lhs.last() == rhs.last();
  }
};

} // namespace view_operations
} // namespace stapl

#endif // STAPL_VIEWS_OPERATIONS_COMPUTE_LOCAL_DOMAIN_HPP
