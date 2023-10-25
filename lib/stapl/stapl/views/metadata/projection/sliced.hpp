/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_PROJECTION_SLICED_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_SLICED_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/views/metadata/extraction/container_extractor_base.hpp>

#include "slice_common.hpp"

#include <utility>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor used to project the domains in the given
///        locality metadata (@c P) to the domain of the given @c
///        View. The result is projected metadata locality
///        information.
///
/// This helper functor is invoked when the given @c View is a
/// @ref SLICED_view.
///
/// @todo operator should return a shared_ptr.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
class sliced_projection
{
  typedef typename std::remove_pointer<typename P::second_type>::type
    lower_md_cont_type;
  typedef typename View::slices_type                           slices_type;
  typedef typename View::domain_type                           domain_type;
  typedef typename lower_md_cont_type::value_type::component_type
    component_type;
  typedef typename domain_type::index_type                     index_type;
  typedef typename dimension_traits<View>::type                dimension_type;

  // indicates whether the target SLICED_view
  // is multidimensional
  static int constexpr dim = dimension_type::value;
  typedef std::integral_constant<bool, (dim > 1)>              is_nd;

public:
  typedef metadata_entry<
    domain_type, component_type, index_type
  >                                                            md_entry_type;
  typedef typename detail::select_container_type<
    md_entry_type, dim
  >::type                                                      md_cont_type;

  typedef std::pair<bool, md_cont_type*>                       return_type;

private:
  template<typename Dims>
  md_cont_type* construct_return_type(Dims const& dims, std::true_type) const
  {
    using return_type_partition = typename md_cont_type::partition_type;
    return new md_cont_type(return_type_partition(domain_type(dims),
                 get_num_locations() > 1 ? dims : homogeneous_tuple<dim>(1ul)));
  }

  template<typename Dims>
  md_cont_type* construct_return_type(Dims const& dims, std::false_type) const
  {
    return new md_cont_type(dims);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Project the metadata entries extracted from the underlying
  ///        multidimensional container into the domain of the @ref SLICED_view.
  ///
  /// The algorithm starts by partitioning the sliced_view's domain according to
  /// the underlying multiarray by slicing each multiarray subdomain in the
  /// extracted metadata container. For each multiarray subdomain, it checks if
  /// the corresponding sliced_view's partition is mapped by the sliced_view's
  /// mapping function to that subdomain. If it does, the partition (and other
  /// information from the metadata entry) is added to the resulting projected
  /// metadata container. The loop over the entries of the extracted metadata
  /// container then continues to find the next partition of the sliced_view's
  /// domain and it's corresponding metadata_entry (or the metadata entry with
  /// the subdomain into which the current sliced_view's partition maps if the
  /// above check failed — the subdomain of a slice can potentially map to
  /// multiple subdomains of the full-dimensioned container).
  ///
  /// @param view The target view
  /// @param part Container of extracted metadata entries
  /// @param part_is_fixed Indicates if part container is fixed-size
  //////////////////////////////////////////////////////////////////////
  return_type operator()(View* view, lower_md_cont_type* part,
                         bool part_is_fixed = true)
  {
    using domain_bounds_t = std::pair<index_type, index_type>;

    // key: bounds of a sliced_view's subdomain
    // val: flag indicating whether this subdomain has already been added to the
    //      projected metadata container
    std::map<domain_bounds_t, bool> visited_subdomains;

    // partitions layout for the original container
    const auto part_dims_orig = part->dimensions();

    // determine if the whole slice is contained in one partition
    const bool full_slice = part_has_full_slice<slices_type>(part_dims_orig);

    // partitions layout for the container with projected metadata
    const auto part_dims_proj = tuple_ops::discard<slices_type>(part_dims_orig);

    // construct the container for the projected metadata, using the partition
    // obtained by slicing the partition of the source container of extracted
    // metadata entries
    md_cont_type* res = construct_return_type(part_dims_proj, is_nd());

    typename View::map_func_type mf(view->mapfunc());

    for (auto&& md : *part)
    {
      // domain of the metadata entry
      auto const& dom = md.domain();

      // current partition of the sliced_view's domain
      const domain_type slice_dom(
        tuple_ops::discard<slices_type>(dom.first()),
        tuple_ops::discard<slices_type>(dom.last())
      );

      auto slice_dom_bounds_info = visited_subdomains.emplace(
        std::make_pair(slice_dom.first(), slice_dom.last()), false );

      bool& slice_dom_bounds_assigned = slice_dom_bounds_info.first->second;

      if (!slice_dom_bounds_assigned)
      {
        auto const& slice_dom_bounds = slice_dom_bounds_info.first->first;

        // map the bounds of current sliced_view's subdomain to a subdomain of
        // the underlying container
        auto slice_first_gid = mf(slice_dom_bounds.first);
        auto slice_last_gid = mf(slice_dom_bounds.second);

        // if current sliced_view's subdomain maps to current container's
        // subdomain, we found the correct projected metadata entry
        if (dom.contains(slice_first_gid) and dom.contains(slice_last_gid))
        {
          // pointer to the base container
          component_type c = md.component();

          const auto idx = tuple_ops::discard<slices_type>(md.id());

          res->operator[](idx) = md_entry_type(
            idx, slice_dom, full_slice ? c : nullptr,
            md.location_qualifier(), md.affinity(), md.handle(), md.location()
          );

          // no need to check this subdomain of the sliced_view again
          slice_dom_bounds_assigned = true;
        }
      }
    }

    res->update();

    // Needed to handle the case where a dimension of a slice has lower number
    // of metadata entries than number of locations. part is read by multiple
    // locations and cannot be deleted until those requests have been serviced.
    rmi_fence();

    delete part;

    return std::make_pair(false, res);
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_SLICED_HPP
