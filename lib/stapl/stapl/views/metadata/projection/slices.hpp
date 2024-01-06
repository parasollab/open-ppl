/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_VIEWS_METADATA_PROJECTION_SLICES_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_SLICES_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>

#include "slice_common.hpp"

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor used to project the domains in the given
///        locality metadata (@c P) to the domain of the given @c
///        View. The result is projected metadata locality
///        information.
///
/// This helper functor is invoked when the given @c View is a
/// @ref slices_segmented_view.
///
/// @todo operator should return a shared_ptr.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
class slices_projection
{
  typedef typename std::remove_pointer<typename P::second_type>::type
    lower_md_cont_type;
  typedef typename View::slices_type                           slices_type;
  typedef typename View::domain_type                           domain_type;
  typedef typename domain_type::index_type                     index_type;
  typedef typename metadata_traits<lower_md_cont_type>::value_type
    value_type;
  typedef typename value_type::component_type                  component_type;
  typedef typename dimension_traits<View>::type                dimension_type;

  // indicates whether the target slices_segmented_view
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
    using md_cont_type_partition = typename md_cont_type::partition_type;
    return new md_cont_type(md_cont_type_partition(domain_type(dims),
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
  ///        multidimensional container into the domain of the
  ///        @ref slices_segmented_view.
  ///
  /// @param view The target view
  /// @param part Container of extracted metadata entries
  /// @param part_is_fixed Indicates if part container is fixed-size
  //////////////////////////////////////////////////////////////////////
  return_type operator()(View* view, lower_md_cont_type* part,
                         bool part_is_fixed = true) const
  {
    // partitions layout for the original container
    const auto part_dims_orig = part->dimensions();

    // determine if the whole slice is contained in one partition
    const bool full_slice = part_has_full_slice<slices_type>(part_dims_orig);

    // partitions layout for the container with projected metadata
    const auto part_dims_proj = tuple_ops::filter<slices_type>(part_dims_orig);

    // construct the container for the projected metadata, using the partition
    // obtained by slicing the partition of the source container of extracted
    // metadata entries
    md_cont_type* res = construct_return_type(part_dims_proj, is_nd());

    using domain_t = typename lower_md_cont_type::domain_type;
    domain_t local_domain(part->local_dimensions());

    // iterate over the local part of the metadata container
    domain_map(local_domain, [&](typename domain_t::index_type const& i) {
      value_type const& md = (*part)[part->get_local_vid(i)];

      // pointer to the base container
      component_type c = md.component();

      // domain for the metadata entry
      auto const& dom = md.domain();

      // reduced domain for the slices view
      const domain_type new_dom(
        tuple_ops::filter<slices_type>(dom.first()),
        tuple_ops::filter<slices_type>(dom.last())
      );

      // index of this entry in the metadata container
      const auto idx = tuple_ops::filter<slices_type>(md.id());

      // don't store the base container pointer if it doesn't contain the
      // whole slice (prevents conversion to fast view)
      res->operator[](idx) = md_entry_type(
        idx, new_dom, full_slice ? c : nullptr,
        md.location_qualifier(), md.affinity(), md.handle(), md.location()
      );
    });

    res->update();

    // Needed to handle the case where a sliced dimension has lower number of
    // metadata entries than number of locations. part is read by multiple
    // locations and cannot be deleted until those requests have been serviced.
    rmi_fence();

    delete part;

    return std::make_pair(false, res);
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_SLICES_HPP
