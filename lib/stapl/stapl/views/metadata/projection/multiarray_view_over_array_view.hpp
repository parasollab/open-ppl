/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_PROJECTION_MAVW_OVER_AVW_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_MAVW_OVER_AVW_HPP

#include <stapl/utility/tuple/homogeneous_tuple.hpp>
#include <stapl/views/metadata/container/multiarray.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor used to project the domains in the given
///        locality metadata (@c P) to the domain of the given
///        multiarray_view over a 1d array_view. The result is projected
///        metadata locality information.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
class multiarray_view_over_array_view_projection
{
  typedef typename std::remove_pointer<typename P::second_type>::type
    lower_md_cont_type;
  typedef typename View::domain_type                           domain_type;
  typedef typename domain_type::index_type                     index_type;
  typedef typename metadata_traits<lower_md_cont_type>::value_type
    value_type;
  typedef typename value_type::component_type                  component_type;
  typedef typename dimension_traits<View>::type                dimension_type;
  typedef typename View::map_func_type::inverse                inverse_mf_type;

public:
  typedef metadata_entry<
    domain_type, component_type, index_type
  >                                                            md_entry_type;
  typedef metadata::multidimensional_container<md_entry_type>  md_cont_type;
  typedef typename md_cont_type::partition_type       return_type_partition;
  typedef std::pair<bool, md_cont_type*>                        return_type;

  static_assert(
    std::is_same<
      return_type_partition,
      multiarray_impl::block_partition<typename domain_type::traversal_type>
    >::value,
    "Unexpected metadata::multidimensional_container partition "
    "(block_partition required).");

  //////////////////////////////////////////////////////////////////////
  /// @brief Project the metadata entries extracted from the underlying
  ///        array view into the domain of the multiarray_view.
  ///
  /// Since the underlying partition is one-dimensional, we assume that the
  /// multiarray_view built on top of it is partitioned along the first
  /// dimension and all elements in the remaining dimensions are local.
  /// In order to project the 1d block extracted from the array_view,
  /// it is sufficient to de-linearize its first and last indices and
  /// create a 3d indexed domain spanning all indices in between.
  ///
  /// Partitioning along the first dimension is also preserved for the returned
  /// container (@ref metadata::multidimensional_container) by constructing it
  /// with block partition of dimensions (num_locations, 1, 1). This allows to
  /// use @ref linearized_multiarray_view_over_array_view_projection when
  /// coarsening linear view over multidimensional views over the 1d array_view.
  ///
  /// @param view The target multiarray_view over a 1d array view
  /// @param part Container of metadata entries
  //////////////////////////////////////////////////////////////////////
  return_type operator()(View* view, lower_md_cont_type* part) const
  {
    const auto part_size = part->size();

    int constexpr dim = dimension_type::value;

    auto ret_dims = homogeneous_tuple<dim>(1ul);
    std::get<0>(ret_dims) = part_size;

    auto const ret_part = return_type_partition(domain_type(ret_dims),
      get_num_locations() > 1 ? ret_dims : homogeneous_tuple<dim>(1ul));

    md_cont_type* res = new md_cont_type(ret_part);
    assert(res->local_size() != 0);

    auto res_local_it = res->begin();

    inverse_mf_type inv(view->mapfunc());

    for (size_t i = 0; i < part->local_size(); ++i)
    {
      value_type const& md = (*part)[part->get_local_vid(i)];

      // one-dimensional domain of the metadata entry
      auto const& dom_1d = md.domain();

      // multi-dimensional projected domain
      auto const dom_nd = domain_type(
        inv(dom_1d.first()), inv(dom_1d.last()), part_size == 1);

      // index of this entry in the metadata container
      auto idx = homogeneous_tuple<dimension_type::value>(0ul);
      std::get<0>(idx) = md.id();

      *res_local_it = md_entry_type(
        idx, dom_nd, md.component(),
        md.location_qualifier(), md.affinity(), md.handle(), md.location()
      );

      ++res_local_it;
    }

    res->update();

    delete part;

    return std::make_pair(false, res);
  }
};

//////////////////////////////////////////////////////////////////////
/// @copybrief multiarray_view_over_array_view_projection
///
/// This helper functor is invoked when the given @c View is a
/// linearized view over (views over a) multiarray view over a 1d
/// array view.
///
/// @todo operator should return a shared_ptr.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
class linearized_multiarray_view_over_array_view_projection
{
  typedef typename std::remove_pointer<typename P::second_type>::type
    lower_md_cont_type;
  typedef typename View::domain_type                           domain_type;
  typedef typename domain_type::index_type                     index_type;
  typedef typename metadata_traits<lower_md_cont_type>::value_type
    value_type;
  typedef typename value_type::component_type                  component_type;
  typedef typename dimension_traits<View>::type                dimension_type;
  typedef typename View::map_func_type::inverse                inverse_mf_type;

public:
  typedef metadata_entry<
    domain_type, component_type, index_type
  >                                                            md_entry_type;
  typedef metadata::flat_container<md_entry_type>              md_cont_type;
  typedef std::pair<bool, md_cont_type*>                       return_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Project the metadata entries extracted from the underlying
  ///        multidimensional view over a 1d array view into the domain of
  ///        the linearized view.
  ///
  /// Essentially the same algorithm as @ref invertible_metadata_projection
  /// with bijective mapping function, but utilizing the fact that the
  /// container of metadata entries is partitioned along the first
  /// dimension only (allowing us to dimension the returned container
  /// appropriately instead of using @ref growable_container).
  ///
  /// @param view The target linearized view
  /// @param part Container of metadata entries
  //////////////////////////////////////////////////////////////////////
  return_type operator()(View* view, lower_md_cont_type* part,
                         bool part_is_fixed = true) const
  {
    const auto part_size = part->size();

    stapl_assert(std::get<0>(part->dimensions()) == part_size,
      "linearized_multiarray_view_over_array_view_projection can be used "
      "only in cases where the multidimensional metadata container is "
      "partitioned along the first dimension only");

    md_cont_type* res = new md_cont_type(part_size);
    assert(res->local_size() != 0);

    auto res_local_it = res->begin();

    // inverse mapping function of the linearized view (i.e., nd_linearize)
    inverse_mf_type inv(view->mapfunc());

    using domain_t = typename lower_md_cont_type::domain_type;
    domain_t local_domain(part->local_dimensions());

    domain_map(local_domain, [&](typename domain_t::index_type const& i) {
      value_type const& md = (*part)[part->get_local_vid(i)];

      // domain for the metadata entry
      auto const& dom_nd = md.domain();

      // projected domain
      auto const dom_1d = domain_type(
        inv(dom_nd.first()), inv(dom_nd.last()), part_size == 1);

      // don't store the base container pointer if it doesn't contain the
      // whole slice (prevents conversion to fast view)
      *res_local_it = md_entry_type(
        std::get<0>(md.id()), dom_1d, md.component(),
        md.location_qualifier(), md.affinity(), md.handle(), md.location()
      );

      ++res_local_it;
    });

    res->update();

    delete part;

    return std::make_pair(false, res);
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_MAVW_OVER_AVW_HPP
