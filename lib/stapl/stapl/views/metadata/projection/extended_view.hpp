/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_PROJECTION_EXTENDED_VIEW_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_EXTENDED_VIEW_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/container_fwd.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/domains/partitioned_domain.hpp>

#include <stapl/views/metadata/locality_dist_metadata.hpp>
#include <stapl/views/metadata/projection/construct_domain.hpp>

#include <stapl/views/metadata/projection/multidimensional.hpp>

namespace stapl {

namespace detail {

template<size_t FixedIndex, typename T, typename Sequence>
struct rewire_dimension;

template<typename Index, size_t FixedIndex, typename T, typename Sequence>
void fix_last(Index& last, Index const& full_last,
              rewire_dimension<FixedIndex, T, Sequence> const&)
{
  std::get<FixedIndex>(last) = std::get<FixedIndex>(full_last);
}

} // namespace detail

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor used to project the domains in the given
///        locality metadata (@c P) to the domain of the given @c
///        View. The result is projected metadata locality
///        information, where the metadata is expressed using the view's
///        domain type.
///
/// This helper functor is invoked when the given @c View is an
/// @ref extended_view and the container on which it is defined is a
/// @ref multiarray.  The mapping function of the view is an instance of
/// rewire_dimension.
///
/// @todo operator should return a shared_ptr.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
struct extended_view_metadata_projection
{
  using lower_md_cont_type =
    typename std::remove_pointer<typename P::second_type>::type;
  using domain_type     = typename View::domain_type;
  using map_func_type   = typename View::map_func_type;
  using component_type  =
    typename lower_md_cont_type::value_type::component_type;

  using dom_info_type   = metadata_entry<
    domain_type, component_type,
    typename lower_md_cont_type::value_type::cid_type>;

  using md_cont_type     = metadata::multidimensional_container<dom_info_type>;

  using return_type      = std::pair<bool, md_cont_type*>;

  return_type operator()(View const* vw, lower_md_cont_type* part,
                         bool part_is_fixed = true) const
  {
    const typename domain_type::index_type full_last = vw->domain().last();
    map_func_type const& mf = vw->mapfunc();

    md_cont_type *extended_metadata = new md_cont_type(part->dimensions());

    if (part->local_size() != 0)
    {
      // Iterate over metadata_entry elements and extend the domain of each.
      for (auto&& md : *part)
      {
        domain_type dom = md.domain();
        typename domain_type::index_type last = dom.last();
        fix_last(last, full_last, mf);
        domain_type extended_dom(dom.first(), last, true);

        extended_metadata->operator[](md.id()) =
          dom_info_type(md.id(), extended_dom, md.component(),
                        md.location_qualifier(), md.affinity(),
                        md.handle(), md.location());
      }
    }

    // Needed to handle the case where there is a single metadata
    // entry. part is read by multiple locations and cannot be deleted until
    // those requests have been serviced.
    rmi_fence();

    delete part;

    return std::make_pair(false, extended_metadata);
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_EXTENDED_VIEW_HPP
