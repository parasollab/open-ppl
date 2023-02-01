/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_VIEWS_METADATA_PROJECTION_INVERTIBLE_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_INVERTIBLE_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/views/metadata/metadata_traits.hpp>
#include <stapl/views/metadata/container_fwd.hpp>

#include <stapl/domains/invert_domain.hpp>
#include <stapl/views/metadata/utility/fixed_size_metadata.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor used to project the domains in the given
///        locality metadata (@c P) to the domain of the given @c
///        View. The result is projected metadata locality
///        information.
///
/// This helper functor is invoked when the given @c View is an
/// invertible view.
///
/// @todo operator should return a shared_ptr.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
class invertible_metadata_projection
{
  typedef typename std::remove_pointer<typename P::second_type>::type
    lower_md_cont_type;
  typedef typename View::domain_type                         domain_type;
  typedef typename metadata_traits<lower_md_cont_type>::value_type
    value_type;
  typedef typename value_type::component_type                component_type;
  typedef typename value_type::cid_type                      cid_type;
  typedef typename View::map_func_type::inverse              inverse_mf_type;

public:
  typedef metadata_entry<
    domain_type, component_type, cid_type
  >                                                          md_entry_type;
  typedef metadata::growable_container<md_entry_type>        md_cont_type;
  typedef std::pair<bool, md_cont_type*>                      return_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Project the metadata entries extracted from the underlying view
  ///        into the domain of the view.
  ///
  /// @param view The original view
  /// @param part Container of metadata entries
  //////////////////////////////////////////////////////////////////////
  return_type operator()(View* view, lower_md_cont_type* part,
                         bool part_is_fixed = true) const
  {
    bool use_fixed_size = part_is_fixed && is_fixed_size_md(view, part);

    md_cont_type* res;

    if (use_fixed_size)
      res = new md_cont_type(view->get_num_locations());
    else
      res = new md_cont_type;

    inverse_mf_type inv(view->mapfunc());

    using domain_t = typename lower_md_cont_type::domain_type;
    domain_t local_domain(part->local_dimensions());

    // iterate over the local part of the metadata container
    domain_map(local_domain, [&](typename domain_t::index_type const& i) {
      value_type const& md = (*part)[part->get_local_vid(i)];

      // get the base container's domain
      auto const& dom = md.domain();
      component_type c = md.component();

      // project the base container's domain into the view's domain
      // using the provided inverse function
      domain_type projected_dom =
        invert_domain<domain_type>(dom, inv) & view->domain();

      // avoid creating empty md entries
      if (!projected_dom.empty()) {
        // if the projected domain is somehow bigger than the original domain
        // of the base container, we will disable fast view optimization
        component_type comp = projected_dom.size() <= dom.size() ? c : nullptr;

        if (!use_fixed_size) {
          res->push_back_here(md_entry_type(
            cid_type(),
            projected_dom, comp,
            md.location_qualifier(), md.affinity(), md.handle(), md.location()
          ));
        } else {
          (*res)[view->get_location_id()] = md_entry_type(
            cid_type(),
            projected_dom, comp,
            md.location_qualifier(), md.affinity(), md.handle(), md.location()
          );
        }
      }
    });

    res->update();

    delete part;

    return std::make_pair(use_fixed_size, res);
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_INVERTIBLE_HPP
