/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_VIEWS_METADATA_PROJECTION_GENERIC_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_GENERIC_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/container_fwd.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/domains/partitioned_domain.hpp>
#include <stapl/views/metadata/locality_dist_metadata.hpp>
#include <stapl/utility/cross_map.hpp>

#include <stapl/containers/type_traits/dimension_traits.hpp>

#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/repetition/repeat.hpp>
#include <boost/preprocessor/iteration/local.hpp>
#include <boost/preprocessor/iteration/iterate.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor used to project the domains in the given
///        locality metadata (@c P) to the domain of the given @c
///        View. The result is projected metadata locality
///        information.
///
/// This helper functor is invoked when there is no information about
/// the given view, and is thus inefficient for most cases. It attempts
/// to create a single metadata entry for each element of the view.
///
/// @todo operator should return a shared_ptr.
///
/// @fixme In its current form, this algorithm is incorrect, as it does
///        not take into account that the input view could be a segmented
///        view and its elements could be a collection of elements. In such
///        a case, it is necessary to check each element in the subview.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
class generic_metadata_projection
{
  typedef typename View::domain_type                         domain_type;
  typedef typename P::value_type::component_type             component_type;
  typedef typename P::value_type::cid_type                   cid_type;
  typedef typename domain_type::index_type                   gid_type;

public:
  typedef metadata_entry<
    domain_type, component_type, cid_type
  >                                                          md_entry_type;
  typedef metadata::growable_container<md_entry_type>        md_cont_type;
  typedef std::pair<bool, md_cont_type*>                     return_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Project the metadata entries extracted from the underlying view
  ///        into the domain of the view.
  ///
  /// @param view The original view
  /// @param part Container of metadata entries
  //////////////////////////////////////////////////////////////////////
  return_type operator()(View* view, P* part,
                         bool part_is_fixed = true) const
  {
    md_cont_type* res = new md_cont_type();

    auto mf = view->mapfunc();
    auto domain = view->domain();

    component_type last_component = nullptr;

    for (auto&& md : *part)
    {
      // pointer to the base container
      component_type c = md.component();

      // domain for the metadata entry
      const domain_type dom = md.domain();

      // go through each element of the input view
      domain_map(domain, [&](gid_type const& e) {
        auto gid = mf(e);

        // if this base container contains the true element
        if (dom.contains(gid))
          res->push_back_here(md_entry_type(
            cid_type(),
            domain_type(e, e),
            c,
            md.location_qualifier(), md.affinity(), md.handle(), md.location()
          ));

      });
    }

    res->update();

    delete part;

    return std::make_pair(false, res);
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_GENERIC_HPP
