/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_PROJECTION_SEGMENTED_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_SEGMENTED_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/domains/partitioned_domain.hpp>
#include <stapl/domains/intersect.hpp>
#include <stapl/views/metadata/locality_dist_metadata.hpp>

#include <stapl/views/metadata/container_fwd.hpp>

#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/views/type_traits/is_native_partition.hpp>


namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor used to project the domains in the given
///        locality metadata (@c P) to the domain of the given @c
///        View. The result is projected metadata locality
///        information.
///
/// This helper functor is invoked when the given @c View is a
/// partitioned view.
///
/// @todo operator should return a shared_ptr.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
struct segmented_metadata_projection
{
  typedef typename std::remove_pointer<typename P::second_type>::type
    lower_md_cont_t;

  // Adjust 1D domains
  template <typename V,typename MD,typename Dim>
  struct adjust_metadata_domains_helper
  {
    typedef typename V::domain_type                      domain_type;
    typedef typename V::map_func_type                    map_func_type;

    typedef typename MD::value_type                      value_type;
    typedef typename value_type::component_type          component_type;
    typedef metadata_entry<domain_type,component_type,
              typename MD::value_type::cid_type >        dom_info_type;
    typedef metadata::growable_container<dom_info_type>  return_type;

    return_type*
    operator()(View* view, lower_md_cont_t* cpart) const
    {
      typedef typename View::view_container_type         container_type;

      typedef std::vector<std::pair<domain_type,bool> >  vec_doms_t;

      // check if the projected metadata is known to have the same number of
      // elements as the underlying container metadata
      constexpr bool static_metadata =
        is_native_partition<typename V::partition_type>::value;

      // get the partitioner from the partitioner_container
      auto par = view->container().partition();
      auto mfg = view->container().mapfunc_generator();

      return_type* res =
        static_metadata ? new return_type(cpart->size()) : new return_type();

      using domain_t = typename lower_md_cont_t::domain_type;
      domain_t local_domain(cpart->local_dimensions());

      const bool has_skipped_seg = !view->domain().is_same_container_domain();

      // iterate over the local part of the metadata container
      domain_map(local_domain, [&](typename domain_t::index_type const& i) {
        value_type const& entry = (*cpart)[cpart->get_local_vid(i)];

        component_type sc = entry.component();

        if (!static_metadata) {
          vec_doms_t doms = par.contained_in(entry.domain(), mfg);
          for (auto& dj : doms)
          {
            if (has_skipped_seg)
              dj.first = intersect(dj.first, view->domain());

            if (dj.second) {
              res->push_back_here(dom_info_type(
                typename dom_info_type::cid_type(), dj.first, sc,
                entry.location_qualifier(), entry.affinity(),
                entry.handle(), entry.location()
              ));
            }
            else {
              res->push_back_here(dom_info_type(
                typename dom_info_type::cid_type(), dj.first, NULL,
                entry.location_qualifier(), entry.affinity(),
                entry.handle(), entry.location()
              ));
            }
          }
        } else {
          vec_doms_t doms = par.contained_in(entry.domain(), mfg);
          stapl_assert(doms.size() == 1, "static metadata not balanced");
          (*res)[view->get_location_id()] =
              dom_info_type(
                typename dom_info_type::cid_type(),
                doms[0].first, sc,
                entry.location_qualifier(), entry.affinity(),
                entry.handle(), entry.location()
              );
        }
      });

      res->update();

      delete cpart;

      return res;
    }

  };

  typedef typename View::domain_type                           domain_type;

  typedef typename dimension_traits<
    typename lower_md_cont_t::value_type::domain_type::gid_type
  >::type                                                      dimension_t;

  typedef typename adjust_metadata_domains_helper<
    View, lower_md_cont_t, dimension_t
  >::return_type                                               md_cont_type;

  typedef std::pair<bool, md_cont_type*>                       return_type;

  return_type operator()(View* view, lower_md_cont_t* part,
                         bool part_is_fixed = true) const
  {
    constexpr bool static_metadata =
      is_native_partition<typename View::partition_type>::value;
    return std::make_pair(part_is_fixed && static_metadata,
      adjust_metadata_domains_helper<View,lower_md_cont_t,dimension_t>()(
        view, part));
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_SEGMENTED_HPP
