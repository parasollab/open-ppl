/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_PROJECTION_CONTAINER_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_CONTAINER_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/container_fwd.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/domains/partitioned_domain.hpp>

#include <stapl/views/metadata/locality_dist_metadata.hpp>
#include <stapl/views/metadata/projection/construct_domain.hpp>

#include <stapl/views/metadata/projection/multidimensional.hpp>

#include <stapl/views/type_traits/underlying_container.hpp>
#include <stapl/views/type_traits/upcast.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor used to project the domains in the given
///        locality metadata (@c P) to the domain of the given @c
///        View. The result is projected metadata locality
///        information, where the metadata is expressed using the view's
///        domain type.
///
/// This helper functor is invoked when the given @c View is not a
/// partitioned view and the container on which it is defined is a
/// pContainer.  The mapping function of the view can be anything
/// at this point.
///
/// @todo Remove construction of partitioned_domain when pView provides
/// a partitioned domain.
/// @todo operator should return a shared_ptr.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
struct container_metadata_projection
{
  typedef typename std::remove_pointer<typename P::second_type>::type
    lower_md_cont_t;
  typedef typename View::domain_type                   domain_type;
  typedef compose_map_func<
            typename View::map_func_type,
            typename View::view_container_type
          >                                            mapfunc_composer;
  typedef typename mapfunc_composer::type              map_func_type;
  typedef typename lower_md_cont_t::value_type::component_type
    component_type;

  typedef metadata_entry<domain_type, component_type,
    typename lower_md_cont_t::value_type::cid_type>          dom_info_type;

  typedef metadata::growable_container<dom_info_type>  md_cont_type;
  typedef std::pair<bool, md_cont_type*>               return_type;

  return_type operator()(View* vw, lower_md_cont_t* part,
                         bool part_is_fixed = true) const
  {
    return project_based_on_dimensionality(vw, part,
      std::integral_constant<bool, dimension_traits<View>::type::value == 1>()
    );
  }

  return_type project_based_on_dimensionality(View* vw, lower_md_cont_t* part,
    std::integral_constant<bool, false>) const
  {
    return multidimensional_metadata_projection<View, P>()(vw, part);
  }

  return_type project_based_on_dimensionality(View* vw, lower_md_cont_t* part,
    std::integral_constant<bool, true>) const
  {
    typedef typename View::domain_type                 domain_type;
    typedef typename underlying_container<View>::type  container_type;
    typedef typename container_type::distribution_type distribution_type;

    domain_type vdom(vw->domain());

    auto& distribution = const_cast<distribution_type&>(
      underlying_container<View>::get(*vw).distribution()
    );

    md_cont_type* out_part = new md_cont_type();
    // Local elements loop

    map_func_type mf = mapfunc_composer::apply(vw->mapfunc(), vw->container());

    // pView should provide a partitioned domain instead of constructing
    // it on the fly when needed.
    typedef partitioned_domain<domain_type> part_dom_type;

    part_dom_type pvdom(vdom);

    auto const& subdomains = pvdom.local_subdomains();

    auto nsubdomains = subdomains.size();

    for (size_t i = 0; i != nsubdomains; ++i)
    {
      auto const& subdomain = subdomains[i];

      if (!subdomain.empty()) {
        typedef typename distribution_type::dom_info_type loc_info;
        std::vector<future<loc_info> > locality_futures;
        locality_futures.reserve(subdomain.size());
        for (typename domain_type::index_type e  = subdomain.first();
                                              e != subdomain.last();
                                              e = subdomain.advance(e, 1))
        {
          auto gid = mf(e);
          locality_futures.push_back(distribution.metadata_at(gid));
        }

        // handle the last element of the subdomain.
        auto gid = mf(subdomain.last());
        locality_futures.push_back(distribution.metadata_at(gid));

        // construct the domains of elements with the same locality info.
        typename std::vector<future<loc_info> >::iterator li =
          locality_futures.begin();

        auto first_e = subdomain.first();
        auto prev_e  = subdomain.first();
        auto curr_e  = subdomain.first();

        loc_info e_info    = (*li).get();
        loc_info curr_info = e_info;

        if (subdomain.size() != 1)
        {
          curr_e = subdomain.advance(curr_e, 1);
          ++li;
          for (; curr_e != subdomain.last();
               curr_e = subdomain.advance(curr_e, 1),
               prev_e = subdomain.advance(prev_e, 1), ++li)
          {
            e_info = (*li).get();

            if ((e_info.location() != curr_info.location()) ||
                (e_info.affinity() != curr_info.affinity()) ||
                (e_info.id() != curr_info.id()))
            {
              out_part->push_back_here(dom_info_type(
                typename dom_info_type::cid_type(),
                construct_domain<domain_type>()(
                  first_e, prev_e, distribution, subdomain
                ),
                static_cast<component_type>(curr_info.component()),
                curr_info.location_qualifier(), curr_info.affinity(),
                curr_info.handle(), curr_info.location()
              ));
              curr_info = e_info;
              first_e = curr_e;
            }
          }

          stapl_assert(e_info.handle() == curr_info.handle(),
            "handle mismatch, expected same container");

           // processing the last element of the subdomain.
          e_info = (*li).get();

          if ((e_info.location() != curr_info.location()) ||
              (e_info.affinity() != curr_info.affinity()) ||
              (e_info.id() != curr_info.id()))
          {
            out_part->push_back_here(dom_info_type(
              typename dom_info_type::cid_type(),
              construct_domain<domain_type>()(
                first_e, prev_e, distribution, subdomain
              ),
              static_cast<component_type>(curr_info.component()),
              curr_info.location_qualifier(), curr_info.affinity(),
              curr_info.handle(), curr_info.location()
            ));
            curr_info = e_info;
            first_e = curr_e;
          }
        }

        out_part->push_back_here(dom_info_type(
          typename dom_info_type::cid_type(),
          construct_domain<domain_type>()(
            first_e, curr_e, distribution, subdomain
          ),
          static_cast<component_type>(e_info.component()),
          e_info.location_qualifier(), curr_info.affinity(),
          e_info.handle(), e_info.location()
        ));
      }
    }
    out_part->update();

    delete part;

    return std::make_pair(false, out_part);
  }

};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_CONTAINER_HPP
