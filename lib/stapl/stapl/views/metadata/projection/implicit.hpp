/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

// Coarsen partition based on locality information

#ifndef STAPL_VIEWS_METADATA_PROJECTION_IMPLICIT_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_IMPLICIT_HPP

#include <stapl/runtime.hpp>

#include <stapl/views/operations/make_reference.hpp>

#include <stapl/views/type_traits/has_locality_metadata.hpp>
#include <stapl/views/type_traits/is_segmented_view.hpp>

#include <stapl/views/metadata/container_fwd.hpp>

#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/domains/partitioned_domain.hpp>

#include <boost/type_traits/is_base_of.hpp>
#include <boost/mpl/int.hpp>

#include <stapl/views/metadata/locality_dist_metadata.hpp>

#include <stapl/views/metadata/projection/construct_domain.hpp>

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
/// partitioned view and the container on which it is defined is not a
/// pContainer.  The mapping function of the view can be anything
/// at this point.
///
/// @todo Remove construction of partitioned domain when pView provides
/// a partitioned domain.
/// @todo operator should return a shared_ptr.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
struct implicit_metadata_projection
{
  typedef typename View::domain_type                   domain_type;
  typedef typename View::map_func_type                 map_func_type;
  typedef typename std::remove_pointer<typename P::second_type>::type
    lower_md_cont_type;
  typedef typename metadata_traits<lower_md_cont_type>::value_type
    value_type;
  typedef typename value_type::component_type          component_type;
  typedef typename value_type::cid_type                cid_type;

  typedef metadata_entry<
    domain_type, component_type, cid_type
  >                                                    md_entry_type;

  typedef metadata::growable_container<md_entry_type>  md_cont_type;
  typedef std::pair<bool, md_cont_type*>               return_type;

  return_type operator()(View* vw, lower_md_cont_type* part,
                         bool part_is_fixed = true) const
  {
    typedef typename View::domain_type domain_type;

    domain_type vdom(vw->domain());

    md_cont_type* out_part = new md_cont_type();

    // Local elements loop
    typename View::map_func_type mf(vw->mapfunc());

    // pView should provide a partitioned domain instead of constructing
    // it on the fly when needed.
    typedef partitioned_domain<domain_type> part_dom_type;
    part_dom_type pvdom(vdom);

    typename part_dom_type::subdomains_view_type subdomains =
      pvdom.local_subdomains();

    typename part_dom_type::size_type nsubdomains = subdomains.size();

    for (size_t i = 0; i != nsubdomains; ++i)
    {
      typename part_dom_type::subdomains_view_type::reference subdomain =
        subdomains[i];

      if (!subdomain.empty()) {
        typedef std::pair<typename lower_md_cont_type::iterator,
                          typename domain_type::index_type>        loc_info;

        std::vector<loc_info> locality_info;
        locality_info.reserve(subdomain.size());
        for (typename domain_type::index_type e  = subdomain.first();
                                              e != subdomain.last();
                                              e = subdomain.advance(e, 1))
        {
          typename map_func_type::gid_type gid = mf(e);
          typename lower_md_cont_type::iterator it =
            std::find_if(part->begin(), part->end(), [&](value_type const& x) {
              return x.domain().contains(gid);
          });
          if (it != part->end())
            locality_info.push_back(std::make_pair(it, e));
          else
            abort("Unable to locate GID in non-STAPL container domain");
        }
        // handle last element in subdomain
        typename map_func_type::gid_type gid = mf(subdomain.last());
        typename lower_md_cont_type::iterator it =
          std::find_if(part->begin(), part->end(), [&](value_type const& x) {
              return x.domain().contains(gid);
          });
        if (it != part->end())
          locality_info.push_back(std::make_pair(it, subdomain.last()));
        else
          abort("Unable to locate GID in non-STAPL container domain");

        if (std::distance(part->begin(), part->end()) != 1)
          std::sort(locality_info.begin(), locality_info.end(),
                    [&](loc_info const& x, loc_info const& y) {
                      return x.first < y.first;
          });

        typename std::vector<loc_info>::iterator first = locality_info.begin();
        typename std::vector<loc_info>::iterator curr = locality_info.begin();
        typename std::vector<loc_info>::iterator prev = curr - 1;

        while (first != locality_info.end())
        {
          while ((curr != locality_info.end()) && (curr->first == first->first))
          {
            // prev is invalid if curr == locality_info.begin()
            if (curr != locality_info.begin())
            {
              if (subdomain.advance(prev->second, 1) == curr->second)
              {
                ++curr;
                ++prev;
              } else {
                break;
              }
            } else {
              // The first element is always included in the domain.
              ++curr;
              ++prev;
            }
          }
          if (curr != first)
          {
            --curr;
            --prev;
          }

          out_part->push_back_here(md_entry_type(
            typename lower_md_cont_type::value_type::cid_type(),
            projection_construct_domain<domain_type>()(
              first->second, curr->second, vdom
            ),
            (*curr->first).component(),
            (*curr->first).location_qualifier(),
            (*curr->first).affinity(),
            (*curr->first).handle(),
            (*curr->first).location()
          ));

          first = curr;
          ++first;
          ++curr;
          ++prev;
        }
      }
    }

    out_part->update();

    delete part;

    // Unconditional use of push_back_here above means that this metadata isn't
    // static
    return std::make_pair(false, out_part);
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_IMPLICIT_HPP
