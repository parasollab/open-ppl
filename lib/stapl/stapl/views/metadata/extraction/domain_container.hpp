/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_EXTRACTION_DOMAIN_CONTAINER_HPP
#define STAPL_VIEWS_METADATA_EXTRACTION_DOMAIN_CONTAINER_HPP

#include <stapl/views/metadata/extract.hpp>
#include <stapl/views/metadata/container/growable.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to create metadata for a @see domain_view
///
/// This extractor expects the container to be a
/// @see view_impl::domain_container
///
/// @tparam C container that uses a domain as element storage
//////////////////////////////////////////////////////////////////////
template<typename C>
class domain_container_extractor
{
  using native_part_type =
    typename std::remove_pointer<
      typename metadata::extract_metadata<
        typename C::view_type>::return_type::second_type
    >::type;

  using view_md_type  = typename native_part_type::value_type;
  using domain_type   = typename view_md_type::domain_type;
  using md_entry_type = metadata_entry<domain_type, C*, size_t>;

public:
  using md_cont_type = growable_container<md_entry_type>;
  using return_type  = std::pair<bool, md_cont_type*>;

  return_type operator()(C* cont) const
  {
    // get the metadata for the original view
    auto original_view = cont->view();

    native_part_type* md_cont
      = metadata::extract_metadata<decltype(original_view)>()(&original_view)
          .second;

    md_cont_type* out_part = new md_cont_type();

    // for each local metadata entry of the original view
    for (size_t i = 0; i < md_cont->local_size(); ++i)
    {
      // get the md entry and transform it to have C's container
      auto md = (*md_cont)[md_cont->get_local_vid(i)];

      md_entry_type new_md(
        md.id(), md.domain(), cont,
        md.location_qualifier(), md.affinity(), md.handle(), md.location()
      );

      // add the entry
      out_part->push_back_here(new_md);
    }

    out_part->update();

    delete md_cont;

    return std::make_pair(false, out_part);
  }
};

} // namespace metadata

} // namespace stapl

#endif
