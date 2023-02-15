/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_EXTRACTION_REVERSE_HPP
#define STAPL_VIEWS_METADATA_EXTRACTION_REVERSE_HPP

#include <stapl/views/metadata/extract.hpp>
#include <stapl/views/metadata/container/reverse.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/views/metadata/utility/convert_to_md_vec_array.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to extract the locality metadata associated
///        with a @ref reverse_view.
///
/// @tparam View A reverse_view
//////////////////////////////////////////////////////////////////////
template<typename View>
class reverse_extractor
{
  using view_part_type =
    typename metadata::extract_metadata<
      typename View::target_view_type
    >::return_type;

  using md_container_type = metadata::reverse_container<view_part_type>;

  using part_cont_type =
    typename std::remove_pointer<typename view_part_type::second_type>::type;

  using ret_cont_type =
    view_wrapper<typename md_container_type::value_type,
                 typename part_cont_type::domain_type,
                 metadata_traits<part_cont_type>::is_isomorphic::value>;

public:
  using return_type = std::pair<bool, ret_cont_type*>;

  template<typename V>
  return_type operator()(V* vw)
  {
    using original_view_type = typename V::target_view_type;

    // construct the original (non-reversed) view
    original_view_type original_view(
      vw->container(), vw->domain(), vw->mapfunc()
    );

    // get the metadata for the original view
    auto md_cont
      = metadata::extract_metadata<original_view_type>()(&original_view);

    // adapt the metadata to be in reverse order
    md_container_type* reverse_md =
      new md_container_type(md_cont.second, vw->m_total_size);

    // wrap the reversed metadata container in a view
    // First element of pair could be md_cont.first.  Explicitly set to false
    // to limit the use of the static metadata optimization in alignment.
    return std::make_pair(false,
      new ret_cont_type(new view<md_container_type>(reverse_md))
    );
  }
};

} // namespace metadata

} // namespace stapl

#endif
