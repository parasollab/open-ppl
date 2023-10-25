/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in md_cont without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_TRANSFORM_HPP
#define STAPL_VIEWS_METADATA_TRANSFORM_HPP

#include <stapl/views/metadata/coarsen_utility.hpp>

namespace stapl {

namespace metadata {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to construct a partitioned_mix_view from the
///        given view and metadata container.
//////////////////////////////////////////////////////////////////////
struct transform_view_from_metadata
{
  template <typename Signature>
  struct result;

  template <typename View, typename MDCont>
  struct result<transform_view_from_metadata(View, MDCont)>
  {
    using wrapper_t =
      metadata::view_wrapper<
        typename MDCont::value_type,
        typename MDCont::domain_type,
        metadata_traits<MDCont>::is_isomorphic::value
      >;

    using type = partitioned_mix_view<View, wrapper_t>;
  };

  template<typename View, typename MDCont>
  typename result<transform_view_from_metadata(View, MDCont)>::type
  operator()(View& view, MDCont& md_cont) const
  {
    typedef typename result<
      transform_view_from_metadata(View, MDCont)
    >::type                                                    return_t;
    typedef typename return_t::view_container_type             container_t;
    typedef metadata::view_wrapper<
      typename MDCont::value_type,
      typename MDCont::domain_type,
      metadata_traits<MDCont>::is_isomorphic::value
    > wrapper_t;


    md_cont.update();

    return return_t(new container_t(view, wrapper_t(new MDCont(md_cont))));
  }
}; // struct transform_view_from_metadata

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Transform fine-grained views into coarse-grained views based
///        on a set of metadata containers.
///
/// @param views A tuple of fine-grained views
/// @param md_conts A tuple of metadata corresponding metadata containers
/// @return A tuple of coarse-grained views.
//////////////////////////////////////////////////////////////////////
template<typename Views, typename MDConts>
auto transform(Views&& views, MDConts&& md_conts)
 -> decltype(
     vs_map(detail::transform_view_from_metadata(), views, md_conts)
   )
{
  return vs_map(detail::transform_view_from_metadata(),
    //std::forward<Views>(views), std::forward<MDConts>(md_conts)
    views, md_conts
  );
}

} // namespace metadata

} // namespace stapl

#endif
