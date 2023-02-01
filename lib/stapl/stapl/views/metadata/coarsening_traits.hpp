/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

// Coarsen partition based on locality information

#ifndef STAPL_VIEWS_METADATA_COARSENING_TRAITS_HPP
#define STAPL_VIEWS_METADATA_COARSENING_TRAITS_HPP

#include <stapl/views/metadata/projection/segmented.hpp>
#include <stapl/views/metadata/projection/container.hpp>
#include <stapl/views/metadata/projection/implicit.hpp>
#include <stapl/views/metadata/projection/invertible.hpp>

#include <stapl/views/type_traits/is_segmented_view.hpp>
#include <stapl/views/type_traits/is_invertible_view.hpp>
#include <stapl/containers/type_traits/is_container.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the type of the metadata projection
///        function object.
//////////////////////////////////////////////////////////////////////
template<typename View, typename P, bool IsContainer,
         bool IsPartitionedView, bool IsInverse>
struct default_metadata_projection
{
  typedef implicit_metadata_projection<const View, P> type;
};

// a non-segmented view over a container that does not have an inverse defined
template <typename View, typename P>
struct default_metadata_projection<View, P, true, false, false>
{
  typedef container_metadata_projection<const View, P> type;
};

// a non-segmented view over a container that has an inverse defined
template <typename View, typename P>
struct default_metadata_projection<View, P, true, false, true>
{
  typedef invertible_metadata_projection<const View, P> type;
};

// a segmented view over anything
template <typename View, typename P, bool IsContainer, bool IsInverse>
struct default_metadata_projection<View, P, IsContainer, true, IsInverse>
{
  typedef segmented_metadata_projection<const View, P> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Traits class that specializes functionality of the coarsening
///        process.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct coarsening_traits
{
  template<typename Part>
  struct construct_projection
  {
    typedef typename default_metadata_projection<
      View, Part,
      is_container<typename View::view_container_type>::value,
      is_segmented_view<View>::value,
      is_invertible_view<View>::value
    >::type type;
  };
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_COARSENING_TRAITS_HPP
