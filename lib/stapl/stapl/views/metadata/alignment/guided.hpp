/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_COARSEN_ALIGN_GUIDED_HPP
#define STAPL_VIEWS_METADATA_COARSEN_ALIGN_GUIDED_HPP

#include <stapl/views/metadata/alignment/guided_offset.hpp>
#include <stapl/views/metadata/alignment/volumetric.hpp>

#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/utility/pack_ops.hpp>

namespace stapl {

namespace metadata {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the alignment policy for a given
///        set of views.
//////////////////////////////////////////////////////////////////////
template<bool Multidimensional, typename ViewSet>
struct select_alignment_policy
{ using type = volumetric_alignment<ViewSet>; };


template<typename ViewSet>
struct select_alignment_policy<false, ViewSet>
{ using type = guided_offset_alignment<ViewSet>; };

} // namespace detail

template<typename ViewSet>
struct guided_alignment;

//////////////////////////////////////////////////////////////////////
/// @brief Generates a metadata alignment for the given view set, using
///        the view at position @p k to guide the alignment.
//////////////////////////////////////////////////////////////////////
template<typename... View>
struct guided_alignment<tuple<View...>>
  : detail::select_alignment_policy<
      pack_ops::and_<(dimension_traits<View>::type::value != 1)...>::value,
      tuple<View...>
    >::type
{ };

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_COARSEN_ALIGN_GUIDED_HPP
