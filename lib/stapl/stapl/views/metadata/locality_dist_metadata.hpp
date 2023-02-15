/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_LOCALITY_DIST_METADATA_HPP
#define STAPL_VIEWS_LOCALITY_DIST_METADATA_HPP

#include <stapl/views/metadata/extraction/generic.hpp>
#include <stapl/views/type_traits/has_locality_metadata.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper to determine the metadata extractor type based on
///        the given container (@c C) type.
///
/// Uses the general container locality/distribution metadata
/// extractor.
/// @todo This catch all method for coarsening ignores the locality of
///   the underlying container and just creates a balanced partition of the
///   elements across the locations of the PARAGRAPH.  Containers should
///   explicitly request this behavior instead of a fallback. Otherwise we
///   silently perform badly.  Force all containers to define
///   loc_dist_metadata.
//////////////////////////////////////////////////////////////////////
template<typename C, bool = detail::has_loc_dist_metadata<C>::value>
struct locality_dist_metadata
{
  typedef generic_metadata_extraction<C> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper to determine the metadata extractor type based on
///        the given container (@c C) type.
///
/// The container/distribution customized its extractor and reflects
/// it via typedef.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct locality_dist_metadata<C, true>
{
  typedef typename C::loc_dist_metadata type;
};

} // namespace stapl

#endif // STAPL_VIEWS_LOCALITY_DIST_METADATA_HPP
