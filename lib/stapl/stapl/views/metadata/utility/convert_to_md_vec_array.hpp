/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_UTILITY_CONVERT_TO_MD_VEC_ARRAY_HPP
#define STAPL_VIEWS_METADATA_UTILITY_CONVERT_TO_MD_VEC_ARRAY_HPP

#include <stapl/views/metadata/container/growable_fwd.hpp>
#include <stapl/views/metadata/container/metadata_view.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines the coarsen metadata view type for the given
///        container type.
///
/// The metadata container type used is @ref metadata::growable_container.
/// @tparam MDCont Metadata container type.
//////////////////////////////////////////////////////////////////////
template<typename MDCont>
struct convert_to_md_vec_array
{
  typedef typename MDCont::second_type::value_type      value_type;
  typedef metadata::growable_container<value_type>      part_type;
  typedef metadata::view<part_type>                     type;
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_UTILITY_CONVERT_TO_MD_VEC_ARRAY_HPP
