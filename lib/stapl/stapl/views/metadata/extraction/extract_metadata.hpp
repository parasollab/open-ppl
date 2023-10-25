/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

// Coarsen partition based on locality information

#ifndef STAPL_VIEWS_METADATA_EXTRACT_METADATA_HPP
#define STAPL_VIEWS_METADATA_EXTRACT_METADATA_HPP

#include <type_traits>

#include <stapl/views/type_traits/has_locality_metadata.hpp>
#include <stapl/views/metadata/locality_dist_metadata.hpp>
#include <stapl/views/metadata/coarsening_traits.hpp>
#include <stapl/views/metadata/coarsen_view_fwd.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to coarsen the data managed for the given @c
///        Container based on the data locality.
///
/// This functor is used when the given container is not a view.
//////////////////////////////////////////////////////////////////////
template<typename Container,
         bool = is_view<Container>::value,
         bool = stapl::detail::has_loc_dist_metadata<Container>::value>
class extract_metadata
  : public locality_dist_metadata<Container>::type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Functor used to coarsen the data managed for the given @c
///        Container based on the data locality.
///
/// This functor is used when the given container is a view and
/// provides a functor to extract metadata locality.
//////////////////////////////////////////////////////////////////////
template<typename View>
class extract_metadata<View, true, true>
  : public View::loc_dist_metadata
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Functor used to coarsen the data managed for the given @c
///        Container based on the data locality.
///
/// This functor is used when the given container is a view
/// and does not provide a functor to extract metadata locality.
///
/// @todo Function operator should return a shared_ptr.
//////////////////////////////////////////////////////////////////////
template<typename View>
class extract_metadata<View, true, false>
{
private:
  using coarsening_type =
    typename select_extraction_projection_policy<View>::type;

public:
  using return_type = typename coarsening_type::return_type;

  return_type operator()(View* view)
  {
    return coarsening_type::apply(view);
  }
};

} // namespace metadata

} // namespace stapl

#endif
