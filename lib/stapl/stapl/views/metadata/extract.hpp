/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_EXTRACT_HPP
#define STAPL_VIEWS_METADATA_EXTRACT_HPP

#include <stapl/views/metadata/extraction/extract_metadata.hpp>
#include <stapl/views/type_traits/is_view.hpp>

namespace stapl {

namespace metadata {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper method to create a unique_ptr to a view's
///        container's metadata
//////////////////////////////////////////////////////////////////////
template <typename View>
struct extract_impl
{
  using metadata_container_type =
    typename std::remove_pointer<typename metadata::extract_metadata<
      typename View::view_container_type>::return_type::second_type>::type;

  using type = std::unique_ptr<metadata_container_type>;

  static type apply(View const& v)
  {
    return type{
      metadata::extract_metadata<typename View::view_container_type>()(
        v.get_container())
        .second
    };
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Extract the metadata from a view's container.
///
///        This function is designed to be invoked on a view that is
///        directly over a container. To recursively distill metadata
///        from composed views, see @c metadata::extract_metadata.
///
/// @return A unique_ptr to the metadata container generated from the
///         extraction process.
//////////////////////////////////////////////////////////////////////
template <typename View>
typename detail::extract_impl<View>::type
extract(View const& v)
{
  static_assert(!is_view<typename View::view_container_type>::value,
                "This function should only be used for non-composed views.");

  return detail::extract_impl<View>::apply(v);
}

} // namespace metadata

} // namespace stapl

#endif
