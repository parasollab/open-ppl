/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_EXTRACTION_SEGMENTED_VIEW_HPP
#define STAPL_VIEWS_METADATA_EXTRACTION_SEGMENTED_VIEW_HPP

#include <stapl/views/metadata/extraction/extract_metadata.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to extract the locality metadata for a view whose
///        elements are subviews (@ref segmented_view).
///
/// @tparam Container Container associated with the view that is segmented.
///         Requires that this container reflects a type called container_type,
///         which is the view that is being segmented. This type will most
///         likely be a @ref view_impl::view_container.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct segmented_view_extractor
{
private:
  /// Type of the container whose elements are subviews. Most likely
  /// a @ref view_impl::view_container
  using container_type = Container;

  /// Metadata entries are linearized, so the index is a scalar
  using index_type = std::size_t;

  /// Type of the view that is being segmented
  using underlying_container_type =
    typename container_type::view_container_type;

  /// Type of the extractor for the view that is being segmented
  using extractor_t = extract_metadata<underlying_container_type>;

public:
  /// Metadata container that is generated from metadata extraction of
  /// the underlying view
  using return_type = typename extractor_t::return_type;

  return_type operator()(Container* ct) const
  {
    return extractor_t()(
      const_cast<underlying_container_type*>(ct->get_view()));
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_EXTRACTION_SEGMENTED_VIEW_HPP
