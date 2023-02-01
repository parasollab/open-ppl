/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIARRAY_LOCALIZE_HPP
#define STAPL_CONTAINERS_MULTIARRAY_LOCALIZE_HPP

#include <stapl/utility/tuple.hpp>

namespace stapl {

namespace multiarray_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to return an index tuple if the gid is
///  a tuple or void if the gid is @p size_t.
/// @ingroup pmultiarrayDist
//////////////////////////////////////////////////////////////////////
template<typename GID>
struct indices_type
{
  using type = make_index_sequence<tuple_size<GID>::value>;
};

template<>
struct indices_type<size_t>
{
  typedef void type;
};

} // namespace multiarray_impl


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to convert a gid with global @ref multiarray
///   indexing to that of a base container.
/// @ingroup pmultiarrayDist
/// @tparam GID The GID type.
//////////////////////////////////////////////////////////////////////
template<typename GID,
         typename = typename multiarray_impl::indices_type<GID>::type>
struct nd_localize;


template<typename GID, std::size_t... Indices>
struct nd_localize<GID, index_sequence<Indices...>>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the localization.
  /// @param g The GID that is to be localized.
  /// @param first The starting position of the local portion.
  //////////////////////////////////////////////////////////////////////
  static GID apply(GID const& gid, GID const& first)
  { return GID(get<Indices>(gid) - get<Indices>(first)...); }
};


//////////////////////////////////////////////////////////////////////
/// @copydoc nd_localize
/// Specialization for 1-dimensional sizes
//////////////////////////////////////////////////////////////////////
template<>
struct nd_localize<size_t>
{
  size_t operator()(size_t g, size_t first)
  { return g - first; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to convert a gid with base container
///  indexing into the global domain of the enclosing @ref multiarray
///  container.
/// @tparam GID The GID type.
/// @ingroup pmultiarrayDist
//////////////////////////////////////////////////////////////////////
template<typename GID,
         typename = typename multiarray_impl::indices_type<GID>::type>
struct nd_globalize;


//////////////////////////////////////////////////////////////////////
/// @copydoc nd_globalize
/// Specialization for higher dimensional gid types.
//////////////////////////////////////////////////////////////////////
template<typename GID, std::size_t... Indices>
struct nd_globalize<GID, index_sequence<Indices...>>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the globalization.
  /// @param g The GID that is to be globalize.
  /// @param first The starting position of the local portion.
  //////////////////////////////////////////////////////////////////////
  static GID apply(GID const& gid, GID const& first)
  { return GID(get<Indices>(gid) + get<Indices>(first)...); }
};


//////////////////////////////////////////////////////////////////////
/// @copydoc nd_globalize
/// Specialization for 1-dimensional sizes
//////////////////////////////////////////////////////////////////////
template<>
struct nd_globalize<size_t>
{
  size_t operator()(size_t g, size_t first)
  { return g + first; }
};

} // namespace stapl

#endif // ifndef STAPL_CONTAINERS_MULTIARRAY_LOCALIZE_HPP
