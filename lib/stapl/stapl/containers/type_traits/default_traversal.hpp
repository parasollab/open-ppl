/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DEFAULT_TRAVERSAL_HPP
#define STAPL_CONTAINERS_DEFAULT_TRAVERSAL_HPP

#include <stapl/utility/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute a the default traversal for an
/// n-dimensional space.
///
/// Returns a tuple type that defines the importance of each element
/// of the coordinate. The dimension holding 0 will be traversed first,
/// followed by the dimension holding 1, and so on.
///
/// For example, the default traversal for a 3-dimensional space is <2, 1, 0>,
/// which means that for a 3D coordinate (x, y, z), the z dimension will
/// be traversed first, followed by y and ending with x.
///
/// @tparam N The number of dimensions in the space.
///
/// @note icc-16 fails to expand an expression if the @c Indices shows up
/// twice in the expansion.
//////////////////////////////////////////////////////////////////////
template<int N, typename = make_index_sequence<N>>
struct default_traversal;


template<int N, std::size_t... Indices>
struct default_traversal<N, index_sequence<Indices...>>
{
  using size = std::integral_constant<std::size_t, sizeof...(Indices)>;
  using type = tuple<boost::mpl::int_<(size::value - Indices - 1)>...>;
};

} // namespace stapl

#endif
