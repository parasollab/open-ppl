/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MAKE_TRAVERSAL_HPP
#define STAPL_CONTAINERS_MAKE_TRAVERSAL_HPP

#include <stapl/utility/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute a traveral type based on an integral
///   variadic pack of indices.
//////////////////////////////////////////////////////////////////////
template<std::size_t... Indices>
struct make_traversal
{
  using size = std::integral_constant<std::size_t, sizeof...(Indices)>;
  using type = tuple<boost::mpl::int_<Indices>...>;
};

} // namespace stapl

#endif
