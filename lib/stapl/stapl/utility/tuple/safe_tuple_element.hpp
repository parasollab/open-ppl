/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_SAFE_TUPLE_ELEMENT_HPP
#define STAPL_UTILITY_TUPLE_SAFE_TUPLE_ELEMENT_HPP

#include <stapl/utility/tuple/tuple_element.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that acts like tuple_element except when the
/// index is -1, in which case it gives a default type instead.
///
/// @tparam I          the int index of the element to get
/// @tparam Tuple      the tuple to index into
//////////////////////////////////////////////////////////////////////
template<int I, class Tuple, class Default>
struct safe_tuple_element
  : stapl::tuple_element<(size_t) I, Tuple>
{ };

template<class Tuple, class Default>
struct safe_tuple_element<-1, Tuple, Default>
{
  using type = Default;
};


} // namespace stapl
#endif // STAPL_UTILITY_TUPLE_SAFE_TUPLE_ELEMENT_HPP
