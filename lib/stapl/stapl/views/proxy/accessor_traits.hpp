/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PROXY_ACCESSOR_TRAITS_HPP
#define STAPL_VIEWS_PROXY_ACCESSOR_TRAITS_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that defines traits for accessors.
///
/// @tparam Accessor Accessor in question
//////////////////////////////////////////////////////////////////////
template<typename Accessor>
struct accessor_traits
{
  typedef std::false_type is_localized;
};

} // namespace stapl

#endif // STAPL_VIEWS_PROXY_ACCESSOR_TRAITS_HPP
