/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

// Coarsen partition based on locality information

#ifndef STAPL_VIEWS_METADATA_TRAITS_HPP
#define STAPL_VIEWS_METADATA_TRAITS_HPP

namespace stapl {

template<typename Container>
struct metadata_traits
{
  typedef std::integral_constant<bool, true> is_isomorphic;
  using value_type = typename Container::value_type;
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_TRAITS_HPP
