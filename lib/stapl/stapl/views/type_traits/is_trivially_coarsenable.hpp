/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_TRIVIALLY_COARSENED_HPP
#define STAPL_VIEWS_TYPE_TRAITS_TRIVIALLY_COARSENED_HPP

#include <stapl/containers/generators/generator_container_base.hpp>
#include <stapl/views/view_traits.hpp>

#include <type_traits>

namespace stapl {

template<typename View>
struct is_trivially_coarsenable
 : std::is_base_of<
     generator_container_base,
     typename view_traits<View>::container
   >
{ };

} //namespace stapl

#endif
