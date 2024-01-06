/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_TYPE_TRAITS_IS_INVERTIBLE_PARTITION_HPP
#define STAPL_CONTAINERS_TYPE_TRAITS_IS_INVERTIBLE_PARTITION_HPP

namespace stapl {

template<typename Part>
struct is_invertible_partition
 : public std::integral_constant<bool, false>
{ };

} // namespace stapl

#endif // STAPL_CONTAINERS_TYPE_TRAITS_IS_INVERTIBLE_PARTITION_HPP
