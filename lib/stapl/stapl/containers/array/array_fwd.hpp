/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_ARRAY_FWD_HPP
#define STAPL_CONTAINERS_ARRAY_FWD_HPP

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief Parallel array container.
/// @ingroup parray
///
/// Parallel sequence container with fixed size. By default, its
/// GID type is a std::size_t, which represents the position of values
/// in the array.
///
/// @tparam T Type of the stored elements in the container. T must be
///   default assignable, copyable and assignable.
/// @tparam PS Partition strategy that defines how to partition
///   the original domain into subdomains. The default partition is
/// @ref balanced_partition.
/// @tparam M Mapper that defines how to map the subdomains produced
///   by the partition to locations. The default mapper is @ref mapper.
/// @tparam Traits A traits class that defines customizable components
///   of array, such as the domain type and base container type. The
///   default traits class is @ref array_traits.
////////////////////////////////////////////////////////////////////////
template<typename T,
         typename PS     = use_default,
         typename M      = use_default,
         typename Traits = use_default>
class array;

#else

template<typename T, typename ...OptionalParams>
class array;

#endif // STAPL_DOCUMENTATION_ONLY

} // namespace stapl

#endif // STAPL_CONTAINERS_ARRAY_FWD_HPP
