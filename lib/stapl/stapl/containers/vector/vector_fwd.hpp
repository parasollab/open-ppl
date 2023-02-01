/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_VECTOR_FWD_HPP
#define STAPL_CONTAINERS_VECTOR_FWD_HPP

#include <stapl/utility/use_default.hpp>

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief Parallel Vector container.
/// @ingroup pvector
/// @tparam T Type of the stored elements in the container.
/// @tparam PS Partition strategy that defines how to partition
///         the original domain into subdomains. The default partition is
///         @ref balanced_partition.
/// @tparam M Mapper that defines how to map the subdomains produced
///         by the partition to locations. The default mapper is @ref mapper.
/// @tparam Traits A traits class that defines customizable components
///         of the vector container.
//////////////////////////////////////////////////////////////////////
template<typename T,
         typename PS     = use_default,
         typename M      = use_default,
         typename Traits = use_default>
class vector;

#else

template<typename T, typename ...OptionalParams>
class vector;

#endif // STAPL_DOCUMENTATION_ONLY

} // namespace stapl

#endif // STAPL_CONTAINERS_VECTOR_FWD_HPP
