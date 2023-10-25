/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIARRAY_FWD_HPP
#define STAPL_CONTAINERS_MULTIARRAY_FWD_HPP

#include <stapl/utility/use_default.hpp>

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief Parallel multidimensional array container.
/// @ingroup pmultiarray
///
/// @tparam N The number of dimensions in the multiarray.
/// @tparam T Type of the stored elements in the container. T must be
/// default assignable, copyable and assignable.
/// @tparam Traversal The major of the multiarray. This type must be a
/// compile-time @ref index_sequence. The default traversal can be computed
/// by the @ref default_traversal metafunction.
/// @tparam PS Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref multiarray_impl::block_partition.
/// @tparam M Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is
///   @ref multidimensional_mapper.
/// @tparam Traits A traits class that defines customizable components
/// of multiarray, such as the domain type and base container type. The
/// default traits class is @ref multiarray_traits.
////////////////////////////////////////////////////////////////////////
template<int N, typename T,
         typename Traversal = typename default_traversal<N>::type,
         typename PS        = multiarray_impl::block_partition<Traversal>,
         typename M         = multidimensional_mapper<
           typename PS::index_type, Traversal>,
         typename Traits    = use_default>
class multiarray;

#else

template<int N, typename T, typename ...OptionalParams>
class multiarray;

#endif

} // namespace stapl

#endif // STAPL_CONTAINERS_MULTIARRAY_FWD_HPP
