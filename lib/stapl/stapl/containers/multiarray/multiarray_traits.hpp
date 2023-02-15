/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIARRAY_TRAITS_HPP
#define STAPL_CONTAINERS_MULTIARRAY_TRAITS_HPP

#include <stapl/containers/distribution/distribution.hpp>

#include <stapl/containers/distribution/operations/base.hpp>
#include <stapl/containers/distribution/operations/random_access.hpp>

#include <stapl/containers/distribution/directory/container_directory.hpp>
#include <stapl/containers/distribution/container_manager/\
container_manager_multidimensional.hpp>
#include <stapl/containers/mapping/multidimensional_mapper.hpp>

#include <stapl/containers/multiarray/base_container.hpp>
#include <stapl/containers/multiarray/base_container_traits.hpp>

#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>

#include <stapl/containers/distribution/container_manager/\
local_partition_info.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the multiarray container. Specifies customizable
/// type parameters that could be changed on a per-container basis.
/// @tparam N The number of dimensions in the multiarray.
/// @tparam T Type of the stored elements in the container.
/// @tparam Traversal The major of the multiarray.
/// @tparam PS Partition strategy that defines how to partition
///   the original domain into subdomains.
/// @tparam M Mapper that defines how to map the subdomains produced
///   by the partition to locations.
/// @ingroup pmultiarrayTraits
////////////////////////////////////////////////////////////////////////
template<int N, class T, class Traversal, class P, class M>
struct multiarray_traits
{
  typedef T                                             value_type;
  typedef Traversal                                     traversal_type;
  typedef P                                             partition_type;
  typedef M                                             mapper_type;
  typedef typename P::value_type                        domain_type;
  typedef typename domain_type::index_type              index_type;
  typedef typename domain_type::gid_type                gid_type;
  typedef typename P::index_type                        cid_type;
  typedef typename domain_type::size_type               size_type;


  typedef typename std::conditional<
    std::is_same<
      typename P::value_type,
      typename multiarray_impl::block_partition<Traversal>::value_type
    >::value
    && std::is_same<
      traversal_type, typename default_traversal<N>::type
    >::value
    && std::is_same<
         typename partition_type::index_type,
         typename multiarray_impl::block_partition<
                    typename default_traversal<N>::type>::index_type
    >::value,
    basic_multiarray_base_container<N, T>,
    multiarray_base_container<T, domain_type, cid_type,
      multiarray_base_container_traits<T, N, Traversal>
    >
  >::type                                               base_container_type;

  typedef container_manager_multidimensional<
    base_container_type>                                container_manager_type;

  typedef container_directory<
    partition_type, mapper_type>                        directory_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the distribution type based on a
  /// container type.
  /// @tparam C Type of the container.
  //////////////////////////////////////////////////////////////////////
  template <typename C>
  struct construct_distribution
  {
    typedef distribution<C, operations::base, operations::random_access> type;
  };
}; // struct multiarray_traits


namespace multiarray_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute traits class for @ref multiarray
///   container. Primary template should not be instantiated.  Partial
///   specialization used to handle dependence of @p partition and
///   @p mapper types on @p traversal.
/// @ingroup pmultiarrayTraits
//////////////////////////////////////////////////////////////////////
template<int N, typename T, typename ...OptionalParams>
struct compute_multiarray_traits;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when none of the optional parameters are
///   provided at template instantiation.  Use @ref multiarray_traits,
///   with default partition and mapper argument types.
/// @ingroup pmultiarrayTraits
//////////////////////////////////////////////////////////////////////
template<int N, typename T>
struct compute_multiarray_traits<N, T>
{
private:
  typedef typename default_traversal<N>::type           traversal_t;
  typedef multiarray_impl::block_partition<traversal_t> partition_t;
  typedef multidimensional_mapper<
    typename partition_t::index_type, traversal_t>      mapper_t;

public:
  typedef multiarray_traits<
    N, T, traversal_t, partition_t, mapper_t>           type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to detect presence of distribution view in
///  multiarray template parameter list and set traits accordingly.
///
/// Primary template covers case when type is not distribution view.
/// Treat it as an explicitly specified partition type.
//////////////////////////////////////////////////////////////////////
template<int N, typename T, typename Q, bool = is_distribution_view<Q>::value>
struct detect_distribution_view_param
{
private:
  typedef multiarray_impl::block_partition<Q>                  partition_t;
  typedef multidimensional_mapper<
    typename partition_t::index_type, Q>                       mapper_t;

public:
  typedef multiarray_traits<N, T, Q, partition_t, mapper_t>    type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization covers case when parameter is a distribution
/// spec.  Partition and mapper are wrapper classes around it.
//////////////////////////////////////////////////////////////////////
template<int N, typename T, typename DistributionSpec>
struct detect_distribution_view_param<N, T, DistributionSpec, true>
{
  typedef multiarray_traits<
    N, T,
    typename DistributionSpec::traversal_type,
    view_based_partition<DistributionSpec>,
    view_based_mapper<DistributionSpec>>         type;
};


template<int N, typename T, typename Q>
struct compute_multiarray_traits<N, T, Q>
  : detect_distribution_view_param<N, T, Q>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when only the @p traversal and @p partition
///   optional parameters are defined.
/// @ingroup pmultiarrayTraits
//////////////////////////////////////////////////////////////////////
template<int N, typename T, typename Traversal, typename Partition>
struct compute_multiarray_traits<N, T, Traversal, Partition>
{
private:
  typedef multidimensional_mapper<
    typename Partition::index_type, Traversal
  >                                                   mapper_t;

public:
  typedef multiarray_traits<
    N, T, Traversal, Partition, mapper_t
  >                                                   type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when only the @p traversal, @p partition and
///   @p Mapper optional parameters are defined.
/// @ingroup pmultiarrayTraits
//////////////////////////////////////////////////////////////////////
template<int N, typename T, typename Traversal,
         typename Partition, typename Mapper>
struct compute_multiarray_traits<N, T, Traversal, Partition, Mapper>
{
  typedef multiarray_traits<
    N, T, Traversal, Partition, Mapper>               type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when traits is actually passed at class
//    template instantiation. Use passed type.
/// @ingroup pmultiarrayTraits
//////////////////////////////////////////////////////////////////////
template<int N, typename T,typename Traversal, typename Partition,
         typename Mapper, typename Traits>
struct compute_multiarray_traits<N, T, Traversal, Partition, Mapper, Traits>
{
  typedef Traits type;
};

} // namespace multiarray_impl

} // namespace stapl

#endif // STAPL_CONTAINERS_MULTIARRAY_TRAITS_HPP

