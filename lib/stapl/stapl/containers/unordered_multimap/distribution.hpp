/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MULTIMAP_DISTRIBUTION_HPP
#define STAPL_CONTAINERS_UNORDERED_MULTIMAP_DISTRIBUTION_HPP

#include <stapl/containers/distribution/multi_associative_distribution.hpp>
#include <stapl/containers/unordered_map/distribution.hpp>
#include <stapl/views/proxy.h>

namespace stapl {

template<typename Container>
class unordered_multimap_distribution;

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref distribution_traits for
///   @ref unordered_multimap_distribution.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct distribution_traits<unordered_multimap_distribution<C> >
{
  typedef C                                              container_type;
  typedef typename container_traits<C>::directory_type   directory_type;
  typedef
    typename container_traits<C>::container_manager_type container_manager_type;
  typedef
    typename container_traits<C>::base_container_type    base_container_type;
  typedef typename container_traits<C>::gid_type         gid_type;
  typedef gid_type                                       index_type;
  typedef typename container_traits<C>::value_type       value_type;
  typedef value_type                                     stored_type;

  typedef container_accessor<
            unordered_multimap_distribution<C> >         accessor_type;
  typedef proxy<value_type, accessor_type>               reference;
  typedef container_iterator<
    unordered_multimap_distribution<C>, accessor_type>   iterator;

  typedef const_container_accessor<
            unordered_multimap_distribution<C> >         const_accessor_type;
  typedef proxy<value_type, const_accessor_type>         const_reference;
  typedef const_container_iterator<
    unordered_multimap_distribution<C>,
    const_accessor_type>                                 const_iterator;
};

//////////////////////////////////////////////////////////////////////
/// @brief Distribution for the @ref unordered_multimap container.
/// @tparam Container Type of the container that is managing
/// this distribution.
/// @see unordered_multimap
////////////////////////////////////////////////////////////////////////
template<typename Container>
class unordered_multimap_distribution
  : public multi_associative_distribution<
             Container, unordered_multimap_distribution<Container> >
{
  typedef multi_associative_distribution<Container,
            unordered_multimap_distribution<Container> > base_type;

public:
  typedef typename base_type::directory_type             directory_type;
  typedef typename base_type::container_manager_type     container_manager_type;
  typedef
    typename container_manager_type::base_container_type base_container_type;
  typedef typename base_container_type::gid_type         gid_type;
  typedef gid_type                                       index_type;
  typedef typename base_container_type::key_type         key_type;
  typedef typename base_container_type::mapped_type      mapped_type;
  typedef typename base_container_type::value_type       value_type;
  typedef value_type                                     stored_type;
  typedef typename directory_type::mapper_type           mapper_type;
  typedef typename mapper_type::value_type               location_type;

  /// The coarsen meta data structure used for the unordered_multimap.
  typedef map_metadata<unordered_multimap_distribution>  loc_dist_metadata;
  typedef distributed_domain<base_type>                  domain_type;

  typedef container_accessor<
            unordered_multimap_distribution>             accessor_type;

  using Accessor = accessor_type;

  STAPL_PROXY_SELECTOR_MEMBER(second)

  typedef proxy<value_type, accessor_type>               reference;
  typedef container_iterator<
            unordered_multimap_distribution,
            accessor_type>                               iterator;

  typedef const_container_accessor<
            unordered_multimap_distribution>             const_accessor_type;
  typedef proxy<value_type, const_accessor_type>         const_reference;
  typedef const_container_iterator<
            unordered_multimap_distribution,
            const_accessor_type>                         const_iterator;

private:
  typedef member_accessor<mapped_type,
   typename reference::template get_second<mapped_type>,
   accessor_type>                                        second_accessor_type;
public:
  typedef proxy<mapped_type, second_accessor_type>       second_reference;

  friend class distributed_domain<unordered_multimap_distribution>;

  unordered_multimap_distribution() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a directory.
  /// @param directory Container directory that is responsible for global
  ///   metadata.
  ////////////////////////////////////////////////////////////////////////
  unordered_multimap_distribution(directory_type const& directory)
    : base_type(directory)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a partition and a mapper.
  /// @param partition Partition used by the container
  /// @param mapper Mapper used by the container
  ////////////////////////////////////////////////////////////////////////
  template <typename PS, typename Map>
  unordered_multimap_distribution(PS const& partition, Map const& mapper)
    : base_type(partition, mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the key value from the type value_type
  /// @param val The key value
  /// @return A reference to the key of the element
  //////////////////////////////////////////////////////////////////////
  gid_type get_key(value_type const& val)
  {
    return val.first;
  }

}; // class unordered_multimap_distibution

} // namespace stapl

#endif // STAPL_CONTAINERS_UNORDERED_MULTIMAP_DISTRIBUTION_HPP
