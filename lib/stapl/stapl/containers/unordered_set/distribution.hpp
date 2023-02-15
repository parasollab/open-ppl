/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_SET_DISTRIBUTION_HPP
#define STAPL_CONTAINERS_UNORDERED_SET_DISTRIBUTION_HPP

#include <stapl/containers/distribution/associative_distribution.hpp>
#include <stapl/containers/iterators/const_container_accessor.hpp>
#include <stapl/containers/iterators/const_container_iterator.hpp>
#include <stapl/views/proxy.h>

namespace stapl {

template<typename Container>
class unordered_set_distribution;

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref distribution_traits for
///   @ref unordered_set_distribution.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct distribution_traits<unordered_set_distribution<C> >
{
  typedef C                                              container_type;
  typedef typename container_traits<C>::directory_type   directory_type;
  typedef
    typename container_traits<C>::container_manager_type container_manager_type;
  typedef
    typename container_traits<C>::base_container_type    base_container_type;
  typedef typename container_traits<C>::key_type         key_type;
  typedef key_type                                       mapped_type;
  typedef typename container_traits<C>::gid_type         gid_type;
  typedef gid_type                                       index_type;
  typedef typename container_traits<C>::value_type       value_type;
  typedef const_container_accessor<
    unordered_set_distribution<C> >                      const_accessor_type;
  typedef proxy<value_type, const_accessor_type>         const_reference;
  typedef const_container_iterator<
    unordered_set_distribution<C>, const_accessor_type>  const_iterator;
  typedef const_accessor_type                            accessor_type;
  typedef const_reference                                reference;
  typedef const_iterator                                 iterator;
};


//////////////////////////////////////////////////////////////////////
/// @brief Distribution for the @ref unordered_set container.
/// @tparam Container Type of the container that is managing
/// this distribution.
/// @see unordered_set
////////////////////////////////////////////////////////////////////////
template<typename Container>
class unordered_set_distribution
  : public associative_distribution<
            Container, unordered_set_distribution<Container> >
{
  typedef associative_distribution<Container,
            unordered_set_distribution<Container> >     base_type;

public:
  typedef typename base_type::directory_type            directory_type;
  typedef typename base_type::container_manager_type    container_manager_type;
  typedef typename container_manager_type::base_container_type
                                                        base_container_type;

  typedef typename container_manager_type::gid_type     gid_type;
  typedef gid_type                                      index_type;

  typedef typename base_container_type::key_type        key_type;
  typedef key_type                                      mapped_type;
  typedef typename base_container_type::value_type      value_type;
  typedef typename directory_type::mapper_type          mapper_type;
  typedef typename mapper_type::value_type              location_type;
  /// The coarsen meta data structure used for the unordered_set.
  typedef map_metadata<unordered_set_distribution>      loc_dist_metadata;
  typedef distributed_domain<base_type>                 domain_type;
  typedef const_container_accessor<
    unordered_set_distribution<Container> >             const_accessor_type;
  typedef proxy<value_type, const_accessor_type>        const_reference;
  typedef const_container_iterator<
    unordered_set_distribution<Container>,
    const_accessor_type>                                const_iterator;
  typedef const_accessor_type                           accessor_type;

  using Accessor = accessor_type;

  STAPL_PROXY_SELECTOR_MEMBER(second)

  typedef const_reference                               reference;
  typedef const_iterator                                iterator;

  friend class distributed_domain<unordered_set_distribution>;

  unordered_set_distribution(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a directory.
  /// @param directory Container directory that is responsible for global
  ///   metadata.
  ////////////////////////////////////////////////////////////////////////
  unordered_set_distribution(directory_type const& directory)
    : base_type(
        directory,
        container_manager_type(
          directory.partition(), directory.mapper(), value_type()
        )
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a partition and a mapper.
  /// @param partition Partition used by the container
  /// @param mapper Mapper used by the container
  ////////////////////////////////////////////////////////////////////////
  template <typename PS, typename Map>
  unordered_set_distribution(PS const& partition, Map const& mapper)
    : base_type(partition, mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the key value from the type value_type
  /// @param val The key value
  /// @return A reference to the key of the element
  //////////////////////////////////////////////////////////////////////
  key_type const& get_key(value_type const& val)
  {
    return val;
  }

}; // class unordered_set_distibution

} // namespace stapl

#endif // STAPL_CONTAINERS_UNORDERED_SET_DISTRIBUTION_HPP
