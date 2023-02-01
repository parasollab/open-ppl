/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MAP_DISTRIBUTION_HPP
#define STAPL_CONTAINERS_UNORDERED_MAP_DISTRIBUTION_HPP

#include <stapl/containers/iterators/container_iterator_fwd.hpp>
#include <stapl/containers/iterators/const_container_iterator_fwd.hpp>
#include <stapl/containers/distribution/associative_distribution.hpp>
#include <stapl/views/proxy.h>

namespace stapl {

template<typename Container>
class unordered_map_distribution;

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref distribution_traits for
///   @ref unordered_map_distribution.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct distribution_traits<unordered_map_distribution<C> >
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
    unordered_map_distribution<C> >                     accessor_type;
  typedef proxy<value_type, accessor_type>              reference;
  typedef container_iterator<
    unordered_map_distribution<C>, accessor_type>       iterator;

  typedef const_container_accessor<
    unordered_map_distribution<C> >                     const_accessor_type;
  typedef proxy<value_type, const_accessor_type>        const_reference;
  typedef const_container_iterator<
    unordered_map_distribution<C>, const_accessor_type> const_iterator;
};


//////////////////////////////////////////////////////////////////////
/// @brief Distribution for the @ref unordered_map container.
/// @tparam Container Type of the container that is managing
/// this distribution.
/// @see unordered_map
////////////////////////////////////////////////////////////////////////
template<typename Container>
class unordered_map_distribution
  : public associative_distribution<
            Container, unordered_map_distribution<Container> >
{
  typedef associative_distribution<Container,
            unordered_map_distribution<Container> >     base_type;

public:
  STAPL_IMPORT_TYPE(typename base_type, directory_type)
  STAPL_IMPORT_TYPE(typename base_type, container_manager_type)
  STAPL_IMPORT_TYPE(typename container_manager_type, base_container_type)
  STAPL_IMPORT_TYPE(typename base_container_type, gid_type)
  STAPL_IMPORT_TYPE(typename base_container_type, value_type)
  STAPL_IMPORT_TYPE(typename base_container_type, key_type)
  STAPL_IMPORT_TYPE(typename base_container_type, mapped_type)

  typedef gid_type                                       index_type;

  typedef typename base_container_type::stored_type      stored_type;
  typedef typename directory_type::mapper_type           mapper_type;
  typedef typename mapper_type::value_type               location_type;
  /// The coarsen meta data structure used for the unordered_map.
  typedef map_metadata<unordered_map_distribution>       loc_dist_metadata;
  typedef distributed_domain<base_type>                  domain_type;

  typedef container_accessor<unordered_map_distribution> accessor_type;

  using Accessor = accessor_type;

  STAPL_PROXY_SELECTOR_MEMBER(second)

  typedef proxy<value_type, accessor_type>               reference;
  typedef container_iterator<
    unordered_map_distribution, accessor_type>           iterator;

  typedef typename reference::second_reference           second_reference;

  typedef const_container_accessor<
    unordered_map_distribution>                          const_accessor_type;
  typedef proxy<value_type, const_accessor_type>         const_reference;
  typedef const_container_iterator<
    unordered_map_distribution, const_accessor_type>     const_iterator;

  friend class distributed_domain<unordered_map_distribution>;

  unordered_map_distribution(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a directory.
  /// @param directory Container directory that is responsible for global
  ///   metadata.
  ////////////////////////////////////////////////////////////////////////
  unordered_map_distribution(directory_type const& directory)
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
  unordered_map_distribution(PS const& partition, Map const& mapper)
    : base_type(partition, mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a partition, a mapper, and a flag for
  ///   the registry.
  /// @param partition Partition used by the container
  /// @param mapper Mapper used by the container
  /// @param reg Flag to indicate that the GIDs of the container are not to be
  ///   registered
  ////////////////////////////////////////////////////////////////////////
  template <typename PS, typename Map, typename Reg>
  unordered_map_distribution(PS const& partition, Map const& mapper,
    Reg const& reg)
    : base_type(partition, mapper, reg)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to modify the value of an existing element. An element with
  ///   an equivalent key should already be in the container.
  /// @param val The <Key,Mapped> pair representing the element
  //////////////////////////////////////////////////////////////////////
  void set_element(value_type const& val)
  {
    if (this->m_container_manager.contains(val.first))
      this->m_container_manager.invoke(val.first,
        &base_container_type::set_element, val);
    else
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, value_type const& val)
            { down_cast<unordered_map_distribution&>(d).set_element(val); },
          std::placeholders::_1, val),
        val.first);
  }
}; // class unordered_map_distibution

} // namespace stapl

#endif // STAPL_CONTAINERS_UNORDERED_MAP_DISTRIBUTION_HPP
