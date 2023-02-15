/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MAP_CONTAINER_DISTRIBUTION_HPP
#define STAPL_CONTAINERS_MAP_CONTAINER_DISTRIBUTION_HPP

#include <stapl/containers/distribution/associative_distribution.hpp>
#include <stapl/containers/map/functional.hpp>

#include <stapl/views/proxy.h>

namespace stapl {

template<typename Container>
class map_distribution;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref distribution_traits for
///   @ref map_distribution.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct distribution_traits<map_distribution<C> >
{

  typedef std::bidirectional_iterator_tag                  iterator_category;

  typedef C                                                      container_type;
  typedef typename container_traits<C>::directory_type           directory_type;
  typedef typename container_traits<C>::container_manager_type
                                                         container_manager_type;
  typedef typename container_traits<C>::base_container_type base_container_type;
  typedef typename container_traits<C>::gid_type                 gid_type;
  typedef gid_type                                               index_type;
  typedef typename container_traits<C>::value_type               value_type;
  typedef container_accessor<map_distribution<C> >               accessor_type;
  typedef const_container_accessor<map_distribution<C> >   const_accessor_type;
  typedef proxy<value_type, accessor_type>                       reference;
  typedef proxy<value_type, const_accessor_type>           const_reference;

  typedef container_iterator<
    map_distribution<C> ,
    accessor_type,
    iterator_category
    >                                                            iterator;

  typedef const_container_iterator<
    map_distribution<C> ,
    const_accessor_type,
    iterator_category
    >                                                            const_iterator;
};


//////////////////////////////////////////////////////////////////////
/// @brief Distribution for the @ref map container.
/// @tparam Container Type of the container that is managing
/// this distribution.
/// @see map
/// @todo Make a pass over the typedefs.  Most can probably be removed.
////////////////////////////////////////////////////////////////////////
template<typename Container>
class map_distribution
  : public associative_distribution<Container,map_distribution<Container>>
{
private:
  typedef associative_distribution<
    Container, map_distribution<Container>>              base_type;

public:
  STAPL_IMPORT_TYPE(typename base_type, directory_type)
  STAPL_IMPORT_TYPE(typename base_type, container_manager_type)
  STAPL_IMPORT_TYPE(typename base_type, mapper_type)

  STAPL_IMPORT_TYPE(typename container_manager_type, base_container_type)
  STAPL_IMPORT_TYPE(typename container_manager_type, gid_type)
  STAPL_IMPORT_TYPE(typename container_manager_type, value_type)

  STAPL_IMPORT_TYPE(typename base_container_type, key_type)
  STAPL_IMPORT_TYPE(typename base_container_type, mapped_type)

  typedef std::bidirectional_iterator_tag                iterator_category;

  typedef gid_type                                       index_type;
  typedef typename mapper_type::value_type               location_type;

  /// The coarsening metadata structure used for the map.
  typedef map_metadata<map_distribution>                 loc_dist_metadata;

  typedef distributed_domain<base_type>                  domain_type;
  typedef container_accessor<map_distribution>           accessor_type;
  typedef const_container_accessor<map_distribution>     const_accessor_type;
  typedef proxy<value_type, accessor_type>               reference;
  typedef proxy<value_type, const_accessor_type>         const_reference;
  typedef typename reference::second_reference           second_reference;

  typedef container_iterator<
    map_distribution,
    accessor_type,
    iterator_category
  >                                                      iterator;

  template<typename... Args>
  map_distribution(Args&&... args)
    : base_type(std::forward<Args>(args)...)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to modify the value of an existing element. An element with
  /// an equivalent key should already be in the container.
  /// @param val The <Key,Mapped> pair representing the element
  //////////////////////////////////////////////////////////////////////
  void set_element(value_type const& val)
  {
    set_element(val.first, val.second);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to modify the value of an existing element. An element with
  /// an equivalent key should already be in the container.
  /// @param key The key of the element to update
  /// @param val The mapped value to update it to
  //////////////////////////////////////////////////////////////////////
  void set_element(key_type const& key, mapped_type const& val)
  {
    if (this->m_container_manager.contains(key))
      this->m_container_manager.invoke(key, &base_container_type::set_element,
                                       value_type(key, val));
    else
      this->m_directory.invoke_where(
        std::bind(
          [](p_object& d, key_type const& key, mapped_type const& val)
          {
            down_cast<map_distribution&>(d).set_element(value_type(key, val));
          },
          std::placeholders::_1, std::placeholders::_2, val),
        key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to modify the value of an existing element. An element with
  /// an equivalent key should already be in the container.
  /// @param key The key of the element to update
  /// @param val The <Key,Mapped> pair representing the element
  //////////////////////////////////////////////////////////////////////
  void set_element(key_type const& key, value_type const& val)
  {
    this->set_element(value_type(key, val.second));
  }
}; // class map_distribution

} // namespace stapl

#endif // STAPL_CONTAINERS_MAP_CONTAINER_DISTRIBUTION_HPP
