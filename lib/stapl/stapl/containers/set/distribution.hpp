/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SET_DISTRIBUTION_HPP
#define STAPL_CONTAINERS_SET_DISTRIBUTION_HPP

#include <stapl/containers/distribution/associative_distribution.hpp>
#include <stapl/views/proxy.h>

namespace stapl {

template<typename Container>
class set_distribution;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref distribution_traits for
///   @ref set_distribution.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct distribution_traits<set_distribution<C>>
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
  typedef container_accessor<set_distribution<C>>                accessor_type;
  typedef const_container_accessor<set_distribution<C>>    const_accessor_type;
  typedef proxy<value_type, accessor_type>                       reference;
  typedef proxy<value_type, const_accessor_type>           const_reference;
  typedef container_iterator<
    set_distribution<C>,
    accessor_type,
    iterator_category
  >                                                               iterator;
  typedef const_container_iterator<
    set_distribution<C>,
    const_accessor_type,
    iterator_category
  >                                                         const_iterator;
};


//////////////////////////////////////////////////////////////////////
/// @brief Distribution for the @ref set container.
/// @tparam Container Type of the container that is managing
/// this distribution.
/// @see set
/// @todo Make a pass over the typedefs.  Most can probably be removed.
////////////////////////////////////////////////////////////////////////
template<typename Container>
class set_distribution
    : public associative_distribution<Container, set_distribution<Container>>
{
  typedef associative_distribution<
    Container, set_distribution<Container>>              base_type;

public:

  typedef std::bidirectional_iterator_tag                iterator_category;

  typedef typename base_type::directory_type             directory_type;
  typedef typename base_type::container_manager_type     container_manager_type;

  typedef typename container_manager_type::base_container_type
                                                         base_container_type;
  typedef typename container_manager_type::gid_type      gid_type;
  typedef gid_type                                       index_type;

  typedef typename base_container_type::key_type         key_type;
  typedef typename container_manager_type::value_type    value_type;
  typedef typename directory_type::mapper_type           mapper_type;
  typedef typename mapper_type::value_type               location_type;

  /// The coarsening metadata structure used for the set.
  typedef map_metadata<set_distribution>                 loc_dist_metadata;

  typedef distributed_domain<base_type>                  domain_type;
  typedef container_accessor<set_distribution>           accessor_type;
  typedef proxy<value_type, accessor_type>               reference;
  typedef const_container_accessor<set_distribution>     const_accessor_type;
  typedef proxy<value_type, const_accessor_type>         const_reference;

  typedef container_iterator<
    set_distribution,
    accessor_type,
    iterator_category
  >                                                      iterator;

  template<typename... Args>
  set_distribution(Args&&... args)
    : base_type(std::forward<Args>(args)...)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Gets the key from the pair
  /// @param val The pair of <key_type,mapped_type> to be inserted.
  /// @return A reference to the key of the element
  //////////////////////////////////////////////////////////////////////
  key_type const& get_key(value_type const& val)
  {
    return val;
  }
}; // class set_distribution

} // namespace stapl

#endif // STAPL_CONTAINERS_SET_CONTAINER_DISTRIBUTION_HPP
