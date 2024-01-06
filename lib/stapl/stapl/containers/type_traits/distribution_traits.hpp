/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_TRAITS_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_TRAITS_HPP

#include <stapl/containers/iterators/container_accessor.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to retrieve associated types for container
/// distributions.
/// This class is similar in spirit to the STL's iterator_traits.
/// and @ref container_traits.
///
/// @tparam Dist The distribution for which to query.
//////////////////////////////////////////////////////////////////////
template<typename Dist>
struct distribution_traits
{
  STAPL_IMPORT_TYPE(typename Dist, container_type)
  STAPL_IMPORT_TYPE(typename Dist, directory_type)
  STAPL_IMPORT_TYPE(typename Dist, container_manager_type)
  STAPL_IMPORT_TYPE(typename Dist, base_container_type)
  STAPL_IMPORT_TYPE(typename Dist, gid_type)
  STAPL_IMPORT_TYPE(typename Dist, value_type)

  typedef gid_type                                index_type;
  typedef typename Dist::reference                reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when distribution is customized by inheriting
/// from @ref distribution base.  Type is incomplete, reflect types from
/// the container_traits of the distribution's container.
///
/// This has the implicit assumption that the distribution has a single
/// parameter, which is the container type.
///
/// @tparam DistributionType The class template for the distribution
/// @tparam C The single template parameter of DistributionType, which
/// is the container type.
//////////////////////////////////////////////////////////////////////
template<template<typename> class DistributionType, typename C>
struct distribution_traits<DistributionType<C>>
{
  typedef C                                             container_type;

  STAPL_IMPORT_TYPE(typename container_traits<C>, directory_type)
  STAPL_IMPORT_TYPE(typename container_traits<C>, container_manager_type)
  STAPL_IMPORT_TYPE(typename container_traits<C>, base_container_type)
  STAPL_IMPORT_TYPE(typename container_traits<C>, gid_type)
  STAPL_IMPORT_TYPE(typename container_traits<C>, value_type)

  typedef gid_type                                      index_type;
  typedef proxy<value_type, container_accessor<C> >     reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when distribution uses @ref distribution,
/// optionally passing operation template template parameters. Type is
/// incomplete, reflect types from the container_traits of the distribution's
/// container.
///
/// This has the implicit assumption that the distribution has a parameters
/// of the container type followed by optional operation classes.
///
/// @tparam DistributionType The class template for the distribution
/// @tparam C The single template parameter of DistributionType, which
///   is the container type.
///
/// @todo use Variadic inheritance until intel can parse it properly
///   (right now it throws "error: duplicate base class name").
/// template<template<typename, template<typename> class...>
///   class DistributionType,
///          typename C, template<typename> class... Operations>
/// struct distribution_traits<DistributionType<C, Operations...>>
//////////////////////////////////////////////////////////////////////
template<template<typename, template<typename> class, template<typename> class,
                  template<typename> class, template<typename> class>
            class DistributionType,
         typename C,
         template<typename> class Op1,
         template<typename> class Op2,
         template<typename> class Op3,
         template<typename> class Op4>
struct distribution_traits<DistributionType<C, Op1, Op2, Op3, Op4>>
{
  typedef C                                             container_type;

  STAPL_IMPORT_TYPE(typename container_traits<C>, directory_type)
  STAPL_IMPORT_TYPE(typename container_traits<C>, container_manager_type)
  STAPL_IMPORT_TYPE(typename container_traits<C>, base_container_type)
  STAPL_IMPORT_TYPE(typename container_traits<C>, gid_type)
  STAPL_IMPORT_TYPE(typename container_traits<C>, value_type)

  typedef gid_type                                      index_type;
  typedef proxy<value_type, container_accessor<C>>      reference;
};

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_TRAITS_HPP
