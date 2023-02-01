/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_VIEWS_METADATA_CONTAINER_FWD_HPP
#define STAPL_VIEWS_METADATA_CONTAINER_FWD_HPP

#include <stapl/views/mapping_functions/identity.hpp>
#include <stapl/views/metadata/metadata_traits.hpp>
#include <stapl/domains/indexed.hpp>

namespace stapl {

namespace metadata {

template <typename MD>
class growable_container;


template <typename MD>
class flat_container;


template<typename V, typename MD>
class projected_container;

template<typename MD>
class multidimensional_container;


template <typename T, typename Domain, bool Isomorphic>
struct view_base;

template<typename C,
         typename Dom     = indexed_domain<size_t>,
         typename MapFunc = f_ident<typename Dom::index_type> >
class view;


template<typename T, typename Domain, bool Isomorphic=false>
struct view_wrapper;

} // namespace metadata

template<typename MD>
struct metadata_traits<metadata::multidimensional_container<MD>>
{
  using is_isomorphic = std::true_type;
  using value_type = MD;
};

} // namespace stapl

#endif /* STAPL_VIEWS_METADATA_CONTAINER_FWD_HPP */
