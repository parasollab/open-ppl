/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GENERATOR_IDENTITY_HPP
#define STAPL_CONTAINERS_GENERATOR_IDENTITY_HPP

#include <stapl/domains/indexed.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Generator container whose value for the element at an index
/// is the index itself.
///
/// That is, if we have an identity_container ic, then ic[i] == i must
/// be true.
///
/// @tparam T Type of the index
/// @todo Add size() and domain() methods
////////////////////////////////////////////////////////////////////////
template<typename T>
struct identity_container
{
  typedef T                   value_type;
  typedef value_type          reference;
  typedef value_type          const_reference;
  typedef value_type          index_type;
  typedef index_type          gid_type;

  //////////////////////////////////////////////////////////////////////
  /// @copydoc get_element
  ////////////////////////////////////////////////////////////////////////
  reference operator[](index_type const& idx) const
  { return idx; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Retrieve the value associated with a particular index (i.e.,
  /// return the index.)
  ///
  /// @param idx The index of the element to retrieve
  /// @return Copy of the index.
  ////////////////////////////////////////////////////////////////////////
  reference get_element(index_type const& idx) const
  { return idx; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the version of the container.
  /// @return Version number of 0
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return 0;
  }

};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for @ref identity_container
///
/// @tparam T Type of value returned by identity_container access methods
//////////////////////////////////////////////////////////////////////
template <typename T>
struct container_traits<identity_container<T>>
{
private:
  typedef identity_container<T> Container;
public:
  typedef typename Container::gid_type        gid_type;
  typedef typename Container::value_type      value_type;
  typedef typename Container::reference       reference;
  typedef typename Container::const_reference const_reference;
  typedef indexed_domain<T>                   domain_type;
};
} // stapl namespace

#endif /* STAPL_CONTAINERS_GENERATOR_IDENTITY_HPP */
