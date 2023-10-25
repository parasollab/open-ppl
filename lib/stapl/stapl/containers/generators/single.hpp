/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GENERATOR_SINGLE_HPP
#define STAPL_CONTAINERS_GENERATOR_SINGLE_HPP

#include <stapl/domains/indexed.hpp>
#include <limits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Generator container that returns the same element for every
/// access.
///
/// If there is a @ref single_container c, there exists some element x
/// such that c[i] == x for all i.
///
/// @tparam T Type of the stored element in the container.
////////////////////////////////////////////////////////////////////////
template<typename T>
struct single_container
{
  typedef T                       value_type;
  typedef value_type              mapped_type;
  typedef value_type              reference;
  typedef indexed_domain<size_t>  domain_type;
  typedef size_t                  gid_type;

private:
  /// Size of the generator container
  size_t m_size;
  /// Instance of the object that is being stored
  T m_x;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create the container by initializing the single element and
  /// providing a size.
  ///
  /// @param x The element that is to be stored in this container.
  /// @param size Size of this container
  ////////////////////////////////////////////////////////////////////////
  single_container(T const& x, size_t size = std::numeric_limits<size_t>::max())
    : m_size(size), m_x(x)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Retrieve a copy of the element at a specific index. In
  /// actuality, returns the same element for every invocation.
  ///
  /// @param i The index of the element to retrieve (unused)
  /// @return Copy of the stored element.
  ////////////////////////////////////////////////////////////////////////
  value_type get_element(size_t const& i) const
  {
    return m_x;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc get_element
  ////////////////////////////////////////////////////////////////////////
  value_type operator[](size_t const& i) const
  {
    return get_element(i);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the size of this container
  ////////////////////////////////////////////////////////////////////////
  size_t size() const
  { return m_size; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a linear domain of the GIDs in this container
  ////////////////////////////////////////////////////////////////////////
  domain_type domain() const
  { return domain_type(m_size); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Serialization of this class
  ////////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_x);
  }
};

} // stapl namespace

#endif
