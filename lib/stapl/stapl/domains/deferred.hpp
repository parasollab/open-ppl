/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_DEFERRED_DOMAIN_HPP
#define STAPL_DOMAINS_DEFERRED_DOMAIN_HPP

#include <stapl/runtime.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief A domain that has been constructed without knowledge of the
/// number of locations across which it will be distributed.
///
/// The domain accepts an instance of the domain it represents and a factory
/// that will reconstruct the domain when the number of locations is available.
/// This is implemented using the assumption that the methods of the domains
/// will not be called until the domain is in the correct communication group,
/// and thus the number of locations is available.
///
/// @todo Either a) defer creation until inside the container's constuctor
/// call chain or (b) create it as needed to begin with using additional
/// information that would be provided by the system_view.
///
/// @note protected inheritance is used to force all calls to domain methods
/// through the deferred_domain interface to allow the domain to be initialized.
/// @note All methods of the domain are not const qualified because any ones
/// of them can result in the initialization of the base domain.
//////////////////////////////////////////////////////////////////////
template<typename Domain>
struct deferred_domain
  : protected Domain
{
private:
  typedef Domain base_type;

  /// Indicate whether the base has been updated with location information.
  bool m_initialized;

  /// Functor that constructs domain given the number of locations.
  std::function<Domain (Domain, location_type)> m_factory;

protected:
  void initialize_domain(void)
  {
    base_type::operator=(m_factory(*this, stapl::get_num_locations()));
    m_initialized = true;
  }

public:
  typedef typename base_type::gid_type   gid_type;
  typedef typename base_type::index_type index_type;
  typedef typename base_type::size_type  size_type;

  deferred_domain(unsigned int)
  { abort("deferred_domain(unsigned int) not supported"); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive a copy of the base domain that hasn't been initialized
  /// with the number of locations and the functor used to generate a copy
  /// of the domain when the location information is available.
  ///
  /// @param other Domain constructed without information on the number of
  /// locations.
  /// @param factory Functor that, given the original domain and the number
  /// of locations will return a new domain that will be used to update the
  /// base class.
  //////////////////////////////////////////////////////////////////////
  deferred_domain(Domain const& other,
    std::function<Domain (Domain const&, location_type)> const& factory)
    : base_type(other), m_initialized(false), m_factory(factory)
  { }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_initialized);
    t.member(m_factory);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the domain with a new last element.
  ///
  /// This method is invoked by the container update_distribution methods.
  //////////////////////////////////////////////////////////////////////
  void update(index_type const& max)
  {
    if (!m_initialized)
      initialize_domain();
    base_type::update(max);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the last index in the domain
  //////////////////////////////////////////////////////////////////////
  index_type first(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::first();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the last index in the domain
  //////////////////////////////////////////////////////////////////////
  index_type last(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::last();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns one-past-the-last index in the domain
  //////////////////////////////////////////////////////////////////////
  index_type open_last(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::open_last();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the given index is present in the domain.
  /// @param idx Index to be checked for membership in the domain.
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type const& idx)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::contains(idx);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of indices in the domain.
  //////////////////////////////////////////////////////////////////////
  size_type size(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::size();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the domain is logically empty.
  //////////////////////////////////////////////////////////////////////
  bool empty(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::empty();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not this domain spans an entire container.
  //////////////////////////////////////////////////////////////////////
  bool is_same_container_domain(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::is_same_container_domain();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Used to examine this class
  /// @param msg message provided to give context to the output
  //////////////////////////////////////////////////////////////////////
  void debug(char* msg = 0)
  {
    std::cerr << "DEFERRED_DOMAIN %p: " << this;
    if (msg)
      std::cerr << msg << "\n";
    if (!m_initialized)
    {
      std::cerr << "initialized = false, initializing now\n";
      initialize_domain();
      std::cerr << "domain after initialization\n";
    } else {
      std::cerr << "initialize = true\ndomain after initialization\n";
    }
    base_type::debug(0);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the index that is the specified distance from the given
  /// index based on the domain ordering.
  ///
  /// @param idx Index base position of the advance operation.
  /// @param d   Number of idicies to advance from the base position.
  /// @return index that is the specified distance from the given index.
  //////////////////////////////////////////////////////////////////////
  template <typename Distance>
  index_type advance(index_type const& idx, Distance d)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::advance(idx, d);
  }
};


template<typename T, int N, typename ...OptionalTraversal>
struct deferred_domain<indexed_domain<T, N, OptionalTraversal...>>
  : protected indexed_domain<T, N, OptionalTraversal...>
{
private:
  typedef indexed_domain<T, N, OptionalTraversal...>          base_type;
  typedef std::function<base_type (base_type, location_type)> factory_type;

  /// Indicate whether the base has been updated with location information.
  bool         m_initialized;

  /// Functor that constructs domain given the number of locations.
  factory_type m_factory;

protected:
  void initialize_domain(void)
  {
    base_type::operator=(m_factory(*this, stapl::get_num_locations()));
    m_initialized = true;
  }

public:
  STAPL_IMPORT_DTYPE(base_type, traversal_type)
  STAPL_IMPORT_DTYPE(base_type, gid_type)
  STAPL_IMPORT_DTYPE(base_type, index_type)
  STAPL_IMPORT_DTYPE(base_type, linear_size_type)
  STAPL_IMPORT_DTYPE(base_type, size_type)

  deferred_domain(unsigned int)
  { abort("deferred_domain(unsigned int) not supported"); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive a copy of the base domain that hasn't been initialized
  /// with the number of locations and the functor used to generate a copy
  /// of the domain when the location information is available.
  ///
  /// @param other Domain constructed without information on the number of
  /// locations.
  /// @param factory Functor that, given the original domain and the number
  /// of locations will return a new domain that will be used to update the
  /// base class.
  //////////////////////////////////////////////////////////////////////
  deferred_domain(factory_type const& factory)
    : m_initialized(false), m_factory(factory)
  { }

  deferred_domain(base_type const& other, factory_type const& factory)
    : base_type(other), m_initialized(false), m_factory(factory)
  { }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_initialized);
    t.member(m_factory);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the last index in the domain
  //////////////////////////////////////////////////////////////////////
  index_type first(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::first();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the last index in the domain
  //////////////////////////////////////////////////////////////////////
  index_type last(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::last();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns one-past-the-last index in the domain
  //////////////////////////////////////////////////////////////////////
  index_type open_last(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::open_last();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the given index is present in the domain.
  /// @param idx Index to be checked for membership in the domain.
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type const& idx)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::contains(idx);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of indices in the domain.
  //////////////////////////////////////////////////////////////////////
  linear_size_type size(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::size();
  }


  size_type dimensions(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::dimensions();
  }


  size_type dimensions(void) const
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::dimensions();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the domain is logically empty.
  //////////////////////////////////////////////////////////////////////
  bool empty(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::empty();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not this domain spans an entire container.
  //////////////////////////////////////////////////////////////////////
  bool is_same_container_domain(void)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::is_same_container_domain();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Update the domain with a new last element.
  ///
  /// This method is invoked by the container update_distribution methods.
  //////////////////////////////////////////////////////////////////////
  void update(index_type const& max)
  {
    if (!m_initialized)
      initialize_domain();
    base_type::update(max);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Used to examine this class
  /// @param msg message provided to give context to the output
  //////////////////////////////////////////////////////////////////////
  void debug(char* msg = 0)
  {
    std::cerr << "DEFERRED_DOMAIN %p: " << this;
    if (msg)
      std::cerr << msg << "\n";
    if (!m_initialized)
    {
      std::cerr << "initialized = false, initializing now\n";
      initialize_domain();
      std::cerr << "domain after initialization\n";
    } else {
      std::cerr << "initialize = true\ndomain after initialization\n";
    }
    base_type::debug(0);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the index that is the specified distance from the given
  /// index based on the domain ordering.
  ///
  /// @param idx Index base position of the advance operation.
  /// @param d   Number of idicies to advance from the base position.
  /// @return index that is the specified distance from the given index.
  //////////////////////////////////////////////////////////////////////
  template <typename Distance>
  index_type advance(index_type const& idx, Distance d)
  {
    if (!m_initialized)
      initialize_domain();
    return base_type::advance(idx, d);
  }
}; // struct deferred_domain<indexed_domain<T, N, OptionalTraversal...>>

}
#endif
