/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_BASE_CONTAINER_WRAPPER_HPP
#define STAPL_VIEWS_METADATA_BASE_CONTAINER_WRAPPER_HPP

#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/runtime/executor/anonymous_executor.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a locality metadata container for a given base
///        container (@c BC).
///
/// This class provides the same interface as
/// growable_container. This class is used when a nested
/// execution is invoked over a fast view.
//////////////////////////////////////////////////////////////////////
template<typename BC>
struct base_container_wrapper
{
  typedef typename BC::cid_type                           cid_type;
  typedef typename BC::domain_type                        bc_domain_type;
  typedef bc_domain_type                                  domain_type;
  typedef typename domain_type::index_type                index_type;
  typedef typename domain_type::dimensions_type           dimensions_type;
  typedef metadata_entry<bc_domain_type, BC*, index_type> value_type;
  typedef value_type&                                     reference;
  typedef value_type*                                     iterator;

  value_type m_md;

  void define_type(typer& t)
  {
    t.member(m_md);
  }

  base_container_wrapper(BC* bc)
    : m_md(index_type(), bc->domain(), bc, LQ_CERTAIN, get_affinity(),
           get_anonymous_executor().get_rmi_handle(),
           get_anonymous_executor().get_location_id())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::begin
  //////////////////////////////////////////////////////////////////////
  iterator begin()
  {
    return &m_md;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::end
  //////////////////////////////////////////////////////////////////////
  iterator end()
  {
    return &m_md + 1;
  }

  reference operator[](index_type const&)
  {
   return m_md;
  }

  size_t size() const
  {
    return 1;
  }

  dimensions_type dimensions() const
  {
    return domain().dimensions();
  }

  dimensions_type local_dimensions() const
  {
    return dimensions();
  }

  domain_type domain() const
  {
    return domain_type(index_type(), index_type(), true);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_local_vid(size_t)
  //////////////////////////////////////////////////////////////////////
  index_type get_local_vid(index_type const&)
  {
   return index_type();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::push_back_here(T const&)
  //////////////////////////////////////////////////////////////////////
  template <typename T>
  void push_back_here(T const&)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::local_size
  //////////////////////////////////////////////////////////////////////
  size_t local_size() const
  {
    return 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_location_element(size_t)
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(index_type const& idx) const
  {
    return get_location_id();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::update
  //////////////////////////////////////////////////////////////////////
  void update()
  { }
}; // struct base_container_wrapper

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_BASE_CONTAINER_WRAPPER_HPP
