/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_CONTAINER_WRAPPER_HPP
#define STAPL_VIEWS_METADATA_CONTAINER_WRAPPER_HPP

#include <stapl/domains/indexed.hpp>

namespace stapl {

namespace metadata {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a metadata entity that does not store information
///        about locality, instead uses the given distribution (@c D)
///        to respond to the information requests.
///
/// This class provides the same interface of metadata_entry.
/// @see metadata_entry.
//////////////////////////////////////////////////////////////////////
template <typename D>
struct metadata_info
{
  typedef D                                                distribution_type;
  typedef typename distribution_traits<
    distribution_type>::base_container_type                base_container_type;
  typedef typename base_container_type::domain_type        domain_type;
  typedef base_container_type*                             component_type;
  typedef typename domain_type::index_type                 index_type;
  typedef typename distribution_type::cid_type             cid_type;
  typedef metadata_entry<
    domain_type, component_type, size_t
  >                                                        dom_info_type;

  distribution_type&     m_dist;
  cid_type               m_id;
  mutable dom_info_type  m_md_entry;
  mutable bool           m_md_initialized;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor based on a given distribution and an index of
  ///        the base container to use to determine the domain.
  ///
  /// @param dist Container's distribution.
  /// @param id Index of the base container to use.
  //////////////////////////////////////////////////////////////////////
  metadata_info(distribution_type* dist, size_t id)
    : m_dist(*dist), m_id(id), m_md_initialized(false)
  { }

  location_type location() const
  {
    return m_id;
  }

  affinity_tag affinity() const
  {
    return get_affinity();
  }

  rmi_handle::reference handle() const
  {
    return m_dist.get_rmi_handle();
  }

  loc_qual location_qualifier() const
  {
    return LQ_CERTAIN;
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method to return the associated metadata information.
  /// @todo Remove the sync_rmi communication with some kind of lazy
  ///       metadata entity.
  //////////////////////////////////////////////////////////////////////
  dom_info_type& get_metadata() const
  {
    if (m_md_initialized)
      return m_md_entry;

    if (m_dist.get_location_id() == m_id)
    {
      base_container_type& tmp = *(m_dist.container_manager().begin());

      m_md_entry = dom_info_type(
        m_id, tmp.domain(), &tmp, location_qualifier(), affinity(),
        handle(), location()
      );
    }
    else
    {
      if (m_md_entry.component() == NULL)
      {
        m_md_entry = sync_rmi(m_id, m_dist.get_rmi_handle(),
                              &D::metadata_of, m_id);
      }
    }

    m_md_initialized = true;

    return m_md_entry;
  }

public:
  size_t size() const
  {
    return this->domain().size();
  }

  cid_type id() const
  {
    return m_id;
  }

  component_type component() const
  {
    return get_metadata().component();
  }

  domain_type domain() const
  {
    return get_metadata().domain();
  }

  void define_type(typer &t)
  {
    t.member(m_dist);
    t.member(m_id);
  }
}; // struct metadata_info

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Metadata container that wraps a distribution of a pContainer.
///
/// This class uses the given distribution to implement the methods
/// without incurring new storage creation for the locality metadata.
/// This metadata container is used for @ref static_array when the
/// number of base containers and its partition is fixed (no migration
/// is supported).
//////////////////////////////////////////////////////////////////////
template<typename D>
struct container_wrapper
{
  typedef D                                                   distribution_type;

  typedef size_t                                              index_type;
  typedef size_t                                              dimensions_type;
  typedef indexed_domain<index_type>                          domain_type;

  typedef detail::metadata_info<distribution_type>            value_type;
  typedef value_type&                                         reference;
  typedef value_type*                                         iterator;

  distribution_type&  m_dist;
  value_type          m_ref;

  void define_type(typer& t)
  {
    t.member(m_dist);
    t.member(m_ref);
  }

  container_wrapper(distribution_type* dist)
    : m_dist(*dist), m_ref(dist, dist->get_location_id())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::begin
  //////////////////////////////////////////////////////////////////////
  iterator begin()
  {
    return &m_ref;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::end
  //////////////////////////////////////////////////////////////////////
  iterator end()
  {
    return &m_ref + 1;
  }

  value_type operator[](size_t index)
  {
    return value_type(&m_dist, index);
  }

  size_t size() const
  {
    return m_dist.get_num_locations();
  }

  dimensions_type dimensions() const
  {
    return this->size();
  }

  domain_type domain() const
  {
    return domain_type(0, this->size()-1);
  }

 //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_local_vid(size_t)
  //////////////////////////////////////////////////////////////////////
  size_t get_local_vid(size_t)
  {
    return m_dist.get_location_id();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::push_back_here(T cons&)
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

  dimensions_type local_dimensions() const
  {
    return 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_location_element(size_t)
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(size_t index) const
  {
    return index;
  }

  void update()
  { }
};

} // namespace metadata

} // namespace stapl

#endif /// STAPL_VIEWS_METADATA_CONTAINER_WRAPPER_HPP
