/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_CONTAINER_REVERSE_HPP
#define STAPL_VIEWS_METADATA_CONTAINER_REVERSE_HPP

#include <stapl/views/core_view.hpp>
#include <stapl/views/operations/view_iterator.hpp>
#include <stapl/views/operations/const_view_iterator.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/domains/reversed.hpp>

#include <stapl/views/metadata/extraction/domain_container.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>

#include <iostream>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Metadata container used for views that represent an
///        elements in reverse order (e.g. @see reverse_view).
///
/// @tparam MDContainer Container for metadata of the original view
///         that is being reversed.
//////////////////////////////////////////////////////////////////////
template<typename MDContainer>
struct reverse_container
{
  using md_cont_t = typename std::remove_pointer<
                      typename MDContainer::second_type>::type;

  typedef typename md_cont_t::value_type                  value_t;
  typedef reversed_domain<typename value_t::domain_type>  dom_t;
  typedef metadata_entry<dom_t,
                      typename value_t::component_type>   value_type;
  typedef value_type                                      reference;

  typedef indexed_domain<size_t>                          domain_type;
  typedef size_t                                          dimensions_type;

  md_cont_t*   m_md;
  size_t      m_total_size;

  reverse_container(md_cont_t* md, size_t total_size)
    : m_md(md),
      m_total_size(total_size)
  { }

  ~reverse_container()
  { delete m_md; }

  reference operator[](size_t idx)
  {
    const size_t last_idx = std::min(m_total_size, m_md->size())-1;

    value_t orig_md = (*m_md)[last_idx - idx];

    return value_type(
      last_idx - orig_md.id(),
      dom_t(orig_md.domain(), m_total_size),
      orig_md.component(),
      orig_md.location_qualifier(), orig_md.affinity(),
      orig_md.handle(), orig_md.location()
    );
  }

  size_t size(void) const
  {
    return std::min(m_total_size, m_md->size());
  }

  size_t dimensions(void) const
  {
    return this->size();
  }

  domain_type domain(void) const
  {
    return domain_type(0, this->size());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the global index of the given local @p index.
  //////////////////////////////////////////////////////////////////////
  size_t get_local_vid(size_t index)
  {
   return m_md->get_local_vid(index);
  }

  size_t local_size(void) const
  {
    return get_location_id() < m_total_size ? m_md->local_size() : 0;
  }

  dimensions_type local_dimensions(void) const
  {
    return m_md->local_dimensions();
  }

  location_type location(size_t idx)
  {
    return (m_md)[(m_md->size()-1)-idx].get_location();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the location where the metadata indexed by @p idx
  ///        is located.
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(size_t idx) const
  {
    return m_md->get_location_element(idx);
  }

  void push_back_here(value_type)
  { }

  void update(void)
  { }
};

} // namespace metadata

template<typename MD>
struct convert_to_md_vec_array<metadata::reverse_container<MD> >
{
  typedef MD                                    value_type;
  typedef metadata::reverse_container<MD>       part_type;
  typedef metadata::view<part_type>             type;
};

} // namespace stapl

#endif
