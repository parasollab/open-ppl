/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_CONTAINER_ARRAY_HPP
#define STAPL_VIEWS_METADATA_CONTAINER_ARRAY_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/container/utility.hpp>
#include <stapl/containers/array/static_array.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Container for metadata used when the number of elements
///        (metadata information) is known at the construction time.
///
/// This container behaves as an array.
/// @tparam MD Type of the metadata used to store the metadata information.
/// @todo Verify if this class can be replaced with growable_container.
//////////////////////////////////////////////////////////////////////
template <typename MD>
class flat_container
  : public static_array<MD>
{
  typedef static_array<MD>                                 base_type;

  typedef typename base_type::distribution_type            distribution_t;
  typedef typename distribution_traits<
    distribution_t>::base_container_type                   base_container_type;

public:
  typedef indexed_domain<size_t>                           domain_type;
  typedef domain_type::index_type                          index_type;
  typedef MD                                               value_type;
  typedef std::size_t                                      dimensions_type;

  typedef typename base_container_type::iterator           iterator;

  flat_container(void) = default;

  flat_container(size_t n)
    : base_type(n)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::begin
  //////////////////////////////////////////////////////////////////////
  iterator begin()
  {
    return this->distribution().container_manager().begin()->begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::end
  //////////////////////////////////////////////////////////////////////
  iterator end()
  {
    return this->distribution().container_manager().begin()->end();
  }

  domain_type domain(void) const
  {
    return domain_type(this->size());
  }

  dimensions_type dimensions(void) const
  {
    return this->size();
  }

  dimensions_type local_dimensions(void) const
  {
    return this->local_size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_local_vid(size_t) const
  //////////////////////////////////////////////////////////////////////
  size_t get_local_vid(size_t index)
  {
    return this->distribution().
      container_manager().begin()->domain().first()+index;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_location_element(size_t) const
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(size_t index) const
  {
    auto loc = const_cast<flat_container*>(this)->distribution().
      directory().key_mapper()(index);
    stapl_assert(loc.second != LQ_LOOKUP,
      "flat_container::get_location_element instructed to forward request");
    return loc.first;
  }

  location_type get_location_id() const
  {
    return this->distribution().get_location_id();
  }

  size_t get_num_locations() const
  {
    return this->distribution().get_num_locations();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::local_size
  //////////////////////////////////////////////////////////////////////
  size_t local_size(void) const
  {
    return const_cast<flat_container*>(this)->distribution().
      container_manager().begin()->size();
  }

  void push_back_here(MD const&)
  {
    stapl::abort("cannot add metadata to fixed-size metadata container");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Verify that all the elements that need to be stored in
  ///        this location were assigned.
  //////////////////////////////////////////////////////////////////////
  void update()
  {
    if (this->size() == 0 || this->get_num_locations() == 1) {
      stapl_assert(detail::all_info_set<iterator>(this->begin(), this->end())(),
        "flat_container::update() called before all metadata "
        "entries have been set on its home location");

      return;
    }

    block_until(detail::all_info_set<iterator>(begin(), end()));
  }

};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_CONTAINER_ARRAY_HPP
