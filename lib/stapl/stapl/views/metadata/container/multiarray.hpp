/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_CONTAINER_MULTIARRAY_HPP
#define STAPL_VIEWS_METADATA_CONTAINER_MULTIARRAY_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/container/utility.hpp>
#include <stapl/containers/multiarray/multiarray_fwd.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Container for metadata that can be represented as a hyper-rectangle
///        and whose number of elements (metadata information) is known
///        at construction time.
///
/// This container behaves as a @see multiarray.
///
/// @tparam MD Type of the metadata used to store the metadata information.
///
/// @todo  Only volumetric distributions specified by the dimensions tuple or
///        partition type can be used at this moment. Extend with more
///        general distributions supported by the multiarray base class.
//////////////////////////////////////////////////////////////////////
template<typename MD>
class multidimensional_container
  : public multiarray<
             dimension_traits<typename MD::index_type>::type::value,
             MD,
             typename MD::domain_type::traversal_type
            >
{
  typedef multiarray<
    dimension_traits<typename MD::index_type>::type::value,
    MD,
    typename MD::domain_type::traversal_type
  >                                                        base_type;

  using base_container_type =
    typename base_type::distribution_type::base_container_type;


public:
  typedef typename base_type::dimensions_type              dimensions_type;
  typedef typename base_type::partition_type               partition_type;
  typedef typename base_type::domain_type                  domain_type;
  typedef typename MD::index_type                          index_type;
  typedef MD                                               value_type;

  using iterator =
    typename base_container_type::container_type::iterator;


  multidimensional_container(void) = default;

  multidimensional_container(dimensions_type const& dims)
    : base_type(dims)
  { }

  multidimensional_container(partition_type const& part)
    : base_type(part)
  { }

  index_type get_local_vid(index_type const& index)
  {
    stapl_assert(this->distribution().container_manager().size() <= 1,
      "This method can only be used when there is at most one base container at"
      " the location");

    if (this->distribution().container_manager().size() == 0)
      return index_type();

    auto const d = this->distribution().container_manager().begin()->domain();

    return nd_globalize<index_type>::apply(index, d.first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_location_element(size_t) const
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(index_type const& index) const
  {
    auto location = const_cast<multidimensional_container*>(this)->
      distribution().directory().key_mapper()(index);
    stapl_assert(location.second != LQ_LOOKUP,
    "multidimensional_container::get_location_element instructed to forward.");
    return location.first;
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
    stapl_assert(this->distribution().container_manager().size() <= 1,
      "This method can only be used when there is at most one base container at"
      " the location");

    return (this->distribution().container_manager().size() != 0) ?
      this->distribution().container_manager().begin()->size() : 0;
  }

  dimensions_type local_dimensions(void) const
  {
    stapl_assert(this->distribution().container_manager().size() <= 1,
      "This method can only be used when there is at most one base container at"
      " the location");

    return (this->distribution().container_manager().size() != 0) ?
      this->distribution().container_manager().begin()->dimensions() :
      dimensions_type();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a begin iterator for the metadata entries on this location
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    stapl_assert(this->distribution().container_manager().size() <= 1,
      "This method can only be used when there is at most one base container at"
      " the location");

    if (this->distribution().container_manager().size() == 0)
      return iterator();

    return
      this->distribution().container_manager().begin()->container().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an end iterator for the metadata entries on this location
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    stapl_assert(this->distribution().container_manager().size() <= 1,
      "This method can only be used when there is at most one base container at"
      " the location");

    if (this->distribution().container_manager().size() == 0)
      return iterator();

    return
      this->distribution().container_manager().begin()->container().end();
  }

  void push_back_here(MD const&)
  {
    stapl::abort("cannot add metadata to fixed-size metadata container");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Wait until the metadata entries on this location are accounted
  ///        for.
  //////////////////////////////////////////////////////////////////////
  void update(void)
  {
    if (this->get_num_locations() == 1) {
      stapl_assert(detail::all_info_set<iterator>(this->begin(), this->end())(),
        "multidimensional_container::update() called before all metadata "
        "entries have been set on its home location");

      return;
    }

    block_until(detail::all_info_set<iterator>(this->begin(), this->end()));
  }

};

} // namespace metadata


} // namespace stapl

#endif // STAPL_VIEWS_METADATA_CONTAINER_MULTIARRAY_HPP
