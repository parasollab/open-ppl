/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_PARTITIONED_MIX_VIEW_HPP
#define STAPL_VIEWS_METADATA_PARTITIONED_MIX_VIEW_HPP

#include <stapl/runtime/p_object.hpp>

#include <stapl/views/array_ro_view.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/mapping_functions/mapping_functions.hpp>
#include <stapl/views/operations/paragraph_requirements.hpp>
#include <stapl/views/metadata/coarsen_container.hpp>

#include "partitioned_mix_view_fwd.hpp"

#include <stapl/domains/interval.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>

namespace stapl {

namespace detail {

template<bool Multidim, typename View, typename Part, typename CC>
struct select_partitioned_mix_view_base
{
  typedef array_ro_view<
    typename select_parameter<
      CC, view_coarsen_impl::coarsen_container<View, Part>
    >::type
  > type;
};

template<typename View, typename Part, typename CC>
struct select_partitioned_mix_view_base<true, View, Part, CC>
{
  typedef multiarray_view<
    typename select_parameter<
      CC, view_coarsen_impl::coarsen_container<View, Part>
    >::type
  > type;
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for view_traits to expose the types provided
///        for partitioned_mix_view.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Part, typename CC>
struct view_traits<partitioned_mix_view<View, Part, CC> >
{
  typedef typename detail::select_partitioned_mix_view_base<
    metadata_traits<Part>::is_isomorphic::value &&
    dimension_traits<Part>::type::value != 1, View, Part, CC
  >::type base_type;

  typedef typename base_type::view_container_type            container;
  typedef typename container::value_type                     value_type;
  typedef typename base_type::map_function                   map_function;
  typedef typename base_type::domain_type                    domain_type;
  typedef typename map_function::index_type                  index_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Represents a view over the partitioned distributed locality
///        metadata used to specify the data used in the tasks.
///
/// @tparam View Type of the original view used to extract the
///              locality metadata.
/// @tparam Part Partition generated during the metadata extraction.
/// @tparam CC Container used for this view (default is a
///            coarsen_container).
///
/// @todo partitioned_mix_view is currently excluded from coarsening,
///       i.e. the coarsener reflects it back unchanged. The correct
///       result should be a
///       partitioned_mix_view<partitioned_mix_view<View, ...>, ...>
///       (see also GFORGE #1500).
//////////////////////////////////////////////////////////////////////
template <typename View, typename Part, typename CC>
class partitioned_mix_view
  : public detail::select_partitioned_mix_view_base<
      metadata_traits<Part>::is_isomorphic::value &&
      dimension_traits<Part>::type::value != 1,
      View, Part, CC
    >::type,
    public view_operations::paragraph_required_operation<
      partitioned_mix_view<View, Part, CC>
    >
{
  typedef typename select_parameter<
    CC, view_coarsen_impl::coarsen_container<View,Part>
    >::type                                              container_t;

  typedef typename detail::select_partitioned_mix_view_base<
      metadata_traits<Part>::is_isomorphic::value &&
      dimension_traits<Part>::type::value != 1,
      View, Part, CC
    >::type base_type;

public:
  typedef container_t                                    view_container_type;
  typedef typename container_t::value_type               value_type;
  typedef typename base_type::domain_type                domain_type;
  typedef typename base_type::map_func_type              map_func_type;

  typedef value_type                                     subview_type;
  typedef value_type                                     reference;

  typedef typename domain_type::size_type                size_type;
  typedef typename domain_type::index_type               index_type;
  typedef typename map_func_type::gid_type               gid_type;

  typedef typename dimension_traits<View>::type          dimension_type;

  using cid_type = index_type;

  typedef partitioned_mix_view                           fast_view_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Functor to extract locality metadata from the partitioned
  ///   mix view.
  ///
  /// Returns the partition information stored by the underlying
  /// metadata container.
  ///
  /// @todo Returning the pointer to the partition stored directly in
  ///   the metadata container causes runtime error on freeing an invalid
  ///   pointer.
  //////////////////////////////////////////////////////////////////////
  struct loc_dist_metadata
  {
    using return_type = Part;

    return_type* operator()(partitioned_mix_view const* view)
    {
      return new Part(view->get_container()->get_partition());
    }
  };

public:

  partitioned_mix_view(partitioned_mix_view const& other)
    : base_type(other)
  { }

  template<typename Cont>
  partitioned_mix_view(Cont* c)
    : base_type(c, domain_type(c->dimensions()))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the location associated with the elements indexed
  ///        with the given @c index.
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(index_type const& index) const
  {
    return this->get_container()->get_location_element(index);
  }
}; // class partitioned_mix_view

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PARTITIONED_MIX_VIEW_HPP
