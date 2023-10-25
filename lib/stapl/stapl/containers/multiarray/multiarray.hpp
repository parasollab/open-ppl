/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIARRAY_HPP
#define STAPL_CONTAINERS_MULTIARRAY_HPP

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/multiarray/multiarray_traits.hpp>
#include <stapl/containers/partitions/block_partition.hpp>
#include <stapl/containers/multiarray/multiarray_metadata.hpp>

#include <boost/mpl/int.hpp>

#include <stapl/containers/mapping/multidimensional_mapper.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/containers/partitions/block_partition.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/type_traits/is_view.hpp>
#include <stapl/containers/distribution/is_distribution_view.hpp>

#include "multiarray_fwd.hpp"
#include "proxy.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for multiarray.
/// @ingroup pmultiarrayTraits
/// @see multiarray container_traits
//////////////////////////////////////////////////////////////////////
template<int N, typename T, typename ...OptionalParams>
struct container_traits<multiarray<N, T, OptionalParams...>>
  : public multiarray_impl::compute_multiarray_traits<
      N, T, OptionalParams...
    >::type
{
  typedef void enable_view_reference;

  template <typename C>
  struct construct_view
  {
    typedef multiarray_view<C> type;
  };
};


template<int N, typename T, typename ...OptionalParams>
class multiarray
  : public container<multiarray<N, T, OptionalParams...>>
{
private:
  typedef container_traits<multiarray>                      traits_type;
  typedef container<multiarray>                             base_type;

public:
  /// @brief The traversal ordering of the multiarray.
  /// @see default_traversal.
  STAPL_IMPORT_DTYPE(traits_type, traversal_type)

  STAPL_IMPORT_DTYPE(base_type, value_type)
  STAPL_IMPORT_DTYPE(base_type, distribution_type)
  STAPL_IMPORT_DTYPE(base_type, partition_type)
  STAPL_IMPORT_DTYPE(base_type, mapper_type)

  typedef typename partition_type::value_type               domain_type;

  STAPL_IMPORT_DTYPE(domain_type, index_type)

  typedef index_type                                        gid_type;
  typedef index_type                                        dimensions_type;

  typedef typename distribution_type::reference             reference;

  typedef metadata::multiarray_extractor<distribution_type> loc_dist_metadata;

  /// Compile time constant of the number of dimensions
  typedef std::integral_constant<int, N>                    dimension_type;

private:
  /// Domain used by the partition
  typedef typename base_type::partition_type::value_type    part_dom_t;

public:
  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Default constructor creates an empty multiarray with zero
  ///   size in all dimensions.
  ////////////////////////////////////////////////////////////////////////
  multiarray(void)
    : base_type(
       partition_type(
         part_dom_t(
           tuple_ops::transform(dimensions_type(), [](size_t const&)
             { return 0; })),
         multiarray_impl::make_multiarray_size<N>()(get_num_locations())),
       mapper_type(partition_type(
         part_dom_t(
           tuple_ops::transform(dimensions_type(), [](size_t const&)
             { return 0; })),
         multiarray_impl::make_multiarray_size<N>()(get_num_locations()))))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a multiarray with a given size and
  /// default construct all elements.
  /// @param sizes The size of the multiarray in each dimension
  ////////////////////////////////////////////////////////////////////////
  multiarray(dimensions_type const& sizes)
   : base_type(
       partition_type(
         part_dom_t(sizes),
         multiarray_impl::make_multiarray_size<N>()(get_num_locations())),
       mapper_type(partition_type(
         part_dom_t(sizes),
         multiarray_impl::make_multiarray_size<N>()(get_num_locations()))))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a multiarray with given sizes in each dimension
  /// and a default value.
  /// @param sizes The size of the multiarray in each dimension
  /// @param default_value Initial value of the elements in the container.
  ////////////////////////////////////////////////////////////////////////
  template <typename ValueType>
  multiarray(dimensions_type const& sizes, ValueType const& default_value)
    : base_type(
        partition_type(
          part_dom_t(sizes),
          multiarray_impl::make_multiarray_size<N>()(get_num_locations())),
        mapper_type(partition_type(
          part_dom_t(sizes),
          multiarray_impl::make_multiarray_size<N>()(get_num_locations()))),
        default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a multiarray with a given sizes in each dimension
  /// and a mapper.
  /// @param sizes The size of the multiarray in each dimension
  /// @param mapper An instance of the mapper for distribution.
  ////////////////////////////////////////////////////////////////////////
  multiarray(dimensions_type const& sizes, mapper_type const& mapper)
    : base_type(
        partition_type(
          part_dom_t(sizes),
          multiarray_impl::make_multiarray_size<N>()(get_num_locations())),
        mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a multiarray with a given partition
  /// @param ps An instance of a partition to use for distribution
  ////////////////////////////////////////////////////////////////////////
  multiarray(partition_type const& ps)
    : base_type(ps, mapper_type(ps))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a multiarray with a given partitioner and mapper,
  /// default constructing all elements.
  /// @param partitioner The partition for the container.
  /// @param mapper An instance of the mapper for distribution.
  ////////////////////////////////////////////////////////////////////////
  multiarray(partition_type const& partitioner, mapper_type const& mapper)
    : base_type(partitioner, mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a multiarray with the distribution specified by
  /// @p dist_view.
  //////////////////////////////////////////////////////////////////////
  template<typename DistSpecsView>
  multiarray(DistSpecsView const& dist_view,
             typename std::enable_if<
               is_distribution_view<DistSpecsView>::value
               && !detail::has_is_composed_dist_spec<DistSpecsView>::value
               && is_view_based<partition_type>::value
               && is_view_based<mapper_type>::value
             >::type* = 0)
    : base_type(std::shared_ptr<DistSpecsView>(new DistSpecsView(dist_view)))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a multiarray with the distribution specified by
  /// @p dist_view. All elements of the container will be initialized to
  /// @p default_value.
  //////////////////////////////////////////////////////////////////////
  template<typename DistSpecsView>
  multiarray(DistSpecsView const& dist_view, value_type const& default_value,
             typename std::enable_if<
               is_distribution_view<DistSpecsView>::value
               && !detail::has_is_composed_dist_spec<DistSpecsView>::value
               && is_view_based<partition_type>::value
               && is_view_based<mapper_type>::value
             >::type* = 0)
    : base_type(std::shared_ptr<DistSpecsView>(new DistSpecsView(dist_view)),
                default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  ///   container, @p sizes_view is an m-1 level composed view representing
  ///   the sizes of the nested containers.
  //////////////////////////////////////////////////////////////////////
  template<typename SizesView>
  multiarray(SizesView const& sizes_view,
              typename std::enable_if<
                  std::is_same<dimensions_type,
                               typename SizesView::dimensions_type>::value
                  && !is_distribution_view<SizesView>::value
                  && !detail::has_is_composed_dist_spec<SizesView>::value
                  && !is_view_based<partition_type>::value
                  && !is_view_based<mapper_type>::value
              >::type* = 0)
    : base_type(
        partition_type(
          part_dom_t(sizes_view.dimensions()),
          multiarray_impl::make_multiarray_size<N>()(get_num_locations())),
        mapper_type(partition_type(
         part_dom_t(sizes_view.dimensions()),
         multiarray_impl::make_multiarray_size<N>()(get_num_locations()))),
        sizes_view)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers. For an m level composed
  /// container, @p comp_spec contains specifications of the distributions
  /// of the current container and each of its elements.
  ///
  /// The specification of the current container's distribution is accessed
  /// by calling the @p spec() method, while the distribution specification
  /// of an element is accessed via @p operator[].
  ///
  /// @param comp_spec Instance of @ref composed_dist_spec representing the
  /// distribution specifications for each nested container.
  //////////////////////////////////////////////////////////////////////
  template<typename ComposedSpec>
  multiarray(ComposedSpec const& comp_spec,
             typename std::enable_if<
               detail::has_is_composed_dist_spec<ComposedSpec>::value &&
               is_view_based<partition_type>::value &&
               is_view_based<mapper_type>::value
             >::type* = 0)
    : base_type(
        std::make_shared<typename ComposedSpec::distribution_spec>(
          comp_spec.spec()),
        comp_spec)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers. For an m level composed
  /// container, @p comp_spec contains specifications of the distributions
  /// of the current container and each of its elements. For cases which the
  /// value type(the nested container) needs more arguments than the size of
  /// the container in its constructor, a factory is passed for the creation
  /// of nested elements.
  ///
  /// The specification of the current container's distribution is accessed
  /// by calling the @p spec() method, while the distribution specification
  /// of an element is accessed via @p operator[].
  ///
  /// @param comp_spec Instance of @ref composed_dist_spec representing the
  /// distribution specifications for each nested container.
  /// @param factory_value_type Boost factory which is used for creation of
  /// elements of the @c multiarray.
  //////////////////////////////////////////////////////////////////////
  template <typename ComposedSpec, typename FactoryValueType>
  multiarray(ComposedSpec const& comp_spec,
             FactoryValueType&& factory_value_type,
             typename std::enable_if<
               detail::has_is_composed_dist_spec<ComposedSpec>::value &&
               is_view_based<partition_type>::value &&
               is_view_based<mapper_type>::value
             >::type* = 0)
    : base_type(
        std::make_shared<typename ComposedSpec::distribution_spec>(
          comp_spec.spec()),
        comp_spec,
        std::forward<FactoryValueType>(factory_value_type))
  { }

  typedef nd_linearize<gid_type, traversal_type>     linearization_type;
  linearization_type                                 m_linear_mf;

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the type of a deep slice on this view
  ///        container.
  ///
  /// @tparam Slices The indices to slice on this deep slice
  /// @tparam Fixed The type of element that will be used to specify the
  ///               fixed values. Typically a tuple of size |Slices|.
  //////////////////////////////////////////////////////////////////////
  template<typename Slices, typename Fixed>
  struct slice_type
  {
    using type =
      typename distribution_type::base_container_type::template slice_type<
        Slices, Fixed
      >::type;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a deep slice by slicing off dimensions specified
  ///        in Slices.
  ///
  /// @note This currently just aborts for a pContainer, but could potentially
  ///       return a shallow slice as a SLICED_view.
  //////////////////////////////////////////////////////////////////////
  template<typename Slices, typename Fixed>
  auto slice(Fixed const& fixed)
    -> decltype(this->distribution().container_manager().begin()
                  ->template slice<Slices>(fixed))
  {
    abort("Deep slice called for stapl::multiarray");
    return this->distribution().container_manager().begin()->
      template slice<Slices>(fixed);
  }

  /// @}

  /// @name Element Manipulation
  /// @{
  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to a specific index of the multiarray.
  /// @param gid The components of the index to be referenced
  /// @return A proxy of the value at the index
  ////////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    return this->distribution().make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to a specific index of the multiarray.
  /// @param gid The index (tuple) for which to create the reference
  /// @return A proxy of the value at the index
  ////////////////////////////////////////////////////////////////////////
  reference operator[](gid_type const& gid)
  {
    return this->distribution().make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to a specific index of the multiarray.
  /// @param i The components of the index to be referenced
  /// @return A proxy of the value at the index
  ////////////////////////////////////////////////////////////////////////
  template <typename... Indices>
  reference operator()(Indices const&... i)
  {
    return this->distribution().make_reference(i...);
  }

  /// @}

  /// @name Memory and Domain Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the size of the multiarray in each dimension
  //////////////////////////////////////////////////////////////////////
  dimensions_type dimensions() const
  {
    return this->domain().dimensions();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroy the distribution of the container (including all of
  ///   its elements) and recreate the container with a different size
  /// @param n The new size of the multiarray dimensions.
  /// @bug This method destroys all internal state of the original partition
  ///   and mapper. Ideally, these should be copied over and reused.
  /// @todo Generalize to allow resizing of non-empty multiarrays.
  ////////////////////////////////////////////////////////////////////////
  void resize(dimensions_type n)
  {
    const dimensions_type old_size = this->dimensions();

    stapl_assert(vs_map_reduce([](size_t const& dim) { return dim == 0; },
                               std::logical_and<bool>(),true, old_size)
                 || this->domain().empty(),
                 "multiarray::resize called on non-empty container");

    const bool sizes_equal =
      vs_map_reduce(std::equal_to<size_t>(),
                    std::logical_and<bool>(),
                    true, old_size, n);

    if (sizes_equal)
      return;

    this->incr_version();

    boost::optional<gang> g;

    if (this->get_num_locations() == 1)
      g = boost::in_place<gang>(this->distribution());

    partition_type new_part(
      part_dom_t(n),
      multiarray_impl::make_multiarray_size<N>()(this->get_num_locations())
    );

    distribution_type new_dist(new_part, mapper_type(new_part));

    rmi_fence();

    this->distribution() = new_dist;

    this->advance_epoch();
  }
  /// @}
}; // class multiarray

}// namespace stapl

#endif
