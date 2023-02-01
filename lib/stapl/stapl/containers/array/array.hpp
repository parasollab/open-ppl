/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_ARRAY_HPP
#define STAPL_CONTAINERS_ARRAY_HPP

#include <type_traits>

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/distribution/static_metadata.hpp>
#include <stapl/containers/distribution/composed_specification.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>
#include <stapl/containers/array/array_traits.hpp>

#include <stapl/domains/indexed.hpp>

#include <stapl/algorithms/algorithm.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/containers/distribution/is_distribution_view.hpp>

#include "array_fwd.hpp"

#include "proxy.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for array.
/// @ingroup parrayTraits
/// @see array container_traite
/// @todo enable_view_reference typedef is temporary until all containers
///   are updated to reflect views for nested container references.
//////////////////////////////////////////////////////////////////////
template<typename T, typename ...OptionalParams>
struct container_traits<array<T, OptionalParams...>>
  : public array_impl::compute_array_traits<T, OptionalParams...>::type
{
  typedef void enable_view_reference;

  template <typename C>
  struct construct_view
  {
    typedef array_view<C> type;
  };
};


template<typename T, typename ...OptionalParams>
class array
  : public container<array<T, OptionalParams...>>
{
public:
  STAPL_IMPORT_DTYPE(container_traits<array>, partition_type)
  STAPL_IMPORT_DTYPE(container_traits<array>, mapper_type)
  STAPL_IMPORT_DTYPE(container_traits<array>, domain_type)

private:
  typedef container<array>                                  base_type;

public:
  typedef T                                                 value_type;

  STAPL_IMPORT_DTYPE(domain_type, index_type)
  STAPL_IMPORT_DTYPE(domain_type, size_type)

  typedef index_type                                        gid_type;

  STAPL_IMPORT_DTYPE(base_type, distribution_type)

  STAPL_IMPORT_DTYPE(distribution_type, reference)
  STAPL_IMPORT_DTYPE(distribution_type, const_reference)
  STAPL_IMPORT_DTYPE(distribution_type, iterator)
  STAPL_IMPORT_DTYPE(distribution_type, const_iterator)

  /// Metadata extractor type used for coarsening
  typedef metadata::static_container_extractor<
    distribution_type
  >                                                         loc_dist_metadata;

  /// Domain used by the mapper
  typedef typename mapper_type::domain_type                 map_dom_t;
  /// Type of the subdomains produced by the partition
  typedef typename partition_type::value_type               part_dom_t;

protected:
  /// @brief Domain of the array
  domain_type m_domain;

public:
  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with size 0. Initially
  /// places the array in an unusable state. Should be used in conjunction
  /// with @ref array::resize
  ////////////////////////////////////////////////////////////////////////
  array(void)
    : base_type(partition_type(part_dom_t(), get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a given size and
  /// default construct all elements.
  /// @param n The size of the array
  ////////////////////////////////////////////////////////////////////////
  array(size_type n)
    : base_type(partition_type(part_dom_t(n), get_num_locations()),
                mapper_type(map_dom_t(get_num_locations()))),
      m_domain(n)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a given size and
  /// construct all elements with a default value.
  /// @param n The size of the array
  /// @param default_value The initial value of the elements in the container
  ////////////////////////////////////////////////////////////////////////
  array(size_type n, value_type const& default_value)
    : base_type(partition_type(part_dom_t(n), get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())),
                default_value),
      m_domain(n)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a given size and instance of mapper
  /// @param n The size of the array
  /// @param mapper An instance of a mapper to use for distribution
  ////////////////////////////////////////////////////////////////////////
  array(size_type n, mapper_type const& mapper)
    : base_type(partition_type(part_dom_t(n), get_num_locations()), mapper),
      m_domain(n)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a given instance of partition
  /// @param ps An instance of a partition to use for distribution
  ////////////////////////////////////////////////////////////////////////
  array(partition_type const& ps)
    : base_type(ps, mapper_type(map_dom_t(ps.size()))),
      m_domain(ps.global_domain().size())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a given instance of partition and
  ///        construct all elements with a default value.
  /// @param ps An instance of a partition to use for distribution
  /// @param default_value  The initial value of the elements in the container
  ////////////////////////////////////////////////////////////////////////
  array(partition_type const& ps, value_type const& default_value)
    : base_type(ps, mapper_type(map_dom_t(ps.size())), default_value),
      m_domain(ps.global_domain().size())
  { }

  ////////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a given partitioner and a mapper.
  /// @param partitioner An instance of the partitioner to use for the
  ///        distribution.
  /// @param mapper An instance of the mapper to use for the distribution.
  ///////////////////////////////////////////////////////////////////////
  array(partition_type const& partitioner, mapper_type const& mapper)
    : base_type(partitioner, mapper),
      m_domain(partitioner.global_domain().size())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a distribution that is specified by
  /// the @p dist_view provided.
  /// @param dist_view view that specifies the mapping of container GIDs
  /// to partition ids, and partition ids to location ids.
  ///
  /// @ref distribution_specs contains a set of functions to create views
  /// for common data distributions.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  array(DistSpecView const& dist_view,
        typename std::enable_if<
          is_distribution_view<DistSpecView>::value &&
          !detail::has_is_composed_dist_spec<DistSpecView>::value
        >::type* = 0)
    : base_type(std::make_shared<DistSpecView>(dist_view)),
      m_domain(dist_view.domain().size())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a distribution that is specified by
  /// the @p dist_view provided, and initialize all elements to
  /// @p default_value.
  /// @param dist_view view that specifies the mapping of container GIDs
  /// to partition ids, and partition ids to location ids.
  /// @param default_value value to assign to each element in the container.
  ///
  /// @ref distribution_specs contains a set of functions to create views
  /// for common data distributions.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  array(DistSpecView const& dist_view,
        value_type const& default_value,
        typename std::enable_if<
            is_distribution_view<DistSpecView>::value &&
            !detail::has_is_composed_dist_spec<DistSpecView>::value
          >::type* = 0)

    : base_type(std::make_shared<DistSpecView>(dist_view), default_value),
      m_domain(dist_view.domain().size())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with an arbitrary distribution that is specified
  /// by the elements of the @p part_cont provided, which are instances of
  /// @ref arbitrary_partition_info.
  /// @param part_cont container of elements that specify the arbitrary
  /// mapping of gids to partition ids and location ids.
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionContainer>
  array(PartitionContainer const& part_cont,
        typename std::enable_if<
          std::is_same<typename PartitionContainer::value_type,
            arbitrary_partition_info>::value>::type* = 0)
    : base_type(&part_cont),
      m_domain(this->distribution().domain().size())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with an arbitrary distribution that is specified
  /// by the elements of the @p part_cont provided, which are instances of
  /// @ref arbitrary_partition_info. All elements are initialized to
  /// @p default_value.
  /// @param part_cont container of elements that specify the arbitrary
  /// mapping of gids to partition ids and location ids.
  /// @param default_value value to assign to each element in the container.
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionContainer>
  array(PartitionContainer const& part_cont,
        value_type const& default_value,
        typename std::enable_if<
          std::is_same<typename PartitionContainer::value_type,
            arbitrary_partition_info>::value>::type* = 0)
    : base_type(&part_cont, default_value),
      m_domain(this->distribution().domain().size())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a given size and default
  /// value where the value type of the container is itself a parallel
  /// container.
  /// @param n The size of the array
  /// @param default_value The initial value of the elements in the container
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  ////////////////////////////////////////////////////////////////////////
  template <typename DP>
  array(size_type n, value_type const& default_value, DP const& dis_policy)
    : base_type(partition_type(part_dom_t(n), get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())),
                default_value, dis_policy),
      m_domain(n)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array of arrays with given
  /// n-dimensional size.
  /// @param dims A cons list specifying the dimensions of the containers.
  ////////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  array(boost::tuples::cons<X,Y> dims)
    : base_type(partition_type(part_dom_t(dims.get_head()),
                               get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())), dims),
      m_domain(dims.get_head())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array of arrays with given
  /// n-dimensional size.
  /// @param dims A cons list specifying the dimensions of the containers.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers.
  ////////////////////////////////////////////////////////////////////////
  template <typename X, typename Y, typename DP>
  array(boost::tuples::cons<X,Y> dims, DP const& dis_policy)
    : base_type(partition_type(
                  part_dom_t(dims.get_head()),
                  get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())), dims,
                dis_policy),
      m_domain(dims.get_head())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  ///   container, @p sizes_view is an m-1 level composed view representing
  ///   the sizes of the nested containers.
  //////////////////////////////////////////////////////////////////////
  template<typename SizesView>
  array(SizesView const& sizes_view,
        typename std::enable_if<
          std::is_same<size_type, typename SizesView::size_type>::value
          && !is_distribution_view<SizesView>::value
          && !std::is_same<typename SizesView::value_type,
                arbitrary_partition_info>::value
          && !detail::has_is_composed_dist_spec<SizesView>::value>::type* = 0)
    : base_type(
        partition_type(part_dom_t(sizes_view.size()), get_num_locations()),
        mapper_type(map_dom_t(get_num_locations())),
        sizes_view),
      m_domain(sizes_view.size())
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
  array(ComposedSpec const& comp_spec,
        typename std::enable_if<
          detail::has_is_composed_dist_spec<ComposedSpec>::value>::type* = 0)
    : base_type(
        std::make_shared<typename ComposedSpec::distribution_spec>(
          comp_spec.spec()),
        comp_spec),
      m_domain(comp_spec.size())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  /// container, @p dist_specs contains specifications of the distributions
  /// to be used at each level of the composed container.
  ///
  /// The first element of the vector specifies the distribution of the outer
  /// container, the second the distribution of the containers at the first
  /// level of composition, etc.  The number of elements in @p dist_specs must
  /// be at least the same as the number of levels of container composition.
  ///
  /// The result of the constructor is a container composition where the size
  /// and distribution of the container elements at a given level of the
  /// composition are the same.
  ///
  /// @param dist_specs distribution specifications that are used to construct
  /// the nested containers at a given level of the composition.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  array(std::vector<DistSpecView> const& dist_specs,
        typename std::enable_if<
          is_distribution_view<DistSpecView>::value &&
          !detail::has_is_composed_dist_spec<DistSpecView>::value>::type* = 0)
    : base_type(
        std::make_shared<DistSpecView>(dist_specs[0]),
        make_composed_dist_spec(
          [&](std::vector<size_t> const& index) {
            stapl_assert(index.size() < dist_specs.size(),
              "dimensionality of index exceeds depth of distribution specs");
            return dist_specs[index.size()];
          })),
      m_domain(dist_specs[0].size())
  { }

  /// @}
  /// @name Element Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to a specific index of the array.
  /// @param idx The index for which to create the reference
  /// @return A proxy of the value at idx
  ////////////////////////////////////////////////////////////////////////
  reference operator[](index_type idx)
  {
    return this->distribution().operator[](idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_reference to a specific index of the array.
  /// @param idx The index for which to create the reference
  /// @return A proxy of the value at idx
  ////////////////////////////////////////////////////////////////////////
  const_reference operator[](index_type idx) const
  {
    return this->distribution().operator[](idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the first element of the array.
  /// @return A proxy of the value of the element at the first index.
  ////////////////////////////////////////////////////////////////////////
  reference front()
  {
    return this->distribution().operator[](this->domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_reference to the first element of the array.
  /// @return A proxy of the value of the element at the first index.
  ////////////////////////////////////////////////////////////////////////
  const_reference front() const
  {
    return this->distribution().operator[](this->domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the last element of the array.
  /// @return A proxy of the value of the element at the last index.
  ////////////////////////////////////////////////////////////////////////
  reference back()
  {
    return this->distribution().operator[](this->domain().last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_reference to the last element of the array.
  /// @return A proxy of the value of the element at the last index.
  ////////////////////////////////////////////////////////////////////////
  const_reference back() const
  {
    return this->distribution().operator[](this->domain().last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to a specific index of the array.
  /// @param idx The index for which to create the reference
  /// @return A proxy of the value at idx
  ////////////////////////////////////////////////////////////////////////
  reference make_reference(index_type const& idx)
  {
    return this->distribution().make_reference(idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_reference to a specific index of the array.
  /// @param idx The index for which to create the reference
  /// @return A proxy of the value at idx
  ////////////////////////////////////////////////////////////////////////
  const_reference make_reference(index_type const& idx) const
  {
    return this->distribution().make_reference(idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to a specific index of the array.
  /// @param gid The index for which to create the iterator
  /// @return An iterator to the value at gid
  ////////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return this->distribution().make_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to a specific index of the array.
  /// @param gid The index for which to create the const_iterator
  /// @return A const_iterator to the value at gid
  ////////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(gid_type const& gid) const
  {
    return this->distribution().make_const_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to the beginning of the array.
  /// @return A global iterator to GID 0
  ////////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to the beginning of the array.
  /// @return A global const_iterator to GID 0
  ////////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to the beginning of the array.
  /// @return A global const_iterator to GID 0
  ////////////////////////////////////////////////////////////////////////
  const_iterator cbegin(void) const
  {
    return this->distribution().cbegin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to one past the end of the array.
  /// @return A global iterator of the end
  ////////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return this->distribution().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to one past the end of the array.
  /// @return A global iterator of the end
  ////////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return this->distribution().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to one past the end of the array.
  /// @return A global iterator of the end
  ////////////////////////////////////////////////////////////////////////
  const_iterator cend(void) const
  {
    return this->distribution().cend();
  }

  /// @}

  /// @name Memory and Domain Management
  /// @{

  domain_type domain(void) const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroy the distribution of the container (including all of
  /// its elements) and recreate the container with a different size
  /// @param n The new size of the array.
  /// @bug This method destroys all internal state of the original partition
  /// and mapper. Ideally, these may want to be copied over.
  ////////////////////////////////////////////////////////////////////////
  void resize(size_type n)
  {
    if (n == this->size())
      return;

    this->incr_version();

    boost::optional<gang> g;

    if (this->get_num_locations() == 1)
      g = boost::in_place<gang>(this->distribution());

    array temp = *this;
    array_view<array> tempv(temp);

    this->distribution() =
      distribution_type(
       partition_type(part_dom_t(n), get_num_locations()),
       mapper_type(map_dom_t(get_num_locations())));

    m_domain = domain_type(n);

    array_view<array> thisv(*this);

    const size_t size_to_copy = tempv.size() < n ? tempv.size() : n;

    // explicit specification due to identical signature of std::copy_n.
    stapl::copy_n(tempv, thisv, size_to_copy);
  }
  /// @}
}; // class array

} // namespace stapl

#endif // STAPL_CONTAINERS_ARRAY_HPP

