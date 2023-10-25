/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_VECTOR_HPP
#define STAPL_CONTAINERS_VECTOR_HPP

#include <type_traits>

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/distribution/composed_specification.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>
#include <stapl/containers/distribution/is_distribution_view.hpp>

#include <stapl/containers/vector/traits/vector_traits.hpp>

#include <stapl/domains/indexed.hpp>

#include <stapl/algorithms/algorithm.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/views/vector_view.hpp>

#include "vector_fwd.hpp"
#include "proxy.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for vector.
/// @ingroup pvectorTraits
///
/// @tparam T Type of the stored elements in the container.
/// @tparam PS Partition strategy that defines how to partition
/// the original domain into subdomains.
/// @tparam M Mapper that defines how to map the subdomains produced
/// by the partition to locations.
/// @tparam Traits A traits class that defines customizable components
/// of the vector container.
/// @todo enable_view_reference typedef is temporary until all containers
///   are updated to reflect views for nested container references.
////////////////////////////////////////////////////////////////////////
template<typename T, typename ...OptionalParams>
struct container_traits<vector<T, OptionalParams...>>
  : public vector_impl::compute_vector_traits<T, OptionalParams...>::type
{
  typedef void enable_view_reference;

  template <typename C>
  struct construct_view
  {
    typedef vector_view<C> type;
  };
};


template<typename T, typename ...OptionalParams>
class vector
  : public container<vector<T, OptionalParams...>>
{
private:
  typedef container_traits<vector>                     traits_type;

public:
  STAPL_IMPORT_TYPE(typename traits_type, partition_type)
  STAPL_IMPORT_TYPE(typename traits_type, mapper_type)

private:
  typedef container<vector>                             base_type;

public:
  typedef typename base_type::distribution_type         distribution_type;
  typedef typename traits_type::manager_type            manager_type;

  typedef T                                             value_type;
  typedef typename traits_type::domain_type             domain_type;
  typedef typename domain_type::index_type              gid_type;
  typedef gid_type                                      index_type;
  typedef size_t                                        size_type;

  /// Domain used by the mapper
  typedef typename mapper_type::domain_type             map_dom_t;
  typedef typename distribution_type::const_iterator    const_iterator;

  STAPL_IMPORT_TYPE(typename distribution_type, directory_type)
  STAPL_IMPORT_TYPE(typename distribution_type, container_manager_type)
  STAPL_IMPORT_TYPE(typename distribution_type, reference)
  STAPL_IMPORT_TYPE(typename distribution_type, iterator)
  /// Distribution metadata type used for coarsening
  STAPL_IMPORT_TYPE(typename distribution_type, loc_dist_metadata)

  /// @name Constructors
  /// @{

  vector(void)
    : base_type(partition_type(domain_type(0), get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a vector with of size @p n.
  /// @param n Size of the vector.
  //////////////////////////////////////////////////////////////////////
  vector(size_t n)
    : base_type(partition_type(domain_type(n),get_num_locations()),
                mapper_type(map_dom_t(get_num_locations()) ) )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a vector with of size @p n filled
  ///        with value @p default_value.
  /// @param n Size of the vector.
  /// @param default_value Value given at initialization.
  //////////////////////////////////////////////////////////////////////
  vector(size_t n, value_type const& default_value)
    : base_type(partition_type(domain_type(n), get_num_locations()),
          mapper_type(map_dom_t(get_num_locations()) ),
          default_value )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a vector given a partitioner.
  /// @param ps An instance of the partitioner to use for the distribution.
  //////////////////////////////////////////////////////////////////////
  vector(partition_type const& ps)
    : base_type(ps, mapper_type(map_dom_t(ps.size())) )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a given instance of partition and
  ///        construct all elements with a default value.
  /// @param ps An instance of a partition to use for distribution
  /// @param default_value  The initial value of the elements in the container
  ////////////////////////////////////////////////////////////////////////
  vector(partition_type const& ps, value_type const& default_value)
    : base_type(ps, mapper_type(map_dom_t(ps.size())), default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a vector given a mapper.
  /// @param n Size of the vector.
  /// @param mapper An instance of the mapper to use for the distribution.
  //////////////////////////////////////////////////////////////////////
  vector(size_t n, mapper_type const& mapper)
    : base_type(partition_type(domain_type(n),
                               get_num_locations()),
                mapper_type(mapper) )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a vector given a partitioner and a mapper.
  /// @param partitioner An instance of the partitioner to use for
  ///        the distribution.
  /// @param mapper An instance of the mapper to use for the distribution.
  //////////////////////////////////////////////////////////////////////
  vector(partition_type const& partitioner, mapper_type const& mapper)
    : base_type(partitioner, mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a vector with a distribution that is specified by
  /// the @p dist_view provided.
  /// @param dist_view view that specifies the mapping of container GIDs
  /// to partition ids, and partition ids to location ids.
  ///
  /// @ref distribution_specs contains a set of functions to create views
  /// for common data distributions.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  vector(DistSpecView const& dist_view,
    typename std::enable_if<
      is_distribution_view<DistSpecView>::value &&
      !detail::has_is_composed_dist_spec<DistSpecView>::value
    >::type* =0)
    : base_type(std::shared_ptr<DistSpecView>(new DistSpecView(dist_view)))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a vector with a distribution that is specified by
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
  vector(DistSpecView const& dist_view, value_type const& default_value,
    typename std::enable_if<
      is_distribution_view<DistSpecView>::value &&
      !detail::has_is_composed_dist_spec<DistSpecView>::value
    >::type* =0)
    : base_type(std::shared_ptr<DistSpecView>(new DistSpecView(dist_view)),
        default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a vector with a given size and default
  ///        value where the value type of the container is itself a parallel
  ///        container.
  /// @param n The size of the vector.
  /// @param default_value The initial value of the elements in the container.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  ////////////////////////////////////////////////////////////////////////
  template <typename DP>
  vector(size_t n, value_type const& default_value, DP const& dis_policy)
    : base_type(partition_type(domain_type(n),
                               get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())),
                default_value, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a vector of vectors with given n-dimensional size.
  /// @param dims A cons list specifying the dimensions of the containers.
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  vector(boost::tuples::cons<X,Y> dims)
    : base_type(partition_type(domain_type(dims.get_head()),
                               get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())), dims)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a vector of vectors with given n-dimensional size.
  /// @param dims A cons list specifying the dimensions of the containers.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers.
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y, typename DP>
  vector(boost::tuples::cons<X,Y> dims, DP const& dis_policy)
    : base_type(partition_type(domain_type(dims.get_head()),
                               get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())),
                            dims, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for composed containers.  For an m level composed
  ///   container, @p sizes_view is an m-1 level composed view representing
  ///   the sizes of the nested containers.
  //////////////////////////////////////////////////////////////////////
  template<typename SizesView>
  vector(SizesView const& sizes_view,
         typename boost::enable_if<
           boost::mpl::and_<
             std::is_same<size_type, typename SizesView::size_type>,
             boost::mpl::not_<is_distribution_view<SizesView> >
           >
         >::type* = 0)
    : base_type(partition_type(domain_type(sizes_view.size()),
                               get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())),
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
  template <typename ComposedSpec>
  vector(ComposedSpec const& comp_spec,
         typename std::enable_if<
           detail::has_is_composed_dist_spec<ComposedSpec>::value>::type* = 0)
    : base_type(
        std::shared_ptr<typename ComposedSpec::distribution_spec>
          (new typename ComposedSpec::distribution_spec(comp_spec.spec())),
        comp_spec
      )
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
  vector(std::vector<DistSpecView> const& dist_specs,
         typename std::enable_if<
           is_distribution_view<DistSpecView>::value &&
           !detail::has_is_composed_dist_spec<DistSpecView>::value>::type* = 0)
    : base_type(
        std::shared_ptr<DistSpecView>(new DistSpecView(dist_specs[0])),
        make_composed_dist_spec(
          [&](std::vector<size_t> const& index) {
            stapl_assert(index.size() < dist_specs.size(),
              "dimensionality of index exceeds depth of distribution specs");
            return dist_specs[index.size()];
        })
      )
  { }

  /// @}
  /// @name Element Manipulation
  /// @{
protected:

  //////////////////////////////////////////////////////////////////////
  /// @brief Add an element @p val at the @p gid position.
  /// @param gid The index to insert the element.
  /// @param val The element to add.
  /// @param view Pointer to a @ref vector_view over this vector (used to
  ///   keep the domains of the container and the view in sync) or nullptr
  ///   if the vector is used directly.
  //////////////////////////////////////////////////////////////////////
  template<typename View>
  void insert_local(gid_type const& gid, value_type const& val, View* view)
  {
    this->distribution().insert(gid, val, view);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief remove an element from the container.
  /// @param gid the gid referencing the element to delete.
  /// @param view Pointer to a @ref vector_view over this vector (used to
  ///   keep the domains of the container and the view in sync) or nullptr
  ///   if the vector is used directly.
  //////////////////////////////////////////////////////////////////////
  template<typename View>
  void erase_local(gid_type const& gid, View* view)
  {
    this->distribution().erase(gid, view);
  }

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to a specific gid of the vector.
  /// @param gid The gid for which to create the reference.
  /// @return A proxy of the value at idx.
  ////////////////////////////////////////////////////////////////////////
  reference operator[](gid_type const& gid)
  {
    return this->distribution().operator[](gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the first element of the vector.
  /// @return A proxy of the value of the element returned.
  //////////////////////////////////////////////////////////////////////
  reference front(void)
  {
    return make_reference(this->distribution().domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the last element of the vector.
  /// @return A proxy of the value of the element returned.
  //////////////////////////////////////////////////////////////////////
  reference back(void)
  {
    return make_reference(this->distribution().domain().last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert locally a new element into the container.
  /// @param val The value to insert.
  //////////////////////////////////////////////////////////////////////
  void add(value_type const& val)
  {
    this->incr_version();
    this->distribution().local_push_back(val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add an element @p val at the @p gid position.
  /// @param gid The index to insert the element.
  /// @param val The element to add.
  /// @param view Pointer to a @ref vector_view over this vector. Should be
  ///   provided if this vector is used as the underlying container of a
  ///   @ref vector_view, so that the container can notify the view of any
  ///   change to the domain it makes.
  /// @todo Replace the return value with future_exist<bool> or similar.
  //////////////////////////////////////////////////////////////////////
  template<typename View = vector_view<vector>>
  void insert(gid_type const& gid, value_type const& val, View* view = nullptr)
  {
    stapl_assert(gid < this->size(), "Index out of bounds");

    this->incr_version();

    if (this->get_location_id() != 0)
      async_rmi(0, this->get_rmi_handle(),
        &vector::insert_local<View>, gid, val, view);
    else
      this->distribution().insert(gid, val, view);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove an element from the container.
  /// @param gid The gid referencing the element to delete.
  /// @param view Pointer to a @ref vector_view over this vector. Should be
  ///   provided if this vector is used as the underlying container of a
  ///   @ref vector_view, so that the container can notify the view of any
  ///   change to the domain it makes.
  //////////////////////////////////////////////////////////////////////
  template<typename View = vector_view<vector>>
  void erase(gid_type const& gid, View* view = nullptr)
  {
    this->incr_version();

    if (this->get_location_id() != 0)
      async_rmi(0, this->get_rmi_handle(),
        &vector::erase_local<View>, gid, view);
    else
      this->distribution().erase(gid, view);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove all elements from the container.
  ///
  /// @note This method must be called by all locations in the gang on
  ///       which the container is instantiated.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->incr_version();
    this->distribution().clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add an element at the end of the vector.
  /// @param v The element to add.
  /// @param view Pointer to a @ref vector_view over this vector. Should be
  ///   provided if this vector is used as the underlying container of a
  ///   @ref vector_view, so that the container can notify the view of any
  ///   change to the domain it makes.
  //////////////////////////////////////////////////////////////////////
  template<typename View = vector_view<vector>>
  void push_back(value_type const& v, View* view = nullptr)
  {
    this->incr_version();
    this->distribution().push_back(v, view);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the last element of the vector.
  /// @param view Pointer to a @ref vector_view over this vector. Should be
  ///   provided if this vector is used as the underlying container of a
  ///   @ref vector_view, so that the container can notify the view of any
  ///   change to the domain it makes.
  //////////////////////////////////////////////////////////////////////
  template<typename View = vector_view<vector>>
  void pop_back(View* view = nullptr)
  {
    this->incr_version();
    this->distribution().pop_back(view);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to the beginning of the vector.
  /// @return A global iterator.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return this->distribution().make_iterator(
                                this->distribution().domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to the beginning of the vector.
  /// @return A global const_iterator.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return this->distribution().make_const_iterator(
                                this->distribution().domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to the beginning of the vector.
  /// @return A global const_iterator.
  //////////////////////////////////////////////////////////////////////
  const_iterator cbegin(void) const
  {
    return this->distribution().make_const_iterator(
                                this->distribution().domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to the end of the vector.
  /// @return A global iterator of the end.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return this->distribution().make_iterator(
                                this->distribution().domain().last()+1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to the end of the vector.
  /// @return A global const_iterator of the end.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return this->distribution().make_const_iterator(
                                this->distribution().domain().last()+1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to the end of the vector.
  /// @return A global const_iterator of the end.
  //////////////////////////////////////////////////////////////////////
  const_iterator cend(void) const
  {
    return this->distribution().make_const_iterator(
                                this->distribution().domain().last()+1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to a specific index of the vector.
  /// @param index The index for which to create the iterator.
  /// @return An iterator to the value at index.
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(index_type const& index)
  {
    return this->distribution().make_iterator(index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to a specific index of the vector.
  /// @param index The index for which to create the const_iterator.
  /// @return A const_iterator to the value at index.
  //////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(index_type const& index) const
  {
    return this->distribution().make_const_iterator(index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to a specific index of the vector.
  /// @param index The index for which to create the reference.
  /// @return A proxy of the value at index.
  ////////////////////////////////////////////////////////////////////////
  reference make_reference(index_type const& index)
  {
    return this->distribution().make_reference(index);
  }

  /// @}

  /// @name Memory and Domain Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroy the distribution of the container (including all of
  /// its elements) and recreate the container with a different size
  /// @param n The new size of the vector.
  /// @bug This method destroys all internal state of the original partition
  /// and mapper. Ideally, these may want to be copied over.
  ////////////////////////////////////////////////////////////////////////
  void resize(size_type n)
  {
    typedef typename partition_type::value_type part_dom_t;

    if (n == this->size())
      return;

    this->incr_version();

    boost::optional<gang> g;

    if (this->get_num_locations() == 1)
      g = boost::in_place<gang>(this->distribution());

    vector temp = *this;
    array_view<vector> tempv(temp);

    partition_type new_part(part_dom_t(n), this->distribution().partition());

    distribution_type new_dist(
      new_part, mapper_type(map_dom_t(new_part.size()))
    );
    rmi_fence();

    this->distribution() = new_dist;

    array_view<vector> thisv(*this);

    const size_t size_to_copy = tempv.size() < n ? tempv.size() : n;

    // explicit specification due to identical signature of std::copy_n.
    stapl::copy_n(tempv, thisv, size_to_copy);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes out all pending requests to the vector
  ///
  /// This is used in conjunction with @ref add() to insert elements into
  /// the container without saturating the communication subsystem with
  /// metadata update requests.
  //////////////////////////////////////////////////////////////////////
  void flush(void)
  {
    this->distribution().synchronize_metadata();
  }

  /// @}
}; // class vector

} // namespace stapl

#endif // STAPL_CONTAINERS_VECTOR_HPP
