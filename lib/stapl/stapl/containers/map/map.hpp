/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MAP_HPP
#define STAPL_CONTAINERS_MAP_HPP

#include <boost/mpl/bool.hpp>

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/map/map_traits.hpp>
#include <stapl/containers/distribution/composed_specification.hpp>
#include <stapl/containers/distribution/is_distribution_view.hpp>
#include <stapl/containers/partitions/viewbased.hpp>

#include <stapl/views/map_view.hpp>

#include "proxy.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for @ref map.
/// @ingroup pmapTraits
/// @see map container_traits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename T, typename ...OptionalParams>
struct container_traits<map<Key, T, OptionalParams...>>
  : public map_impl::compute_map_traits<Key, T, OptionalParams...>::type
{
  typedef void enable_view_reference;

  template <typename C>
  struct construct_view
  {
    typedef map_view<C> type;
  };
};


template<typename Key, typename T, typename ...OptionalParams>
class map
  : public container<map<Key, T, OptionalParams...>>
{
private:
  typedef stapl::container_traits<map>                  traits_type;
  typedef container<map>                                base_type;

  STAPL_IMPORT_TYPE(typename traits_type, manager_type)
  STAPL_IMPORT_TYPE(typename traits_type, mapper_type)
  STAPL_IMPORT_TYPE(typename traits_type, partition_type)

  STAPL_IMPORT_TYPE(typename base_type, directory_type)
  STAPL_IMPORT_TYPE(typename base_type, container_manager_type)

  typedef typename
    base_type::distribution_type::cid_type              cid_type;

  typedef typename
    base_type::mapper_type::domain_type                 map_dom_t;

  composed_dist_spec*                                   m_comp_spec;

public:
  typedef Key                                           key_type;
  typedef T                                             mapped_type;
  typedef std::pair<const Key,T>                        value_type;
  typedef typename traits_type::compare_type            key_compare;
  typedef size_t                                        size_type;

  STAPL_IMPORT_TYPE(typename traits_type, gid_type)

  STAPL_IMPORT_TYPE(typename base_type, distribution_type)

  STAPL_IMPORT_TYPE(typename distribution_type, reference)
  STAPL_IMPORT_TYPE(typename distribution_type, second_reference)
  STAPL_IMPORT_TYPE(typename distribution_type, iterator)
  STAPL_IMPORT_TYPE(typename distribution_type, loc_dist_metadata)

  typedef iterator_domain<
    distribution_type, detail::get_first<Key>
  >                                                     domain_type;

  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a parallel map where the ownership of keys is
  ///   determined by a simple block-cyclic distribution of the key space.
  /// @todo The block size should be determined in a more intelligent way
  /// @todo The base container ordering approach in the associative
  ///   distribution is not implemented correctly. The ifdef'd version (which
  ///   should distribute keys in an unknown range across locations better)
  ///   relies more heavily on it and causes errors (see gforge tracker #1257).
  ///   Squash bug and then use the commented out strategy.
  //////////////////////////////////////////////////////////////////////
  map(void)
    : base_type(
        directory_type(
          partition_type(
            typename partition_type::value_type(
              std::numeric_limits<Key>::min(),
              std::numeric_limits<Key>::max()-1
            ),
            get_num_locations()
          ),
          mapper_type(map_dom_t(get_num_locations()))
        ),
        container_manager_type(
          partition_type(
            typename partition_type::value_type(
              std::numeric_limits<Key>::min(),
              std::numeric_limits<Key>::max()-1
            ),
            get_num_locations()
          ),
          mapper_type(map_dom_t(get_num_locations()))
        ), boost::mpl::false_()
      ),
      m_comp_spec(nullptr)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a parallel map where the ownership of keys is
  /// determined by a balanced partition of a given domain.
  ///
  /// @param domain The domain of keys that this map will allow
  ///
  /// @todo Get rid of free-standing get_num_locations calls
  //////////////////////////////////////////////////////////////////////
  map(typename partition_type::value_type const& domain)
    : base_type(
        directory_type(
          partition_type(domain, get_num_locations()),
          mapper_type(map_dom_t(get_num_locations()))
        ),
        container_manager_type(
          partition_type(domain, get_num_locations()),
          mapper_type(map_dom_t(get_num_locations()))
        ), boost::mpl::false_()
      ),
      m_comp_spec(nullptr)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a parallel map where the ownership of keys is
  /// determined by a given partition.
  ///
  /// @param partition Partition strategy for the GID space.
  //////////////////////////////////////////////////////////////////////
  map(partition_type const& partition)
    : base_type(
        directory_type(
          partition, mapper_type(partition.domain())
        ),
        container_manager_type(partition, mapper_type(partition.domain())),
        boost::mpl::false_()
      ),
      m_comp_spec(nullptr)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a parallel map where the ownership of keys is
  /// determined by a given partition and mapper.
  ///
  /// @param partitioner Partition strategy for the GID space.
  /// @param mapper Mapper to assign subdomains to locations
  //////////////////////////////////////////////////////////////////////
  map(partition_type const& partitioner, mapper_type const& mapper)
    : base_type(distribution_type(
        directory_type(partitioner, mapper),
        container_manager_type(partitioner, mapper), boost::mpl::false_()
      )),
      m_comp_spec(nullptr)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a map with a distribution that is specified by
  /// the @p dist_view provided.
  /// @param dist_view view that specifies the mapping of container GIDs
  /// to partition ids, and partition ids to location ids.
  ///
  /// @ref distribution_spec contains a set of functions to create views
  /// for common data distributions.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  map(DistSpecView const& dist_view,
      typename std::enable_if<
        is_distribution_view<DistSpecView>::value &&
        !detail::has_is_composed_dist_spec<DistSpecView>::value
      >::type* = 0)
    : base_type(std::shared_ptr<DistSpecView>(new DistSpecView(dist_view)),
                boost::mpl::false_()),
      m_comp_spec(nullptr)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a map with an arbitrary distribution that is specified
  /// by the elements of the @p part_cont provided, which are instances of
  /// @ref arbitrary_partition_info.
  /// @param part_cont container of elements that specify the arbitrary
  /// mapping of gids to partition ids and location ids.
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionContainer>
  map(PartitionContainer const& part_cont,
      typename std::enable_if<
        std::is_same<typename PartitionContainer::value_type,
          arbitrary_partition_info>::value>::type* = 0)
    : base_type(&part_cont, boost::mpl::false_()),
      m_comp_spec(nullptr)
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
  map(ComposedSpec const& comp_spec,
      typename std::enable_if<
        detail::has_is_composed_dist_spec<ComposedSpec>::value>::type* = 0)
    : base_type(
        partition_type(comp_spec.spec()),
        mapper_type(comp_spec.spec()),
        comp_spec,
        boost::mpl::false_()
      ),
      m_comp_spec(new composed_dist_spec(comp_spec))
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
  map(std::vector<DistSpecView> const& dist_specs,
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
        }),
        boost::mpl::false_()
      ),
      m_comp_spec(
        make_composed_dist_spec_ptr(
          [&](std::vector<size_t> const& index) {
            stapl_assert(index.size() < dist_specs.size(),
              "dimensionality of index exceeds depth of distribution specs");
            return dist_specs[index.size()];
        })
      )
  { }

  ~map(void)
  {
    if (m_comp_spec)
      delete m_comp_spec;
    m_comp_spec = nullptr;
  }
  /// @}

  /// @name Element Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the mapped value for a specific key.
  /// @param key The key of the element to access.
  /// @return Proxy over the mapped value
  //////////////////////////////////////////////////////////////////////
  second_reference operator[](key_type const& key)
  {
    this->distribution().create(key, m_comp_spec);

    return this->distribution().make_reference(key).second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the mapped value for a specific key.
  /// @param key The key of the element to access.
  /// @return Proxy over the mapped value
  //////////////////////////////////////////////////////////////////////
  reference make_reference(key_type const& key)
  {
    return this->distribution().make_reference(key);
  }

private:
  bool in_domain(key_type const& key)
  {
    return this->distribution().directory().manager()
                 .partition().global_domain().contains(key);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a <Key,Mapped> pair in the container. If the key already
  /// exists, the request will be dropped.
  /// @param val The pair of <key_type,mapped_type> to be inserted.
  /// @return Always true
  /// @bug The return should be replaced with a future of a bool indicating
  /// whether or not the insertion was successful.
  //////////////////////////////////////////////////////////////////////
  bool insert(value_type const& val)
  {
    this->incr_version();
    this->distribution().insert(val);

    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a Key, and its Mapped value in the container. If the key
  /// already exists, the request will be dropped.
  /// @param key The Key of type key_type to be inserted.
  /// @param val The Mapped element of type mapped_type to be inserted.
  /// @return Always true
  /// @bug The return should be replaced with a future of a bool indicating
  /// whether or not the insertion was successful.
  //////////////////////////////////////////////////////////////////////
  bool insert(key_type const& key, mapped_type const& val)
  {
    this->incr_version();
    this->distribution().insert(value_type(key,val));
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Asynchronous insert that executes the functor provided on the
  ///   value type if the element with the specified key exists, and otherwise
  ///   inserts the key-value pair provided.
  /// @param val The key-value pair inserted if the key is not found
  ///   in the container.
  /// @param f The Functor applied to val if it is present.
  //////////////////////////////////////////////////////////////////////
  template <typename Functor>
  void insert(value_type const& val, Functor const& f)
  {
    this->incr_version();
    this->distribution().insert_fn(
      val, std::bind(f, std::placeholders::_1, val));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Asynchronous insert that executes the functor provided on the
  ///   value type if the element with the specified key exists, and otherwise
  ///   inserts the key-value pair provided.
  /// @param key The Key of type key_type to be inserted.
  /// @param val The Mapped element of type mapped_type to be inserted.
  /// @param f The Functor applied to val if it is present.
  //////////////////////////////////////////////////////////////////////
  template <typename Functor>
  void insert(key_type const& key, mapped_type const& val, Functor const& f)
  {
    this->insert(value_type(key,val), f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove an element from the container.
  /// @param key The Key of the element to be removed.
  //////////////////////////////////////////////////////////////////////
  size_type erase(key_type const& key)
  {
    this->incr_version();
    return this->distribution().erase(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove all elements from the container.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->incr_version();
    this->distribution().clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to the first key in the map
  /// @return A global iterator of the begin
  ////////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to one past the end of the map.
  /// @return A global iterator of the end
  ////////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return this->distribution().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator over a key.
  /// @param key The Key of type key_type on which the iterator should point at
  /// @return the Iterator over the key
  /// @todo Specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(key_type const& key)
  {
    return this->distribution().make_iterator(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to the data (i.e., the pair's second
  /// element) associated with a given GID.
  /// @param gid The GID associated with the element for which we want to apply
  /// the functor and read the result.
  /// @param f The functor to apply to gid
  /// @warning This assumes that the type Functor reflects a public type named
  /// result_type and that the invocation of its function operator returns a
  /// value that is convertible to result_type. In addition, Functor's
  /// function operator must be const.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void data_apply_async(gid_type const& gid, F const& f)
  {
    this->distribution().data_apply_async(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to the data (i.e., the pair's second
  /// element) associated with a given GID and return the value.
  /// @param gid The GID associated with the element for which we want to apply
  /// the functor and read the result.
  /// @param f The functor to apply to gid
  /// @return The result of applying f to the value at gid
  /// @warning This assumes that the type Functor reflects a public type named
  /// result_type and that the invocation of its function operator returns a
  /// value that is convertible to result_type. In addition, Functor's
  /// function operator must be const.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename F::result_type data_apply(gid_type const& gid, F const& f)
  {
    return this->distribution().data_apply(gid, f);
  }

  /// @}

  /// @name Memory and Domain Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements in the container.
  ///
  /// This method is one-sided, If other locations may be concurrently
  /// performing operations that change their local size and the effects
  /// are desired to be observed in a deterministic way, then appropriate
  /// synchronization, e.g. a fence, may be required before or after the
  /// call to size, to enforce appropriate ordering.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->distribution().domain().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns 1 if the specified key exists, and otherwise 0
  /// @param key key to find
  /// @return 0 or 1 if the unique key is in the map
  //////////////////////////////////////////////////////////////////////
  int count(key_type const& key)
  {
    return find(key) == this->end() ? 0 : 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the ref of the member if the specified key exists,
  ///   and otherwise an iterator to the end of the map.
  /// @param key key to find
  /// @return a reference to the mapped value for the specified key
  //////////////////////////////////////////////////////////////////////
  iterator find(key_type const& key)
  {
    return this->distribution().find(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return if the container is empty or not.
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return this->size() == 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the domain of the container
  //////////////////////////////////////////////////////////////////////
  domain_type domain(void) const
  {
    return domain_type(this->distribution());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Redistribute the data stored in the container to match the
  /// distribution specified by the distribution view provided.
  ///
  /// @param dist_view View-based specification of the desired distribution.
  ///
  /// @note The method is only available in map instances that use
  /// @ref view_based_partition and @ref view_based_mapper as their
  /// partition and mapper types, respectively.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  void redistribute(DistSpecView const& dist_view,
    typename boost::enable_if<boost::mpl::and_<
      is_distribution_view<DistSpecView>,
      is_view_based<partition_type>,
      is_view_based<mapper_type> > >::type* =0)
  {
    this->distribution().redistribute(
      std::shared_ptr<DistSpecView>(new DistSpecView(dist_view)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the metadata of the distribution with mapping information
  /// for elements that will be inserted into the container.
  /// @param updates vector of update information that specifies a contiguous
  /// range of GIDs that will be mapped to the specified partition id and
  /// location id.
  ///
  /// @warning This method is only supported for container instances using
  /// view-based partitioner and mapper classes.
  //////////////////////////////////////////////////////////////////////
  void update_distribution(
    std::vector<std::tuple<std::pair<gid_type, gid_type>, cid_type,
      location_type>> const& updates)
  {
    this->incr_version();
    this->distribution().update(updates);
  }

  /// @}

  //////////////////////////////////////////////////////////////////////
  /// @todo Method to be removed. it is only for testing purposes
  //////////////////////////////////////////////////////////////////////
  void print(void) const
  {
    this->distribution().print();
  }
}; // class map

} // namespace stapl

#endif
