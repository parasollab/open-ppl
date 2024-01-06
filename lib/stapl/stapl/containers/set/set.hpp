/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SET_SET_HPP
#define STAPL_CONTAINERS_SET_SET_HPP

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>
#include <stapl/containers/set/set_traits.hpp>
#include <stapl/views/set_view.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/runtime.hpp>
#include <stapl/runtime/utility/functional.hpp>
#include <boost/mpl/bool.hpp>
#include "proxy.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for @ref set.
/// @ingroup psetTraits
/// @see set container_traits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename ...OptionalParams>
struct container_traits<set<Key, OptionalParams...>>
  : public set_impl::compute_set_traits<Key, OptionalParams...>::type
{
  typedef void enable_view_reference;

  template <typename C>
  struct construct_view
  {
    typedef set_view<C> type;
  };
};


template<typename Key, typename ...OptionalParams>
class set
  : public container<set<Key, OptionalParams...>>
{
private:
  typedef container<set>                                 base_type;

  STAPL_IMPORT_TYPE(typename container_traits<set>, compare_type)
  STAPL_IMPORT_TYPE(typename container_traits<set>, partition_type)
  STAPL_IMPORT_TYPE(typename container_traits<set>, mapper_type)
  STAPL_IMPORT_TYPE(typename container_traits<set>, manager_type)
  STAPL_IMPORT_TYPE(typename container_traits<set>, container_manager_type)
  STAPL_IMPORT_TYPE(typename container_traits<set>, directory_type)

protected:
  /// Domain used by the partitioner
  typedef typename partition_type::value_type            part_dom_t;

  /// Domain used by the mapper
  typedef typename mapper_type::domain_type              map_dom_t;


  std::vector<Key>                                       m_temporary_storage;

  bool                                                   m_using_buffer;

  std::pair <Key, Key>                                   m_min_max;

public:
  STAPL_IMPORT_TYPE(typename container_traits<set>, gid_type)

  typedef Key                                            key_type;
  typedef Key                                            value_type;
  typedef compare_type                                   key_compare;
  typedef size_t                                         size_type;
  typedef typename base_type::distribution_type          distribution_type;

  typedef typename distribution_type::reference          reference;
  typedef typename distribution_type::const_reference    const_reference;
  typedef typename distribution_type::iterator           iterator;

  typedef iterator_domain<distribution_type >            domain_type;

  /// Distribution metadata type used for coarsening
  typedef typename distribution_type::loc_dist_metadata  loc_dist_metadata;

  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a parallel set where the ownership of keys is
  /// determined by the domain calculated based on the key space if it is
  /// using the temporary buffer and the rebalance function, otherwise the
  /// ownership of keys is determined by a simple balanced distribution of
  /// the key space.
  /// @param use_buffer bool that specifies the use of temporary buffer.
  //////////////////////////////////////////////////////////////////////
  set(bool use_buffer=false)
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
     ), m_using_buffer(use_buffer)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a set with a given domain.
  //////////////////////////////////////////////////////////////////////
  set(typename partition_type::value_type const& domain)
    : base_type(
        directory_type(
          partition_type(domain, get_num_locations()),
          mapper_type(map_dom_t(get_num_locations()))),
        container_manager_type(
          partition_type(domain, get_num_locations()),
          mapper_type(map_dom_t(get_num_locations()))
        ), boost::mpl::false_()), m_using_buffer(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a parallel set where the ownership of keys is
  /// determined by a given partition.
  ///
  /// @param partition Partition strategy for the GID space.
  //////////////////////////////////////////////////////////////////////
  set(partition_type const& partition)
    : base_type(
        directory_type(
          partition,
          mapper_type(partition.domain())),
        container_manager_type(partition, mapper_type(partition.domain())
      ), boost::mpl::false_()), m_using_buffer(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a set with a given partitioner and mapper
  /// an instance of mapper
  ///
  /// @param partition The partition to be used for this set
  /// @param mapper    The mapper to be used for distribution
  //////////////////////////////////////////////////////////////////////
  set(partition_type const& partition, mapper_type const& mapper)
    : base_type(partition, mapper, boost::mpl::false_()), m_using_buffer(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a set with a distribution that is specified by
  /// the @p dist_view provided.
  /// @param dist_view view that specifies the mapping of container GIDs
  /// to partition ids, and partition ids to location ids.
  ///
  /// @ref distribution_specs contains a set of functions to create views
  /// for common data distributions.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  set(DistSpecView const& dist_view,
      typename std::enable_if<
        is_distribution_view<DistSpecView>::value &&
        !detail::has_is_composed_dist_spec<DistSpecView>::value
      >::type* = 0)
    : base_type(std::shared_ptr<DistSpecView>(new DistSpecView(dist_view)),
                boost::mpl::false_()), m_using_buffer(false)
  { }

  /// @}

  /// @name Element Manipulation
  /// @{

  reference operator[](key_type const& key)
  {
    return this->distribution().make_reference(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference for a specific key.
  /// @param key The key of the element to access.
  /// @return Proxy over the key
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
  /// @brief Insert a Key in the container. If the key already
  /// exists, the request will be dropped.
  /// @param val The element to be inserted.
  /// @todo The return should be replaced with a future of a bool indicating
  /// whether or not the insertion was successful.
  //////////////////////////////////////////////////////////////////////
  void insert(value_type const& val)
  {
    if (m_using_buffer)
    {
      if (m_temporary_storage.size() == 0)
        m_min_max = std::make_pair(val, val);

      if (m_min_max.first > val)
          m_min_max.first = val;

      if (m_min_max.second < val)
          m_min_max.second = val;

      m_temporary_storage.push_back(val);
    }
    else
    {
      stapl_assert(in_domain(val),
                   "Trying to insert a key outside the domain\n");
      this->incr_version();
      this->distribution().insert(val);
    }
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
  /// @brief Find an element from the set
  /// @return A global iterator of the element
  /// @todo Should return a reference instead of iterator ?
  //////////////////////////////////////////////////////////////////////
  iterator find(value_type const& val)
  {
    return this->distribution().find(val);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to the beginning of the set.
  /// @return A global iterator of the begin
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to one past the end of the set.
  /// @return A global iterator of the end
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return this->distribution().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator over a key.
  /// @param key The Key of type key_type on which the iterator should point at
  /// @return the Iterator over the key
  /// @todo Specialize sequence operation to avoid exporting this method
  /// @todo Should this be removed?
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(key_type const& key)
  {
    return this->distribution().make_iterator(key);
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
  size_t size(void)
  {
    return this->distribution().domain().size();
  }

  bool empty(void) const
  {
    return (this->size()==0);
  }

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

  using pair_type = std::pair<value_type, value_type>;
  pair_type identity(pair_type val)
  {
    return val;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Redistribute the data stored in the temporary storage to
  /// redistribute across locations.
  //////////////////////////////////////////////////////////////////////
  void rebalance(void)
  {
    struct min_max_op
    {
      pair_type operator()(pair_type const& a, pair_type const& b) const
      {
        return std::make_pair((b.first < a.first) ? b.first : a.first,
          (a.second < b.second) ? b.second : a.second);
      }
    };

    using operator_type_m_min_max = decltype(non_commutative(min_max_op{}));

    operator_type_m_min_max op_m_min_max{min_max_op{}};

    future<pair_type> m_min_max_val = allreduce_rmi(op_m_min_max,
      this->get_rmi_handle(), &set::identity, m_min_max);

    m_min_max = m_min_max_val.get();

    // Create new domain.
    indexed_domain<value_type> new_domain(m_min_max.first, m_min_max.second);


    // Create new partition using the new domain.
    balanced_partition<indexed_domain<Key>>
      new_partition(new_domain, this->get_num_locations());

    // Replacing mapper and partition of the distribution.
    this->distribution().replace_partition_mapper(new_partition,
      mapper_type(new_partition.domain()));

    // Change buffered state
    m_using_buffer = false;

    // Insert elements to set.
    for (typename std::vector<value_type>::size_type i = 0;
      i != m_temporary_storage.size(); i++)
      this->insert(m_temporary_storage[i]);

    // rmi_fence() is needed to ensure all elements are inserted in
    // the container before continuing.
    rmi_fence();
  }
  /// @}
}; // class set

} // namespace stapl

#endif // STAPL_CONTAINERS_SET_SET_HPP
