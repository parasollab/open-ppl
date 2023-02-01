/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LIST_LIST_HPP
#define STAPL_CONTAINERS_LIST_LIST_HPP

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/list/traits/list_traits.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>
#include <stapl/containers/distribution/is_distribution_view.hpp>

#include <stapl/domains/indexed.hpp>

#include "list_fwd.hpp"
#include "proxy.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for list.
/// @ingroup plistTraits
/// @see list container_traits
//////////////////////////////////////////////////////////////////////
template<typename T, typename PS, typename M, typename Traits>
struct container_traits<list<T, PS, M, Traits> >
  : public select_parameter<
      Traits,
      list_traits<
        T,
        typename select_parameter<
          PS, balanced_partition<indexed_domain<size_t> >
        >::type,
        typename select_parameter<M, mapper<size_t> >::type
    > >::type
{
  typedef typename select_parameter<
      Traits,
      list_traits<
        T,
        typename select_parameter<
          PS, balanced_partition<indexed_domain<size_t> >
        >::type,
        typename select_parameter<M, mapper<size_t> >::type
    > >::type traits_t;

  typedef list<T, PS, M, Traits>                                container_t;

  typedef typename traits_t::template
           construct_distribution<container_t>::type            dist_t;

  typedef list_distributed_domain<dist_t>                       domain_type;

  typedef container_accessor<container_t>                       accessor_t;
  typedef proxy<typename traits_t::value_type, accessor_t>      reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Parallel list container.
/// @ingroup plist
///
/// @tparam T Type of the stored elements in the container. T must be
///           default constructable, copyable and assignable.
/// @tparam PS Partition type that defines how to partition the number
///            of elements into sub set of elements. The default
///            partition is @ref balanced_partition.
/// @tparam M Mapper type that defines how to map the subdomains
///           produced by the partition to locations. The default
///           mapper is @ref mapper.
/// @tparam Traits A traits class that defines customizable components
///                of list, such as base container type. The default
///                traits class is @ref list_traits.
////////////////////////////////////////////////////////////////////////
template<typename T, typename PS, typename M, typename Traits>
class list
  : public container<list<T, PS, M, Traits> >
{
  typedef typename container_traits<list>::partition_type   partition_type;
  typedef typename container_traits<list>::mapper_type      mapper_type;

private:
  typedef typename container_traits<list>::traits_t      traits_type;

  typedef container<list>                                base_type;

  typedef indexed_domain<size_t>                         dom_t;

public:
  typedef typename base_type::distribution_type          distribution_type;
  typedef typename distribution_type::directory_type     directory_type;

  typedef typename distribution_type::
                       container_manager_type            container_manager_type;

  typedef typename traits_type::manager_type             manager_type;

  typedef T                                              value_type;
  typedef typename traits_type::gid_type                 gid_type;
  typedef typename distribution_type::domain_type        domain_type;

  typedef size_t                                         size_type;

  typedef typename distribution_type::reference          reference;
  typedef typename distribution_type::iterator           iterator;
  typedef typename distribution_type::const_iterator     const_iterator;

  /// Distribution metadata type used for coarsening
  typedef typename distribution_type::loc_dist_metadata  loc_dist_metadata;

  /// Domain used by the mapper
  typedef typename mapper_type::domain_type              map_dom_t;

protected:

  /// Domain used by the partition
  /// @todo Determine if m_domain can be removed and its use replaced
  domain_type m_domain;

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper constructor for nested list construction.
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  void init(size_t n, boost::tuples::cons<X,Y> dims)
  {
    typedef typename traits_type::base_container_type base_container_type;
    iterator it=this->begin();

    for (; it!=this->end(); ++it)
    {
      value_type* nc = new value_type(dims);
      gid_type gid = index_of(*it);
      if (gid.m_location==this->get_location_id())
      {
        this->distribution().container_manager().
          invoke(gid,
                 &base_container_type::set_element,
                 gid, container_wrapper_ref<value_type>(*nc));
      }
    }
    //rmi_fence();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Base case to stop the recursion construction.
  //////////////////////////////////////////////////////////////////////
  void init(size_t, boost::tuples::null_type)
  { }

public:
  /// @name Constructors
  /// @{

  list(void)
    : base_type(distribution_type(directory_type(), container_manager_type())),
      m_domain(this->distribution().domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a list with @p n default constructed elements.
  //////////////////////////////////////////////////////////////////////
  list(size_t n)
    : base_type(partition_type(dom_t(n),get_num_locations()),
                mapper_type(map_dom_t(get_num_locations()) ) ),
      m_domain(this->distribution().domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a list with a given size and default value.
  ///
  /// @param n The size of the list.
  /// @param default_value The initial value of the elements in the
  ///                      container.
  //////////////////////////////////////////////////////////////////////
  list(size_t n, value_type const& default_value)
    : base_type(partition_type(dom_t(n), get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())),
                default_value ),
      m_domain(this->distribution().domain())
  { }

  ////////////////////////////////////////////////////////////////////////
  /// @brief Construct a list given a mapper and a size parameter.
  /// @param n Size of the list.
  /// @param mapper An instance of the mapper to use for the distribution.
  ////////////////////////////////////////////////////////////////////////
  list(size_t n, mapper_type const& mapper)
    : base_type(
        partition_type(dom_t(n), get_num_locations()),
        mapper),
      m_domain(this->distribution().domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a list with a given partitioner.
  /// @param ps An instance of the partitioner to use for the distribution.
  //////////////////////////////////////////////////////////////////////
  list(partition_type const& ps)
    : base_type(ps, mapper_type(map_dom_t(ps.size())) ),
      m_domain(this->distribution().domain())

  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a list given a partitioner and a mapper.
  /// @param partitioner An instance of the partitioner to use for
  ///        the distribution.
  /// @param mapper An instance of the mapper to use for the distribution.
  //////////////////////////////////////////////////////////////////////
  list(partition_type const& partitioner, mapper_type const& mapper)
    : base_type(partitioner, mapper),
      m_domain(this->distribution().domain())
  { }

  template <typename DistSpecView>
  list(DistSpecView const& dist_view,
    typename boost::enable_if<is_distribution_view<DistSpecView> >::type* =0)
    : base_type(std::shared_ptr<DistSpecView>(new DistSpecView(dist_view))),
      m_domain(this->distribution().domain())
  { }

  template <typename DistSpecView>
  list(DistSpecView const& dist_view, value_type const& default_value,
    typename boost::enable_if<is_distribution_view<DistSpecView> >::type* =0)
    : base_type(std::shared_ptr<DistSpecView>(new DistSpecView(dist_view)),
                default_value),
      m_domain(this->distribution().domain())
  { }


  ////////////////////////////////////////////////////////////////////////
  /// @brief Copy constructs a list from another list container.
  ////////////////////////////////////////////////////////////////////////
  list(list const& other)
    : base_type(other),m_domain(this->distribution().domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a list with a given size and default value
  ///        where the value type of the container is itself a
  ///        parallel container.
  ///
  /// @param n The size of the list.
  /// @param default_value The initial value of the elements in the
  ///                      container.
  /// @param dis_policy A distribution policy that specifies how to
  ///                   distribute the nested containers, in the
  ///                   context of containers of containers.
  ////////////////////////////////////////////////////////////////////////
  template <typename DP>
  list(size_t n, value_type const& default_value, DP const& dis_policy)
    : base_type(partition_type(dom_t(n), get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())),
                default_value, dis_policy),
      m_domain(this->distribution().domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a list of lists with given n-dimensional size.
  ///
  /// @param dims A cons list specifying the dimensions of the containers.
  ////////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  list(boost::tuples::cons<X,Y> dims)
    : base_type(partition_type(dom_t(dims.get_head()),
                               get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())),
                dims),
      m_domain(this->distribution().domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a list of lists with given n-dimensional size.
  ///
  /// @param dims A cons list specifying the dimensions of the containers.
  /// @param dis_policy A distribution policy that specifies how to
  ///                   distribute the nested containers.
  ////////////////////////////////////////////////////////////////////////
  template <typename X, typename Y, typename DP>
  list(boost::tuples::cons<X,Y> dims, DP const& dis_policy)
    : base_type(partition_type(dom_t(dims.get_head()),
                               get_num_locations()),
                mapper_type(map_dom_t(get_num_locations())),
                dims,
                dis_policy),
      m_domain(this->distribution().domain())
  { }

  /// @}

  /// @name Element Manipulation
  /// @{

  iterator begin(void)
  {
    return this->distribution().
      make_iterator(this->distribution().domain().first());
  }

  const_iterator begin(void) const
  {
    return this->distribution().
      make_const_iterator(this->distribution().domain().first());
  }

  const_iterator cbegin(void) const
  {
    return this->distribution().
      make_const_iterator(this->distribution().domain().first());
  }

  iterator end(void)
  {
    return this->distribution().
      make_iterator(this->distribution().domain().open_last());
  }

  const_iterator end(void) const
  {
    return this->distribution().
      make_const_iterator(this->distribution().domain().open_last());
  }

  const_iterator cend(void) const
  {
    return this->distribution().
      make_const_iterator(this->distribution().domain().open_last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a reference to the specified element in the list.
  /// @param gid Gid of the element.
  ////////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    return this->distribution().make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an iterator pointing to the specified element
  ///        in the list.
  /// @param gid Gid of the element.
  ////////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return this->distribution().make_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a const_iterator pointing to the specified element
  ///        in the list.
  /// @param gid Gid of the element.
  ////////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(gid_type const& gid) const
  {
    return this->distribution().make_const_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the first element of the list.
  //////////////////////////////////////////////////////////////////////
  reference front(void)
  {
    return make_reference(this->distribution().domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the last element of the list.
  //////////////////////////////////////////////////////////////////////
  reference back(void)
  {
    return make_reference(this->distribution().domain().last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given @p value before the element pointed to by
  ///        the iterator @p pos and returns an iterator to the new element.
  //////////////////////////////////////////////////////////////////////
  iterator insert(iterator const& pos, value_type const& value)
  {
    this->incr_version();
    return this->distribution().insert(pos, value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the element pointed by the iterator @p pos and returns
  ///        an iterator pointing to the next element.
  //////////////////////////////////////////////////////////////////////
  iterator erase(iterator const& pos)
  {
    this->incr_version();
    return this->distribution().erase(pos);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove all elements from the container
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
  /// @brief Inserts the given @p value at the beginning of the list.
  //////////////////////////////////////////////////////////////////////
  void push_front(value_type const& value)
  {
    this->incr_version();
    this->distribution().push_front(value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given @p value at the end of the list.
  //////////////////////////////////////////////////////////////////////
  void push_back(value_type const& value)
  {
    this->incr_version();
    this->distribution().push_back(value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert the given @p value into the list.
  ///
  /// The value is inserted in the list on the location where add is invoked.
  //////////////////////////////////////////////////////////////////////
  void add(value_type const& value)
  {
    this->incr_version();
    this->distribution().push_back_local(value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the first element of the list.
  //////////////////////////////////////////////////////////////////////
  void pop_front(void)
  {
    this->incr_version();
    this->distribution().pop_front();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the last element of the list.
  //////////////////////////////////////////////////////////////////////
  void pop_back(void)
  {
    this->incr_version();
    this->distribution().pop_back();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Splits the list at the given position @p pos and inserts
  ///        between the two split lists the sublist defined from
  ///        @p first to @p last iterator in the specified list @p pl.
  ///
  /// @todo This method needs to be implemented.
  //////////////////////////////////////////////////////////////////////
  void splice(iterator pos, list& pl, iterator first, iterator l)
  {
    this->incr_version();
    abort("Splice functionality not implemented.");
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
    return this->domain().size();
  }

  bool empty(void) const
  {
    return (this->size()==0);
  }

  domain_type domain(void) const
  {
    return m_domain;
  }

  /// @}
}; // class list

} // namespace stapl

#endif // STAPL_CONTAINERS_LIST_LIST_HPP
