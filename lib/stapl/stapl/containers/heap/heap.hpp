/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_HEAP_HPP
#define STAPL_CONTAINERS_HEAP_HPP


#include <stapl/containers/base/container.hpp>
#include <stapl/containers/heap/traits/heap_traits.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>
#include <stapl/domains/indexed.hpp>

#include "heap_fwd.hpp"
#include "proxy.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for heap.
/// @ingroup pheapTraits
///
/// @tparam T Type of the stored elements in the container.
/// @tparam Comp The comparator used to infer an ordering between
///         elements.
/// @tparam Partitioner Partition strategy that defines how to partition
/// the original domain into subdomains.
/// @tparam Mapper Mapper that defines how to map the subdomains produced
/// by the partition to locations.
/// @tparam Traits A traits class that defines customizable components
/// of the heap container.
////////////////////////////////////////////////////////////////////////
template<typename T, typename Comp,
         typename Partitioner, typename Mapper, typename Traits >
struct container_traits<heap<T, Comp, Partitioner, Mapper, Traits> >
  : public select_parameter<
      Traits,
      heap_traits<
        T,
        typename select_parameter<Comp, std::less<T> >::type,
        typename select_parameter<
          Partitioner, balanced_partition<indexed_domain<size_t> >
        >::type,
        typename select_parameter<Mapper, mapper<size_t> >::type
     >
   >::type
{
private:
  typedef typename select_parameter<
    Comp, std::less<T>
   >::type                                                   comparator_t;

  typedef typename select_parameter<
    Partitioner,
    balanced_partition<indexed_domain<size_t> >
  >::type                                                    partition_t;

  typedef typename select_parameter<
    Mapper, mapper<size_t>
  >::type                                                    mapper_t;

  typedef typename select_parameter<
    Traits,
    heap_traits<T, comparator_t, partition_t, mapper_t>
  >::type                                                    traits_t;

  typedef heap<T,Comp, Partitioner, Mapper, Traits>          container_t;
  typedef typename traits_t::
    template construct_distribution<container_t>::type       dist_t;

public:
  typedef list_distributed_domain<dist_t>                    domain_type;
  typedef T                                                  value_type;
  typedef container_accessor<container_t>                    accessor_t;
  typedef proxy<value_type, accessor_t>                      reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Parallel Priority Queue container
/// @ingroup pheap
/// @tparam T Type of the stored elements in the container.
/// @tparam Comp The comparator used to infer an ordering between
///         elements.
/// @tparam Partitioner Partition strategy that defines how to partition
///         the original domain into subdomains.
/// @tparam Mapper Mapper that defines how to map the subdomains produced
///         by the partition to locations.
/// @tparam Traits A traits class that defines customizable components
///         of the heap container.
//////////////////////////////////////////////////////////////////////
template<typename T,
         typename Comp,
         typename Partitioner,
         typename Mapper, typename Traits>
class heap
  : public container<heap<T, Comp, Partitioner, Mapper, Traits> >
{
  typedef typename select_parameter<Partitioner,
          balanced_partition<
          indexed_domain<size_t> > >::type         partition_type;
  typedef typename select_parameter<Mapper,
                   mapper<size_t> >::type          mapper_type;

private:
  typedef typename select_parameter<Traits,
          heap_traits<T, typename select_parameter<
                         Comp, std::less<T> >::type,
          typename select_parameter<Partitioner,
                   balanced_partition<indexed_domain<size_t> > >::type,
          typename select_parameter<Mapper,
                                    mapper<size_t> >::type
   > >::type                                                traits_type;

  typedef container<heap>                                   base_type;

  BOOST_STATIC_ASSERT((boost::is_convertible<T,
                       typename container_traits<heap>::value_type>::value));
  BOOST_STATIC_ASSERT((boost::is_convertible<Comp,
                       typename container_traits<heap>::comp_type>::value));

public:
  typedef typename base_type::distribution_type     distribution_type;
  typedef typename distribution_type::directory_type   directory_type;
  typedef typename distribution_type::
          container_manager_type          container_manager_type;
  typedef typename traits_type::manager_type             manager_type;

  typedef T                                          value_type;
  typedef typename traits_type::gid_type             gid_type;
  typedef typename distribution_type::domain_type    domain_type;

  typedef size_t                                     size_type;

  typedef typename distribution_type::reference      reference;
  typedef typename distribution_type::iterator       iterator;
  typedef typename distribution_type::const_iterator  const_iterator;

  ///Distribution metadata type used for coarsening.
  typedef typename distribution_type::loc_dist_metadata loc_dist_metadata;

  typedef indexed_domain<size_t> dom_t;

protected:

  domain_type m_domain;

public:

  /// @name Constructors
  /// @{

  heap()
    : base_type(distribution_type(directory_type(), container_manager_type())),
      m_domain(this->distribution().domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a heap given a partitioner and a mapper.
  /// @param partitioner An instance of the partitioner to use for
  ///        the distribution.
  /// @param mapper An instance of the mapper to use for the distribution.
  //////////////////////////////////////////////////////////////////////
  heap(partition_type const& partitioner, mapper_type const& mapper)
    : base_type(partitioner, mapper),
      m_domain(this->distribution().domain())
  { }

  heap(heap const& other)
    : base_type(other),
      m_domain(this->distribution().domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Populate the heap by copying the elements represented by a view.
  /// @tparam View The view type.
  /// @param v The view to populate the container.
  //////////////////////////////////////////////////////////////////////
  template <typename View>
  heap(View v)
    : base_type(
        distribution_type(directory_type(), container_manager_type())
      ),
      m_domain(this->distribution().domain())
  {
    this->make(v);
  }

  /// @}

  /// @name Element Manipulation
  /// @{

  //================================================================
  // iterators
  //================================================================

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over the first element
  ///        of the container.
  //////////////////////////////////////////////////////////////////////
  iterator begin()
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over one position past the last element
  ///        in the container.
  //////////////////////////////////////////////////////////////////////
  iterator end()
  {
    return this->distribution().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const iterator over the first element
  ///        of the container.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin() const
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const iterator over one position past the last element
  ///        in the container.
  //////////////////////////////////////////////////////////////////////
  const_iterator end() const
  {
    return this->distribution().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the iterator of a GID.
  /// @param gid The GID for which to create the iterator.
  /// @return An iterator to the element at gid.
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return this->distribution().make_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to an element using its GID.
  /// @param gid The GID for which to create a reference.
  /// @return A proxy of the element at gid.
  //////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    return this->distribution().make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to an element using its GID and domain.
  /// @param gid The GID for which to create a reference.
  /// @param gid The domain of the existing element.
  /// @return A proxy of the element at gid.
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(domain_type const& domain, gid_type const& gid)
  {
    return this->distribution().make_iterator(domain,gid);
  }


  //================================================================
  // heap methods
  //================================================================

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert an element into the container.
  /// @param elem Element to be added.
  //////////////////////////////////////////////////////////////////////
  void push(T const& elem)
  {
    this->incr_version();
    this->distribution().push(elem);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the first element according to the order
  ///        determined by the comparator and removes it from the container.
  //////////////////////////////////////////////////////////////////////
  value_type pop()
  {
    this->incr_version();
    return this->distribution().pop();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the k greatest values existing in the container
  ///        according to the order determined by the comparator.
  /// @param k Number of elements to remove.
  /// @return A view containing the k greatest values removed
  ///         from the container.
  //////////////////////////////////////////////////////////////////////
  array_view<array<value_type> > pop_k(size_t const& k)
  {
    this->incr_version();
    return this->distribution().pop_k(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the first element according to the order
  ///        determined by the comparator.
  //////////////////////////////////////////////////////////////////////
  value_type top()
  {
    return this->distribution().top();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a copy of the k greatest values existing in the container
  ///        according to the order determined by the comparator.
  /// @param k Number of greatest elements to obtain.
  /// @return A view containing a copy of the k greatest
  ///         values of the container.
  //////////////////////////////////////////////////////////////////////
  array_view<array<value_type> > top_k(size_t const& k)
  {
    return this->distribution().top_k(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns if the container follows the ordering
  ///        inferred by the comparator.
  //////////////////////////////////////////////////////////////////////
  bool is_heap()
  {
    return this->distribution().is_heap();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear all elements in the container and fill it
  ///        with elements of a view.
  /// @tparam The view used to populate the container.
  //////////////////////////////////////////////////////////////////////
  template <typename V>
  void make(V v)
  {
    this->incr_version();
    this->distribution().make(v);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Rearranges the container elements using the comparator
  ///        provided in such a way that it forms a heap.
  //////////////////////////////////////////////////////////////////////
  void heapify()
  {
    this->distribution().heapify();
  }

  /// @}

public:
  /// @name Memory and Domain Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the container.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return this->domain().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return whether the container is empty
  ///        (i.e. whether its size is 0).
  //////////////////////////////////////////////////////////////////////
  bool empty() const
  {
    return (this->size()==0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove all elements from the container.
  //////////////////////////////////////////////////////////////////////
  void clear()
  {
    this->incr_version();
    this->distribution().clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the distribution domain.
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return domain_type(this->distribution());
  }

  /// @}
}; //class heap

} //namespace stapl

#endif /* STAPL_CONTAINERS_HEAP_HPP */
