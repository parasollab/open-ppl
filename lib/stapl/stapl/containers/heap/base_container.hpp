/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_HEAP_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_HEAP_BASE_CONTAINER_HPP

#include <stapl/containers/base/bc_base.hpp>
#include <stapl/containers/list/list_gid.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/heap/seq_heap.hpp>
#include <stapl/containers/heap/traits/base_container_traits.hpp>
#include <stapl/domains/list_bc_domain.hpp>
#include <stapl/runtime/stapl_assert.hpp>
#include <stapl/utility/hash.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor providing promotion for the base container. It creates
///        a GID that can be use to reference an element at the base container
///        level from a raw iterator to underlying data.
/// @ingroup pheapDist
/// @tparam BC The base container type on which we create the GID over.
/// @todo Move to detail namespace if possible.
//////////////////////////////////////////////////////////////////////
template <typename BC>
struct heap_promotion
{
private:
  BC* m_ptr_bc;

public:
  //////////////////////////////////////////////////////////////////////
  /// @todo Remove the const_cast when the list_gid accepts const base
  ///       containers.
  //////////////////////////////////////////////////////////////////////
  explicit heap_promotion(BC const* p)
    : m_ptr_bc(const_cast<BC*>(p))
  { }

  template <typename IT>
  typename BC::gid_type operator()(IT it) const
  {
    return typename BC::gid_type(it, m_ptr_bc, m_ptr_bc->location());
  }
};


template<typename T, typename Comp, typename Traits>
class heap_base_container;


template<typename T, typename Comp, typename Traits>
struct container_traits<heap_base_container<T, Comp, Traits> >
{
  typedef typename Traits::value_type                      value_type;

  typedef list_bc_domain<
    heap_base_container<T, Comp, Traits>,
    heap_promotion<heap_base_container<T, Comp, Traits> >
  >                                                        domain_type;

  typedef local_iterator<
    heap_base_container<T, Comp, Traits>
  >                                                        iterator;

  typedef const_local_iterator<
    heap_base_container<T, Comp, Traits>
  >                                                        const_iterator;

  typedef typename stapl::list_gid<
    iterator, heap_base_container<T, Comp, Traits>
  >                                                        gid_type;

  typedef typename Traits::container_type                  container_type;

  typedef local_accessor<
    heap_base_container<T, Comp, Traits>
  >                                                        accessor_t;

  typedef const_local_accessor<
    heap_base_container<T, Comp, Traits>
  >                                                        const_accessor_t;

  typedef proxy<value_type, accessor_t>                    reference;

  typedef proxy<value_type, const_accessor_t>              const_reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief The base container used for @ref heap.
/// @ingroup pheapDist
///
/// @see heap
/// @tparam T The value type of the container.
/// @tparam Comp The comparator used to maintain ordering of the elements.
/// @tparam Traits The collection of traits types that defines customizable
///         components used by the base container.
/// @todo Determine which type of definitions can be
///       removed from the public interface.
////////////////////////////////////////////////////////////////////////
template<typename T, typename Comp, typename Traits>
class heap_base_container
  : public bc_base
{
public:
  typedef typename Traits::value_type                value_type;
  typedef typename Traits::comp_type                 comp_type;
  typedef typename Traits::container_type            container_type;
  typedef local_iterator<heap_base_container>        iterator;
  typedef const_local_iterator<heap_base_container>  const_iterator;
  typedef local_accessor<heap_base_container>        accessor_t;
  typedef const_local_accessor<heap_base_container>  const_accessor_t;
  typedef proxy<value_type,accessor_t>               reference;
  typedef proxy<value_type,const_accessor_t>         const_reference;
  typedef typename stapl::list_gid<
    iterator, heap_base_container
  >                                                  gid_type;
  typedef size_t                                     cid_type;

  typedef heap_promotion<heap_base_container>        heap_promotion_type;

  typedef list_bc_domain<
    heap_base_container,
    heap_promotion<heap_base_container>
  >                                                  domain_type;

protected:
  /// The underlying raw data.
  container_type             m_data;
  /// The location of the base container.
  location_type              m_loc;

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a base container using the default comparator.
  //////////////////////////////////////////////////////////////////////
  heap_base_container(Comp c = Comp())
    : m_data(c),
      m_loc(get_location_id())
  { }

  heap_base_container(heap_base_container const& other)
    : m_data(other.m_data),
      m_loc(get_location_id())
  { }

  heap_base_container(size_t sz, Comp c)
    : m_data(sz, c),
      m_loc(get_location_id())
  { }

  ~heap_base_container(void)
  {
    this->m_data.clear();
    this->m_loc = index_bounds<location_type>::invalid();
  }

  // ========================
  // Base container methods
  // ========================

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the data of the base container.
  //////////////////////////////////////////////////////////////////////
  container_type& container(void)
  {
    return m_data;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the location of the base container.
  //////////////////////////////////////////////////////////////////////
  location_type location(void) const
  {
    return m_loc;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a GID for an element from an iterator to it.
  /// @param it iterator referencing an element of the underlying data.
  /// @return GID referencing an element of the base container.
  //////////////////////////////////////////////////////////////////////
  gid_type gid_of(iterator const& it)
  {
    return gid_type(it,this,this->location());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the element corresponding to a specific GID.
  /// @param gid The id identifying an element for which we want the value.
  /// @return A copy of the element.
  //////////////////////////////////////////////////////////////////////
  value_type get_element(gid_type const& gid)
  {
    return *(gid.m_pointer);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set an element in the base container.
  /// @param gid GID identifying an element of the base container to be set.
  /// @param t The new value to be assigned.
  //////////////////////////////////////////////////////////////////////
  void set_element(gid_type const& gid, value_type const& t)
  {
    *(gid.m_pointer) = t;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the base container.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->m_data.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return whether the base container is empty
  ///        (i.e. whether its size is 0).
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return this->m_data.empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove all elements from the base container.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_data.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create on the fly a domain for the base container and return it.
  //////////////////////////////////////////////////////////////////////
  domain_type domain(void) const
  {
    return domain_type(*this, heap_promotion_type(this));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the iterator of a GID.
  /// @param gid The GID for which to create the iterator.
  /// @return An iterator to the element at gid.
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return gid.m_pointer;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to an element of the base container
  ///        using its GID.
  /// @param gid The GID for which to create a reference.
  /// @return A proxy of the element at gid.
  //////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid) const
  {
    return reference(accessor_t(this, m_data.begin()+local_position(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a local iterator to the first element
  ///        of the base container.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    iterator it(m_data.begin(),this);
    return it;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a const local iterator to the first element
  ///        of the base container.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return const_iterator(m_data.begin(),this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a local iterator to one position past the
  ///        last element of the base container.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return iterator(m_data.end(),this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a const local iterator to one
  ///        position past the last element of the base container.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return const_iterator(m_data.end(),this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the iterator of a GID.
  /// @param gid The GID for which to create the iterator.
  /// @return An iterator to the element at gid.
  //////////////////////////////////////////////////////////////////////
  iterator find(gid_type const& gid)
  {
    return gid.m_pointer;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the const iterator of a GID.
  /// @param gid The GID for which to create the iterator.
  /// @return A const iterator to the element at gid.
  //////////////////////////////////////////////////////////////////////
  const_iterator find_element(gid_type const& gid) const
  {
    return gid.m_pointer;
  }

  void define_type(typer &t)
  {
    t.base<bc_base>(*this);
    t.member(this->m_data);
    t.member(this->m_loc);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether a GID is contained in the range provided.
  /// @param gidf The first GID of the range.
  /// @param gidl The last GID of the range.
  /// @param gid The GID to search for in the range.
  //////////////////////////////////////////////////////////////////////
  bool search(gid_type const& gidf, gid_type const& gidl, gid_type const& gid)
  {
    typename gid_type::pointer_type it = gidf.m_pointer;
    while (it != gidl.m_pointer) {
      if (gid.m_pointer == it)
        return true;
      ++it;
    }
    if (gid.m_pointer == it)
      return true;
    return false;
  }

  //================================================================
  // Methods of seq heap
  //================================================================

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert an element into the container.
  /// @param elem Element to be added.
  //////////////////////////////////////////////////////////////////////
  void push(value_type const& elem)
  {
    m_data.push(elem);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the first element according to the order
  ///        determined by the comparator and removes
  ///        it from the base container.
  //////////////////////////////////////////////////////////////////////
  value_type pop(void)
  {
    return m_data.pop();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the first element according to the order
  ///        determined by the comparator.
  //////////////////////////////////////////////////////////////////////
  value_type top(void)
  {
    return m_data.top();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the GID of the greater element according
  ///        to the order determined by the comparator.
  //////////////////////////////////////////////////////////////////////
  gid_type top_gid(void)
  {
    iterator it(m_data.top_it(),this);
    gid_type result(it,this,get_location_id());
    return result;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Change the key of an element without breaking the heap property.
  /// @param gid The GID of the element to be change.
  /// @param val The new value of the element to be assign.
  ///
  /// The update of an element's key forces reevaluation of its position in
  /// the base container.
  //////////////////////////////////////////////////////////////////////
  void update_key(gid_type const& gid, value_type const& val)
  {
    this->set_element(gid, val);
    m_data.heapify();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns if the base container follows the ordering
  ///        inferred by the comparator.
  //////////////////////////////////////////////////////////////////////
  bool is_heap(void)
  {
    return m_data.is_heap();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear all elements in the base container and fill it
  ///        with elements of a view.
  /// @tparam V The view used to populate the container.
  //////////////////////////////////////////////////////////////////////
  template<typename V>
  void make(V v)
  {
    m_data.make(v);
  }

private :
  //////////////////////////////////////////////////////////////////////
  /// @brief Rearranges the container elements using the comparator
  ///        provided in such a way that it forms a heap.
  //////////////////////////////////////////////////////////////////////
  void heapify(void)
  {
    m_data.heapify();
  }
}; // class heap_base_container


//////////////////////////////////////////////////////////////////////
/// @brief Specialization hash for the heap GID.
/// @tparam iterator The local iterator of the base container
/// @tparam BC The base container type
/// @todo Relocate to list/list_gid.hpp
//////////////////////////////////////////////////////////////////////
template<typename iterator, typename BC>
struct hash<list_gid<iterator,BC*> >
{
  std::size_t operator()(list_gid<iterator,BC*> const& gid) const
  {
  return (size_t)(*gid.m_pointer) + (size_t)(gid.m_base_container);
  }
};

} // namespace stapl

#endif // STAPL_CONTAINERS_HEAP_BASE_CONTAINER_HPP
