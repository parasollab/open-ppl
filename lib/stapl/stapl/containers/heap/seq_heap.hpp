/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.
//
// // All rights reserved.
//
// // The information and source code contained herein is the exclusive
// // property of TEES and may not be disclosed, examined or reproduced
// // in whole or in part without explicit written authorization from TEES.
//
*/

#ifndef STAPL_CONTAINERS_HEAP_STORAGE_HPP
#define STAPL_CONTAINERS_HEAP_STORAGE_HPP

#include <algorithm>
#include <vector>
#include <iostream>
#include <iterator>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default container used as the internal storage of the heap
///        base_container.
/// @ingroup pheapDist
/// @tparam T The value type of the container.
/// @tparam Comp The comparator used to maintain ordering of the elements.
//////////////////////////////////////////////////////////////////////
template <typename T, typename Comp>
class seq_heap
{
public:
  typedef typename std::vector<T>::iterator       iterator;
  typedef typename std::vector<T>::const_iterator const_iterator;
  typedef T                                       value_type;

protected:
  std::vector <T>       m_v;
  Comp                  m_c;

public:
  seq_heap(Comp c = Comp())
    : m_c(c)
  { }

  seq_heap(const seq_heap<T,Comp>& other)
    : m_v(other.m_v), m_c(other.m_c)
  { }

  seq_heap(size_t size, Comp c)
    : m_c(c)
  {
    m_v.reserve(size);
  }

  seq_heap(std::vector <T> v, Comp c)
    : m_v(v), m_c(c)
  {
    this->heapify();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert an element into the container.
  /// @param elem Element to be added.
  //////////////////////////////////////////////////////////////////////
  void push(T const& elem)
  {
    m_v.push_back(elem);
    push_heap(m_v.begin(),m_v.end(), m_c);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the first element according to the order
  ///        determined by the comparator and removes it from the container.
  //////////////////////////////////////////////////////////////////////
  T pop(void)
  {
    pop_heap(m_v.begin(),m_v.end(), m_c);
    T tmp = m_v.back();
    m_v.pop_back();
    return tmp;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the first element according to the order
  ///        determined by the comparator.
  //////////////////////////////////////////////////////////////////////
  T top(void)
  {
    return m_v.front();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference of the greatest element according to the order
  ///        determined by the comparator.
  //////////////////////////////////////////////////////////////////////
  T& top_ref(void)
  {
    return m_v.back();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the greatest element according
  ///        to the order determined by the comparator.
  //////////////////////////////////////////////////////////////////////
  iterator top_it(void)
  {
    return m_v.begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear all elements in the container and fill it
  ///        with elements of a view.
  /// @tparam The view used to populate the container.
  //////////////////////////////////////////////////////////////////////
  template <typename V>
  void make(V v)
  {
    m_v.resize(v.size());
    std::copy(v.begin(), v.end(), m_v.begin());
    std::make_heap(m_v.begin(),m_v.end(),m_c);
  }

 //////////////////////////////////////////////////////////////////////
 /// @brief Returns the index of the container where the ordering specified
 ///        by the comparator is violated.
 //////////////////////////////////////////////////////////////////////
 size_t is_heap_until(void)
  {
    size_t parent = 0;
    for (size_t child = 1; child < m_v.size(); ++child)
      {
        if (m_c(m_v.begin()[parent], m_v.begin()[child]))
          return child;
        if ((child & 1) == 0)
          ++parent;
      }
    return m_v.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns if the container follows the ordering
  ///        inferred by the comparator.
  //////////////////////////////////////////////////////////////////////
  bool is_heap(void)
  {
    return is_heap_until() == m_v.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the container.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return m_v.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return whether the container is empty
  ///        (i.e. whether its size is 0).
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return m_v.empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove all elements from the container.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    m_v.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over the first element
  ///        of the container.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return m_v.begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over one position past the last element
  ///        in the container.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return m_v.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const iterator over the first element
  ///        of the container.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return m_v.begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const iterator over one position past the last element
  ///        in the container.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return m_v.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the first element of the container.
  //////////////////////////////////////////////////////////////////////
  T front(void)
  {
    return m_v.front();
  }

  void define_type(typer &t)
  {
    t.member(this->m_v);
    t.member(this->m_c);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Rearranges the container elements using the comparator
  ///        provided in such a way that it forms a heap.
  //////////////////////////////////////////////////////////////////////
  void heapify(void)
  {
    std::make_heap(m_v.begin(),m_v.end(),m_c);
  }

};

}//namespace stapl
#endif
