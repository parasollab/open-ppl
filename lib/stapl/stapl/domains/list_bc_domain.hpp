/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_LIST_BC_DOMAIN_HPP
#define STAPL_DOMAINS_LIST_BC_DOMAIN_HPP

#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <cstddef>
#include <iterator>
#include <iosfwd>
#include <boost/type_traits/is_same.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a domain using base container's iterators to
///        specify the range of indexes.
///
/// @note This domain is specialized to work with the base container
///       for @ref list.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename Functor>
class list_bc_domain
{
public:
  typedef typename container_traits<Container>::gid_type index_type;
  typedef std::size_t                                    size_type;

private:
  /// @todo @c const_iterator is defined as @c const iterator.
  typedef typename Container::iterator const_iterator;

  Container const* m_cont;
  index_type       m_first;
  index_type       m_last;
  index_type       m_end;
  bool             m_dynamic;
  size_type        m_size;
  Functor          m_functor;

  //////////////////////////////////////////////////////////////////////
  /// @todo Propagate constness and add const to return reference type.
  //////////////////////////////////////////////////////////////////////
  Container /*const*/& container(void) const
  {
    stapl_assert(m_cont, "Container is not set");
    return *const_cast<Container*>(m_cont);
  }

public:
  list_bc_domain(void)
    : m_cont(0),
      m_first(),
      m_last(),
      m_end(),
      m_dynamic(true),
      m_size(index_bounds<size_type>::invalid())
  { }

  explicit list_bc_domain(Container const& cont,
                          Functor const& func = Functor())
    : m_cont(&cont),
      m_first(),
      m_last(),
      m_end(),
      m_dynamic(true),
      m_size(index_bounds<size_type>::invalid()),
      m_functor(func)
  { }

  list_bc_domain(index_type const& idx1,
                 index_type const& idx2,
                 Container const& cont,
                 size_type size = index_bounds<size_type>::invalid())
    : m_cont(&cont),
      m_first(idx1),
      m_last(idx2),
      m_end(),
      m_dynamic(false),
      m_size(size)
  { }

  list_bc_domain(index_type const& idx1,
                 index_type const& idx2,
                 list_bc_domain const& other)
    : m_cont(other.m_cont),
      m_first(idx1),
      m_last(idx2),
      m_end(),
      m_dynamic(false),
      m_size(index_bounds<size_type>::invalid()),
      m_functor(other.m_functor)
  {
    if (m_first == other.m_first && m_last == other.m_last)
      m_size = other.m_size;
  }

  Functor const& generator(void) const
  {
    return m_functor;
  }

  index_type first(void) const
  {
    if (m_dynamic) {
      const_iterator tmp = container().begin();
      return m_functor(tmp);
    }
    return m_first;
  }

  index_type last(void) const
  {
    return last_helper(
         typename boost::is_same<
              typename std::iterator_traits<const_iterator>::iterator_category,
              std::forward_iterator_tag>::type() );
  }

  index_type open_last(void) const
  {
    if (m_dynamic) {
      const_iterator tmp = container().end();
      return m_functor(tmp);
    }
    if (m_first==m_last)
      return m_last;
    return m_functor(m_last.m_pointer+1);
  }

  size_type size(void) const {
    if (!m_cont)
      return 0;
    if (m_dynamic)
      return container().size();
    else {
      if (m_size!=index_bounds<size_type>::invalid())
        return m_size;
      return std::distance(m_first.m_pointer,m_last.m_pointer + 1);
    }
  }

  bool contains(index_type const& idx) const
  {
    // TODO: Check when is possible use c.find(idx)!=c.end() to test if idx is
    // included
    // Functor f;
    if (m_dynamic)
      return idx.m_pointer!=container().end();
    else {
      if ((first() == idx) || (last() == idx))
        return true;
      else {
        const_iterator tmp = m_first.m_pointer;
        while (tmp++!=m_last.m_pointer) {
          if (tmp == idx.m_pointer)
            return true;
        }
      }
    }
    return false;
  }

  bool empty(void) const
  {
    return (size()==0);
  }

  template<typename Distance>
  index_type advance(index_type const& idx, Distance n) const
  {
    const_iterator it_idx = idx.m_pointer;
    it_idx = it_idx + n;
    return m_functor(it_idx);
  }

  size_type distance(index_type const& a, index_type const& b) const
  {
    auto dist = std::distance(a.m_pointer, b.m_pointer);
    return (dist < 0) ? -dist : dist;
  }

  bool is_same_container_domain(void) const
  {
    return m_dynamic;
  }

  list_bc_domain operator&(list_bc_domain const& other) const
  {
    if (empty() || other.empty())
      return list_bc_domain();

    index_type olower = other.first();
    index_type oupper = other.last();

    bool contains_first = other.contains(first());
    bool contains_last = other.contains(last());

    if (contains_first && contains_last)
      return *this;

    if (!contains_first && !contains_last)
      return list_bc_domain();

    if (contains_first)
      olower = first();

    if (contains_last)
      oupper = last();

    return list_bc_domain(olower,oupper,*this);
  }

  void define_type(typer& t)
  {
    t.member(m_cont);

    if (!m_dynamic)
    {
      t.member(m_first);
      t.member(m_last);
    }

    t.member(m_dynamic);
    t.member(m_size);
    t.member(m_functor);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for computing the last valid element when
  ///        the iterator is at least bi-directional.
  //////////////////////////////////////////////////////////////////////
  index_type last_helper(boost::false_type) const
  {
    if (is_same_container_domain()) {
      const_iterator tmp = container().end();
      if (container().size()>0)
        --tmp;
      return m_functor(tmp);
    }
    return m_last;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for computing the last valid element when
  ///        the iterator is a forward iterator.
  //////////////////////////////////////////////////////////////////////
  index_type last_helper(boost::true_type) const
  {
    if (is_same_container_domain()) {
      const_iterator tmp = container().begin();
      std::advance(tmp, (container().size()-1));
      return m_functor(tmp);
    }
    return m_last;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for iterators that are at least bi-directional.
  //////////////////////////////////////////////////////////////////////
  template<typename Distance>
  index_type advance_helper(index_type const& idx,
                            Distance n,
                            boost::false_type) const
  {
    const_iterator it_idx = idx.m_pointer;
    std::advance(it_idx, n);
    // if (it_idx == container().end())
    //   return index_bounds<index_type>::invalid();
    return m_functor(it_idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for forward iterators.
  //////////////////////////////////////////////////////////////////////
  template<typename Distance>
  index_type advance_helper(index_type const& idx,
                            Distance n,
                            boost::true_type) const
  {
    const_iterator it_idx = idx.m_pointer;
    if (n<0) {
      const_iterator it_tmp = container().begin();
      size_type sz = std::distance(it_tmp, it_idx);
      sz += n;
      std::advance(it_tmp, sz);
      return m_functor(it_tmp);
    }
    std::advance(it_idx, n);
    // if (it_idx==container().end())
    //   return index_bounds<index_type>::invalid();
    return m_functor(it_idx);
  }
}; // class list_bc_domain


template<typename C, typename Index>
std::ostream& operator<<(std::ostream& os, list_bc_domain<C,Index> const& d)
{
  if (d.empty())
    return os << "empty";
  return os << "[" << d.first() << ".." << d.last() << "]";
}

} // namespace stapl

#endif // STAPL_LIST_BC_DOMAIN_HPP
