/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_OPERATIONS_CONST_VIEW_ITERATOR_HPP
#define STAPL_VIEWS_OPERATIONS_CONST_VIEW_ITERATOR_HPP

#include <stapl/views/view_traits.hpp>
#include <stapl/views/type_traits/is_identity.hpp>
#include <stapl/views/type_traits/is_domain_sparse.hpp>
#include <stapl/views/proxy/const_index_accessor.hpp>
#include <stapl/views/iterator/iterator_facade.h>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Definition of read only iterator used for the
///        segmented_view @c View.
///
/// The reference type associated with this iterator is a view over
/// the view's container.
//////////////////////////////////////////////////////////////////////
template <typename View>
class const_view_iterator
{
  typedef View                                     view_type;
  typedef typename View::domain_type               domain_type;
  typedef typename View::index_type                index_type;

  View const*  m_view;
  index_type   m_index;

public:
  typedef const_view_iterator                      iterator;
  typedef std::forward_iterator_tag                iterator_category;

  typedef typename view_traits<View>::value_type   value_type;
  typedef typename view_traits<View>::const_reference    reference;
  typedef ptrdiff_t                                difference_type;

  typedef value_type*                              pointer;

  const_view_iterator(void)
    : m_view(0),
      m_index()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an iterator over an element of the given @c view
  ///        at position @c index.
  //////////////////////////////////////////////////////////////////////
  const_view_iterator(View const& view, index_type const& index)
    : m_view(&view), m_index(index)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an iterator over the given @c view at position
  ///        @c index.
  //////////////////////////////////////////////////////////////////////
  const_view_iterator(View const* view, index_type const& index)
    : m_view(view), m_index(index)
  { }

  reference operator*() const
  {
    return (*m_view)[m_index];
  }

  const_view_iterator& operator=(const_view_iterator const& other)
  {
    if (this != &other) {
      m_view  = other.m_view;
      m_index = other.m_index;
    }
    return *this;
  }

  index_type index() const
  {
    return m_index;
  }

  void define_type(typer &t)
  {
    t.member(m_view);
    t.member(m_index);
  }

  const_view_iterator& operator++()
  {
    m_index = m_view->next(m_index);
    return (*this);
  }

  const_view_iterator operator++(int)
  {
    const_view_iterator tmp = *this;
    operator++();
    return tmp;
  }

  const_view_iterator& operator--()
  {
    m_index = m_view->prev(m_index);
    return (*this);
  }

  const_view_iterator operator--(int)
  {
    const_view_iterator tmp = *this;
    operator--();
    return tmp;
  }

  bool operator!=(const_view_iterator const& it) const
  {
    return it.m_index != m_index;
  }

  bool operator==(const_view_iterator const& it) const
  {
    return it.m_index == m_index;
  }

  template <typename Iterator>
  bool operator<(Iterator const& it) const
  {
    return m_index < it.m_index;
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo For loop should not be there for random access iterators.
  /// @todo Verify who is using this class and what type of iterator
  ///       category is used.
  //////////////////////////////////////////////////////////////////////
  const_view_iterator operator+(long n) const
  {
    const_view_iterator tmp = *this;
    for (long i=0;i<n;i++)
      tmp.m_index = tmp.m_view->next(tmp.m_index);
    return tmp;
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo For loop should not be there for random access iterators.
  //////////////////////////////////////////////////////////////////////
  const_view_iterator operator-(long n) const
  {
    const_view_iterator tmp = *this;
    for (long i=0;i<n;i++)
      tmp.m_index = tmp.m_view->prev(tmp.m_index);
    return tmp;
  }

  ptrdiff_t operator-(const_view_iterator const& other) const
  {
    long n = other.m_index - this->m_index;
    return n;
  }

  friend bool operator<(const_view_iterator const& c,
                        const_view_iterator const& d)
  {
    return c.m_index < d.m_index;
  }
}; //class const_view_iterator

} //namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Definition of the iterator used for the @c View.
///
/// The reference type associated with this iterator is a proxy over
/// the view's value type.
//////////////////////////////////////////////////////////////////////
template <typename View, typename Category = std::forward_iterator_tag>
class const_index_iterator
  : public iterator_facade<const_index_iterator<View, Category>,
                           detail::const_index_accessor<View>,
                           Category>
{
  friend class iterator_core_access;

public:
  typedef typename View::domain_type                              domain_type;
  typedef typename domain_type::gid_type                          gid_type;
  typedef typename View::index_type                               index_type;
  typedef detail::const_index_accessor<View>                      accessor_t;

private:
  typedef iterator_facade<const_index_iterator, accessor_t, Category> base_type;
  typedef typename base_type::difference_type                     diff_t;

  View const* m_view;
  index_type  m_index;

public:
  const_index_iterator()
    : m_view(0),
      m_index()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an iterator over the given @c view at position
  ///        @c index.
  //////////////////////////////////////////////////////////////////////
  const_index_iterator(View const& view, index_type index)
    : m_view(&view), m_index(index)
  { }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the associated accessor with the iterator.
  //////////////////////////////////////////////////////////////////////
  accessor_t access() const
  {
    return accessor_t(m_view, m_index);
  }

  void increment()
  {
    m_index = m_view->next(m_index);
  }

  void decrement()
  {
    m_index = m_view->prev(m_index);
  }

  void advance(diff_t n)
  {
    m_index = m_view->advance(m_index, n);
  }

  bool equal(const_index_iterator const& rhs) const
  {
    return m_index == rhs.m_index;
  }

  template <typename OtherView>
  bool equal(const_index_iterator<OtherView, Category> const& rhs) const
  {
    return m_index == rhs.m_index;
  }

  diff_t distance_to(const_index_iterator const& rhs) const
  {
    return m_view->distance(m_index, rhs.m_index);
  }

  bool less_than(const_index_iterator const& rhs) const
  {
    return m_view->less_than(m_index, rhs.m_index);
  }

public:
  void define_type(typer& t)
  {
    t.member(m_view);
    t.member(m_index);
  }

  index_type index()
  {
    return m_index;
  }

}; //class const_index_iterator


template <typename T, typename A, typename Category>
class const_index_iterator<proxy<T,A>, Category>
  : public iterator_facade<const_index_iterator<proxy<T,A>, Category>,
                           detail::const_index_accessor<proxy<T,A> >,
                           Category>
{
  friend class iterator_core_access;

  typedef proxy<T,A> View;

public:
  typedef typename View::domain_type                              domain_type;
  typedef typename domain_type::gid_type                          gid_type;
  typedef typename View::index_type                               index_type;
  typedef detail::const_index_accessor<View>                      accessor_t;

private:
  typedef iterator_facade<const_index_iterator, accessor_t, Category> base_type;
  typedef typename base_type::difference_type                     diff_t;

  View       m_view;
  index_type m_index;

public:
  const_index_iterator()
    : m_view(),
      m_index()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an iterator over the given @c view at position
  ///        @c index.
  //////////////////////////////////////////////////////////////////////
  const_index_iterator(View const& view, index_type index)
    : m_view(view), m_index(index)
  { }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the associated accessor with the iterator.
  //////////////////////////////////////////////////////////////////////
  accessor_t access() const
  {
    return accessor_t(&m_view, m_index);
  }

  void increment()
  {
    m_index = m_view.next(m_index);
  }

  void decrement()
  {
    m_index = m_view.prev(m_index);
  }

  void advance(diff_t n)
  {
    m_index = m_view.advance(m_index, n);
  }

  bool equal(const_index_iterator const& rhs) const
  {
    return m_index == rhs.m_index;
  }

  template <typename OtherView>
  bool equal(const_index_iterator<OtherView, Category> const& rhs) const
  {
    return m_index == rhs.m_index;
  }

  diff_t distance_to(const_index_iterator const& rhs) const
  {
    return m_view.distance(m_index, rhs.m_index);
  }

  bool less_than(const_index_iterator const& rhs) const
  {
    return m_view.less_than(m_index, rhs.m_index);
  }

public:
  void define_type(typer& t)
  {
    t.member(m_view);
    t.member(m_index);
  }

  index_type index()
  {
    return m_index;
  }
}; //class const_index_iterator


} // namespace stapl

#endif  // STAPL_VIEWS_OPERATIONS_CONST_VIEW_ITERATOR_HPP
