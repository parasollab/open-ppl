/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ITERATOR_MEMBER_ITERATOR_H
#define STAPL_ITERATOR_MEMBER_ITERATOR_H

#include <stapl/views/proxy/accessor_base.hpp>
#include <stapl/views/iterator/iterator_facade.h>
#include <stapl/views/null_reference.hpp>
#include <stapl/views/proxy/accessor_traits.hpp>
#include <iterator>
#include <type_traits>

namespace stapl {

template<typename I, typename PA>
class member_iterator;

class iterator_core_access;


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Function object that advances an iterator a given number of times
///
/// @tparam Iterator Iterator type
//////////////////////////////////////////////////////////////////////
template<typename Iterator>
class advance_inner
{
  Iterator m_iter;
  typedef typename std::iterator_traits<Iterator>::difference_type diff_t;

  diff_t m_diff;

public:
  typedef Iterator result_type;

  advance_inner(Iterator const& iter, diff_t diff)
   : m_iter(iter), m_diff(diff)
  { }

  template<typename T>
  result_type operator()(T const&) const
  {
    return this->advance();
  }

  result_type advance() const
  {
    Iterator it = m_iter;
    std::advance(it, m_diff);
    return it;
  }

  void define_type(typer& t)
  {
    t.member(m_iter);
    t.member(m_diff);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object that advances an iterator a given number of times
///        and applies a higher-order function to the result of dereferencing
///        that iterator.
///
/// @tparam Iterator Iterator type
/// @tparam F Function to be applied over the dereferenced iterator
//////////////////////////////////////////////////////////////////////
template<typename Iterator, typename F>
struct advance_apply
 : private advance_inner<Iterator>, private F
{
  typedef typename std::iterator_traits<Iterator>::difference_type diff_t;

  advance_apply(F const& f, Iterator const& iter, diff_t diff)
   : advance_inner<Iterator>(iter, diff), F(f)
  { }

  template<typename T>
  void operator()(T const&) const
  {
    F::operator()(*this->advance());
  }

  void define_type(typer& t)
  {
    t.base<advance_inner<Iterator> >(*this);
    t.base<F>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object that advances an iterator a given number of times
///        and returns the result of applying a higher-order function to the
///        dereferenced value of that iterator.
///
/// @tparam Iterator Iterator type
/// @tparam F Function to be applied over the dereferenced iterator
//////////////////////////////////////////////////////////////////////
template<typename Iterator, typename F>
struct advance_apply_get
 : private advance_inner<Iterator>, private F
{
  typedef typename F::result_type result_type;
  typedef typename std::iterator_traits<Iterator>::difference_type diff_t;

  advance_apply_get(F const& f, Iterator const& iter, diff_t diff)
   : advance_inner<Iterator>(iter, diff), F(f)
  { }

  template<typename T>
  result_type operator()(T const&) const
  {
    return F::operator()(*this->advance());
  }

  void define_type(typer& t)
  {
    t.base<advance_inner<Iterator> >(*this);
    t.base<F>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines an accessor to a pointer on a remote location using an
///        iterator to reference it.
///
/// @tparam Iterator Local iterator type.
/// @tparam ParentAccessor Accessor used to forward the method invocations.
//////////////////////////////////////////////////////////////////////
template<typename Iterator, typename ParentAccessor>
class remote_pointer_accessor
  : public accessor_base<
      typename std::iterator_traits<Iterator>::value_type,
      remote_pointer_accessor<Iterator, ParentAccessor>
    >
{
public:
  typedef typename std::iterator_traits<Iterator>::value_type value_type;
  typedef typename std::iterator_traits<Iterator>::difference_type diff_t;

private:
  Iterator m_iter;
  ParentAccessor m_parent;
  diff_t m_diff;

public:

  remote_pointer_accessor(Iterator iter, ParentAccessor parent, diff_t diff)
    : m_iter(iter), m_parent(parent), m_diff(diff)
  { }

  bool is_local() const
  {
    return accessor_core_access::is_local(m_parent);
  }

  template<typename F>
  void apply_set(F const& f) const
  {
    accessor_core_access::apply_set(
      m_parent, advance_apply<Iterator, F>(f, m_iter, m_diff)
    );
  }

  template<typename F>
  typename F::result_type apply_get(F const& f) const
  {
    return accessor_core_access::apply_get(
      m_parent, advance_apply_get<Iterator, F>(f, m_iter, m_diff)
    );
  }

  void define_type(typer&)
  {
    stapl_assert(false, "This accessor should not be shipped");
  }
}; // class pointer_accessor


//////////////////////////////////////////////////////////////////////
/// @brief Defines an iterator used to reference data member elements
///        through a parent accessor.
///
/// This iterator is used to define the iterator returned for a proxy
/// over stl containers (e.g., std::vector) that are remote.
/// @tparam BaseIterator Local iterator type.
/// @tparam ParentAccessor Accessor used to forward the method invocations.
/// @todo Distance metric assumes random access iterators
//////////////////////////////////////////////////////////////////////
template <typename BaseIterator, typename ParentAccessor>
class global_member_iterator
  : public iterator_facade<
      member_iterator<BaseIterator, ParentAccessor>,
      remote_pointer_accessor<BaseIterator, ParentAccessor>,
      typename std::iterator_traits<BaseIterator>::iterator_category
    >
{
  friend class stapl::iterator_core_access;

private:
  typedef typename std::iterator_traits<BaseIterator>::difference_type diff_t;
  typedef remote_pointer_accessor<BaseIterator, ParentAccessor> accessor_type;

  BaseIterator     m_iter;
  ParentAccessor   m_parent_accessor;
  diff_t           m_diff;

public:
  global_member_iterator()
    : m_iter(), m_parent_accessor(null_reference()), m_diff(0)
  { }
    // static_assert(
    //   std::is_same<
    //     typename std::iterator_traits<BaseIterator>::iterator_category,
    //     std::random_access_iterator_tag
    //   >::value, "Iterator has to be random access iterator");
  //}

  global_member_iterator(BaseIterator iter, ParentAccessor const& accessor)
    : m_iter(iter), m_parent_accessor(accessor), m_diff(0)
  { }
  //   static_assert(
  //     std::is_same<
  //       typename std::iterator_traits<BaseIterator>::iterator_category,
  //       std::random_access_iterator_tag
  //     >::value, "Iterator has to be random access iterator");
  // }

  void define_type(typer&)
  {
    stapl_assert(false, "This iterator should not be shipped");
  }

private:
  accessor_type access() const
  {
    return accessor_type(m_iter, m_parent_accessor, m_diff);
  }

  bool equal(global_member_iterator const& rhs) const
  {
    BaseIterator true_lhs = accessor_core_access::apply_get(
      m_parent_accessor, advance_inner<BaseIterator>(m_iter, m_diff)
    );

    BaseIterator true_rhs = accessor_core_access::apply_get(
      rhs.m_parent_accessor, advance_inner<BaseIterator>(rhs.m_iter, rhs.m_diff)
    );

    return true_lhs == true_rhs;
  }

  void increment()
  {
    ++m_diff;
  }

  void decrement()
  {
    --m_diff;
  }

  void advance(diff_t n)
  {
    m_diff += n;
  }

  diff_t distance_to(global_member_iterator const& rhs) const
  {
    BaseIterator true_lhs = accessor_core_access::apply_get(
      m_parent_accessor, advance_inner<BaseIterator>(m_iter, m_diff)
    );

    BaseIterator true_rhs = accessor_core_access::apply_get(
      rhs.m_parent_accessor, advance_inner<BaseIterator>(rhs.m_iter, rhs.m_diff)
    );

    return std::distance(true_lhs, true_rhs);
  }

  bool less_than(global_member_iterator const& rhs) const
  {
    return m_iter < rhs.m_iter;
  }

  BaseIterator base()
  {
    return m_iter;
  }
}; // class global_member_iterator


//////////////////////////////////////////////////////////////////////
/// @brief Defines an accessor to local element using an iterator to
///        reference it.
///
/// @tparam Iterator Local iterator type.
//////////////////////////////////////////////////////////////////////
template<typename Iterator>
class pointer_accessor
  : public accessor_base<
      typename std::iterator_traits<Iterator>::value_type,
      pointer_accessor<Iterator>
    >
{
private:
  Iterator m_iter;
public:
  typedef typename std::iterator_traits<Iterator>::value_type value_type;

  pointer_accessor(Iterator iter)
    : m_iter(iter)
  { }

  bool is_local() const
  {
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the functor provided to the element reference by the
  /// iterator.
  ///
  /// @warning This method is only valid when the referenced element is local.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void apply_set(F const& f) const
  {
    f(*m_iter);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the functor provided to the element reference by the
  /// iterator.
  ///
  /// @warning This method is only valid when the referenced element is local.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename F::result_type apply_get(F const& f) const
  {
    return f(*m_iter);
  }

private:
  void define_type(typer&);

}; // class pointer_accessor

//////////////////////////////////////////////////////////////////////
/// @brief Defines an iterator used to reference data member elements
///        directly through a pointer.
///
/// This iterator is used to define the iterator returned for a proxy
/// over stl containers (e.g., std::vector).
/// @tparam BaseIterator Local iterator type.
/// @tparam ParentAccessor Accessor used to forward the method invocations.
/// @note This iterator is used to point to local elements.
//////////////////////////////////////////////////////////////////////
template <typename BaseIterator, typename ParentAccessor>
class localized_member_iterator
  : public iterator_facade<
      member_iterator<BaseIterator, ParentAccessor>,
      pointer_accessor<BaseIterator>,
      typename std::iterator_traits<BaseIterator>::iterator_category
    >
{
  friend class stapl::iterator_core_access;

private:
  typedef typename std::iterator_traits<BaseIterator>::difference_type diff_t;

  BaseIterator     m_iter;

public:
  localized_member_iterator()
    : m_iter()
  { }

  localized_member_iterator(BaseIterator iter, ParentAccessor const&)
    : m_iter(iter)
  { }

private:
  pointer_accessor<BaseIterator> access() const
  {
    return pointer_accessor<BaseIterator>(m_iter);
  }

  bool equal(localized_member_iterator const& rhs) const
  {
    return m_iter == rhs.m_iter;
  }

  void increment()
  {
    ++m_iter;
  }

  void decrement()
  {
    --m_iter;
  }

  void advance(diff_t n)
  {
    std::advance(m_iter, n);
  }

  diff_t distance_to(localized_member_iterator const& rhs) const
  {
    return std::distance(m_iter, rhs.m_iter);
  }

  bool less_than(localized_member_iterator const& rhs) const
  {
    return m_iter < rhs.m_iter;
  }

  BaseIterator base()
  {
    return m_iter;
  }

  void define_type(typer& t);

}; // class localized_member_iterator


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction used to compute whether to use the member_iterator
///        defined for local elements or global elements.
//////////////////////////////////////////////////////////////////////
template<typename Iterator, typename ParentAccessor>
using select_member_iterator =
  std::conditional<
    accessor_traits<ParentAccessor>::is_localized::value,
    localized_member_iterator<Iterator, ParentAccessor>,
    global_member_iterator<Iterator, ParentAccessor>>;


} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of stand-alone begin for rvalue references
/// to a proxy with an pointer_accessor.
///
/// Example:
/// auto it = stapl::begin(proxy<T, pointer_accessor<Outer>>(t));
///
/// @ingroup iteratorSpecializations
//////////////////////////////////////////////////////////////////////
template <typename T, typename Outer>
auto begin(stapl::proxy<T, detail::pointer_accessor<Outer>>&& t)
  -> decltype(
    const_cast<stapl::proxy<T, detail::pointer_accessor<Outer>> const&&>(t)
      .begin())
{
  return const_cast<stapl::proxy<T, detail::pointer_accessor<Outer>> const&&>(t)
    .begin();
}


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of stand-alone begin for lvalue references
/// to a proxy with an pointer_accessor.
///
/// Example:
/// proxy<T, pointer_accessor<Outer>> p(t);
/// auto it = stapl::begin(p);
///
/// @ingroup iteratorSpecializations
//////////////////////////////////////////////////////////////////////
template <typename T, typename Outer>
auto begin(stapl::proxy<T, detail::pointer_accessor<Outer>>& t)
  -> decltype(
    const_cast<stapl::proxy<T, detail::pointer_accessor<Outer>> const&>(t)
      .begin())
{
  return const_cast<stapl::proxy<T, detail::pointer_accessor<Outer>> const&>(t)
    .begin();
}

//////////////////////////////////////////////////////////////////////
/// @brief Defines an iterator used to reference data member elements
///        directly through a pointer.
///
/// This iterator is used to define the iterator returned for a proxy
/// over stl containers (e.g., std::vector).
/// @tparam BaseIterator Local iterator type.
/// @tparam ParentAccessor Accessor used to forward the method invocations.
/// @todo This iterator is only valid for random access iterators, from
///       containers like std::vector. This should be extended to handle
///       more generic use cases.
//////////////////////////////////////////////////////////////////////
template<typename Iterator, typename ParentAccessor>
class member_iterator
 : public detail::select_member_iterator<Iterator, ParentAccessor>::type
{
  typedef typename detail::select_member_iterator<
    Iterator, ParentAccessor
  >::type base_type;

  friend class stapl::iterator_core_access;

public:
  member_iterator(Iterator const& it, ParentAccessor const& acc)
   : base_type(it, acc)
  { }

  member_iterator(void) = default;
};

template<typename I>
struct accessor_traits<detail::pointer_accessor<I>>
{
  typedef std::true_type is_localized;
};

} // namespace stapl

#endif
