/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ITERATOR_DOMAIN_HPP
#define STAPL_ITERATOR_DOMAIN_HPP

#include <boost/type_traits/is_same.hpp>
#include <boost/type_traits/is_signed.hpp>

#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/containers/iterators/container_iterator.hpp>
#include <stapl/containers/iterators/const_container_iterator.hpp>
#include <stapl/containers/iterators/gid_of_fwd.hpp>

#include <stapl/runtime.hpp>

namespace stapl {

namespace domain_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to get the gid associated with a given iterator.
//////////////////////////////////////////////////////////////////////
template<typename Ret>
struct f_deref
{
  template<typename It>
  Ret operator()(It it) const
  {
    return *it;
  }
};

template<typename Ret>
struct f_deref_gid
{
  template<typename It>
  Ret operator()(It it) const
  {
    return gid_of(it);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines a domain represented by two iterators, where all
///        the elements between these two iterators belong to the
///        domain.
/// @tparam Container Container which the iterators are coming from.
/// @tparam Functor Functor type used to extract the index from the
///                 pointed element.
//////////////////////////////////////////////////////////////////////
template <typename Container,
          typename Functor =
            f_deref<typename container_traits<Container>::gid_type> >
class core_iteratorDom
{
public:
  typedef typename container_traits<Container>::gid_type index_type;
  typedef std::size_t                                    size_type;

private:
  /// @todo Not all containers have a @c const_iterator.
  typedef typename Container::iterator const_iterator;

  pointer_wrapper<const Container> m_container;

  const_iterator                   m_first;
  const_iterator                   m_last;

  bool                             m_dynamic;

  size_type                        m_size;
  Functor                          m_functor;
  index_type                       m_end;

public:
  core_iteratorDom(void)
    : m_first(), m_last(),
      m_dynamic(true),
      m_size(index_bounds<size_type>::invalid()),
      m_end()
  { }

  explicit
  core_iteratorDom(Container const& cont,
                   Functor const& func = Functor(),
                   index_type const& end = index_type())
    : m_container(&cont),
      m_first(), m_last(),
      m_dynamic(true),
      m_size(index_bounds<size_type>::invalid()),
      m_functor(func),
      m_end(end)
  { }


  core_iteratorDom(index_type const& index1,
                   index_type const& index2)
    : m_first(index1), m_last(index2),
      m_dynamic(false),
      m_size(index_bounds<size_type>::invalid()),
      m_end()
  { }


  core_iteratorDom(index_type const& index1,
                   index_type const& index2,
                   Container const& cont,
                   size_type size = index_bounds<size_type>::invalid())
    : m_container(&cont),
      m_first(this->container().find(index1)),
      m_last(this->container().find(index2)),
      m_dynamic(false),
      m_size(size),
      m_end()
  { }


  core_iteratorDom(index_type const& index1,
                   index_type const& index2,
                   index_type const& end,
                   Container const& c)
    : m_container(&c),
      m_first(this->container().find(index1)),
      m_last(this->container().find(index2)),
      m_dynamic(false),
      m_size(index_bounds<size_type>::invalid()),
      m_end(end)
  { }


  core_iteratorDom(index_type const& index1,
                   index_type const& index2,
                   core_iteratorDom const& other)
    : m_container(other.m_container),
      m_first(this->container().find(index1)),
      m_last(this->container().find(index2)),
      m_dynamic(false),
      m_size(index_bounds<size_type>::invalid()),
      m_functor(other.m_functor),
      m_end(other.m_end)
  {
    if (m_first == other.m_first && m_last == other.m_last)
    {
      m_size = other.m_size;
      m_dynamic = other.m_dynamic;
    }
  }

  bool container_null(void) const
  {
    return m_container == nullptr;
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Propagate constness and then add const to reference return type.
  //////////////////////////////////////////////////////////////////////
  Container /*const*/& container(void) const
  {
    stapl_assert(m_container, "Container is not set");
    return const_cast<Container&>(*m_container);
  }

  index_type first(void) const
  {
    if (m_dynamic)
    {
      const_iterator tmp = this->container().begin();
      return m_functor(tmp);
    }
    return m_functor(m_first);
  }


  index_type last(void) const
  {
    return last_helper(
      typename std::is_same<
        typename std::iterator_traits<const_iterator>::iterator_category,
        std::forward_iterator_tag
      >::type()
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::open_last
  //////////////////////////////////////////////////////////////////////
  index_type open_last(void) const
  {
    if (m_dynamic)
      return index_bounds<index_type>::invalid();

    if (empty())
      return m_functor(m_last);

    return advance(m_functor(m_last), 1);
  }


  size_type size(void) const
  {
    if (this->container_null())
      return 0;

    if (m_dynamic)
      return this->container().size();

    if (m_size != index_bounds<size_type>::invalid())
      return m_size;

    return std::distance(m_first, m_last) + 1;
  }


  bool contains(index_type const& index) const
  {
    if (m_dynamic)
      return this->container().find(index) != this->container().end();

    if ((first() == index) || (last() == index))
        return true;

    // else
    const_iterator tmp = m_first;
    while (tmp++ != m_last)
    {
      if (m_functor(tmp) == index)
        return true;
    }

    return false;
  }


  bool empty(void) const
  {
    return size() == 0;
  }


  template<typename Distance>
  index_type advance(index_type const& index, Distance n) const
  {
    typedef typename std::iterator_traits<const_iterator>::iterator_category
      category_type;

    typedef std::integral_constant<
       bool,
       std::is_same<category_type, std::forward_iterator_tag>::value
         && std::is_signed<Distance>::value
    > helper_type;

    return advance_helper(index, n, helper_type());
  }


  size_type distance(index_type const& a, index_type const& b) const
  {
    auto dist = std::distance(
      this->container().find(a), this->container().find(b));
    return (dist < 0) ? -dist : dist;
  }


  bool is_same_container_domain(void) const
  {
    return m_dynamic;
  }

  template<typename ODom>
  core_iteratorDom operator&(ODom const& other) const
  {
    if (empty() || other.empty())
      return core_iteratorDom();

    index_type olower = other.first();
    index_type oupper = other.last();

    bool contains_first = other.contains(first());
    bool contains_last = other.contains(last());

    if (contains_first && contains_last)
      return *this;

    if (!contains_first && !contains_last)
      return core_iteratorDom();

    if (contains_first)
      olower = first();

    if (contains_last)
      oupper = last();

    return core_iteratorDom(olower,oupper, *this);
  }

  void define_type(typer& t)
  {
    t.member(m_container);

    if (!m_dynamic)
    {
      t.member(m_first);
      t.member(m_last);
    }

    t.member(m_dynamic);
    t.member(m_size);
    t.member(m_functor);
    t.member(m_end);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for iterators that are at least bi-directional.
  //////////////////////////////////////////////////////////////////////
  index_type last_helper(std::false_type) const
  {
    if (is_same_container_domain())
    {
      const_iterator last = this->container().end();

      if (this->container().size() > 0)
        --last;

      return m_functor(last);
    }

    return m_functor(m_last);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for forward iterators.
  //////////////////////////////////////////////////////////////////////
  index_type last_helper(std::true_type) const
  {
    if (is_same_container_domain())
    {
      const_iterator tmp_first = this->container().begin();
      std::advance(tmp_first, (this->container().size()-1));
      return m_functor(tmp_first);
    }

    return m_functor(m_last);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for iterators that are at least bi-directional
  ///        or forward iterators and unsigned @p Distance types.
  //////////////////////////////////////////////////////////////////////
  template<typename Distance>
  index_type advance_helper(index_type const& index,
                            Distance n,
                            std::false_type) const
  {
    const_iterator it_index = this->container().find(index);
    std::advance(it_index, n);

    if (it_index == this->container().end())
      return index_bounds<index_type>::invalid();

    return m_functor(it_index);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for forward iterators and signed @p Distance types.
  //////////////////////////////////////////////////////////////////////
  template<typename Distance>
  index_type advance_helper(index_type const& index,
                            Distance n,
                            std::true_type) const
  {
    const_iterator it_index = this->container().find(index);

    if (n < 0)
    {
      const_iterator it_tmp = this->container().begin();
      size_type sz = std::distance(it_tmp, it_index);
      sz += n;
      std::advance(it_tmp, sz);
      return m_functor(it_tmp);
    }

    std::advance(it_index, n);

    if (it_index == this->container().end())
      return index_bounds<index_type>::invalid();

    return m_functor(it_index);
  }
}; // class core_iteratorDom


template<typename C, typename Index>
std::ostream& operator<<(std::ostream& os, core_iteratorDom<C, Index> const& d)
{
  if (d.empty())
    return os << "empty";

  return os << "[" << d.first() << ".." << d.last() << "]";
}

} // namespace domain_impl


//////////////////////////////////////////////////////////////////////
/// @brief Domains based on iterators.
//////////////////////////////////////////////////////////////////////
template<typename Container,
         typename Functor =
           domain_impl::f_deref<
             typename container_traits<Container>::gid_type
           >
        >
class iterator_domain
  : public domain_impl::core_iteratorDom<Container, Functor>
{
public:
  typedef domain_impl::core_iteratorDom<Container, Functor> k_dom_type;

private:
  typedef k_dom_type                                        base_type;

public:
  typedef typename base_type::index_type                    index_type;
  typedef index_type                                        gid_type;
  typedef std::size_t                                       size_type;

  /// Functor used to select the field used as index_type.
  typedef Functor                                           field_selector;

  iterator_domain(void)
    : base_type(k_dom_type())
  { }

  explicit iterator_domain(k_dom_type const& kdom)
    : base_type(kdom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @todo Make explicit when the unordered_map does not expect an implicit
  ///       conversion.
  //////////////////////////////////////////////////////////////////////
  /*explicit*/
  iterator_domain(iterator_domain const& other)
    : base_type(other)
  { }

  template<typename OtherContainer>
  iterator_domain(iterator_domain<OtherContainer, Functor> const& other,
                  Container const& cont)
    : base_type(k_dom_type(other.first(), other.last(), cont, other.size()))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @todo Required by graph_metadata.
  //////////////////////////////////////////////////////////////////////
  template<typename OtherContainer>
  iterator_domain(iterator_domain<OtherContainer, Functor> const& other,
                  Container const* cont)
    : base_type(k_dom_type(other.first(), other.last(), *cont, other.size()))
  { }

  explicit iterator_domain(Container const& cont,
                           Functor const& func = Functor())
    : base_type(k_dom_type(cont, func))
  { }

  iterator_domain(Container const& cont,
                  Functor const& func,
                  index_type const& end)
    : base_type(k_dom_type(cont, func, end))
  { }

  iterator_domain(index_type const& index1,
                  index_type const& index2,
                  Container const& cont,
                  size_type size = index_bounds<size_type>::invalid())
    : base_type(k_dom_type(index1, index2, cont, size))
  { }

  iterator_domain(index_type const& index1,
                  index_type const& index2,
                  index_type const& end,
                  Container const& c)
    : base_type(k_dom_type(index1, index2, end, c))
  { }

  iterator_domain(index_type const& index1,
                  index_type const& index2)
    : base_type(k_dom_type(index1, index2))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Restrict domain constructor.
  //////////////////////////////////////////////////////////////////////
  iterator_domain(index_type const& index1,
                  index_type const& index2,
                  iterator_domain const& other)
    : base_type(k_dom_type(index1, index2, other))
  { }

  index_type open_last(void) const
  {
    return base_type::open_last();
  }

  iterator_domain operator&(iterator_domain const& other)
  {
    return iterator_domain(
      static_cast<base_type&>(*this) & static_cast<base_type const&>(other));
  }

  template<typename ODom>
  iterator_domain operator&(ODom const& other)
  {
    return iterator_domain(
      static_cast<base_type&>(*this) & other);
  }

#ifdef _STAPL
  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
#endif
}; // class iterator_domain

} // namespace stapl

#endif // STAPL_ITERATOR_DOMAIN_HPP
