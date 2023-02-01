/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_LAZY_INSERT_VIEW_HPP
#define STAPL_VIEWS_LAZY_INSERT_VIEW_HPP

#include <stapl/views/core_view.hpp>
#include <stapl/domains/infinite.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @todo recheck the semantics of the m_flag:
/// the original container flushes when it is destroyed.
/// all copies of the container share the same pointers.
/// this makes the assumption that the original container
/// outlines the copies.  this may (or may not) be true.
//////////////////////////////////////////////////////////////////////

template <typename C>
class lazy_view_container
{
  typedef std::vector<typename container_traits<C>::value_type> local_storage_t;

public:
  typedef C                                         container_type;
  typedef typename C::value_type                    value_type;
  typedef infinite                                  domain_type;
  typedef domain_type::index_type                   index_type;
  typedef index_type                                gid_type;
  typedef value_type                                reference;

protected:
  container_type*   m_container;
  local_storage_t*  m_storage;
  bool              m_flag;

private:

  lazy_view_container(void)
  {
    stapl_assert(false,"Wrong constructor");
  }

public:

  void define_type(stapl::typer &t)
  {
    t.member(m_container);
    t.member(m_storage);
    t.member(m_flag);
  }

  lazy_view_container(lazy_view_container* other)
    : m_container(other->m_container),
      m_storage(other->m_storage),
      m_flag(false)
  { }

  lazy_view_container(lazy_view_container const& other)
    : m_container(other.m_container),
      m_storage(other.m_storage),
      m_flag(false)
  { }

  template<typename Other>
  lazy_view_container(Other* other, C* new_cont)
    : m_container(new_cont),
      m_storage(new local_storage_t()),
      m_flag(true)
  { }

  lazy_view_container(container_type* cont)
    : m_container(cont),
      m_storage(new local_storage_t()),
      m_flag(true)
  { }

  ~lazy_view_container(void)
  {
    if (m_flag) {
      this->flush();
    }
  }

  void add(value_type const& value)
  {
    m_storage->push_back(value);
  }

  void flush(void)
  {
    stapl_assert(m_container!=0,"Wrong container pointer");

    for (auto&& x : *m_storage)
      m_container->push_back(x);

    // this fence is needed because traffic generated during the
    // destruction of views is not enforced by the PARAGRAPH. see
    // gforge task 807.
    rmi_fence();
  }

  size_t version(void) const
  {
    return 0;
  }

  size_t size(void) const
  { return domain_type().size(); }

  domain_type domain(void) const
  {
    return domain_type();
  }

}; //class lazy_view_container


template <typename C>
class lazy_insert_view
  : public core_view<C, infinite, f_ident<size_t> >
{
  typedef core_view<C, infinite, f_ident<size_t> >         base_type;

  typedef view_operations::sequence<lazy_insert_view>      sequence_op_type;

public:
  STAPL_VIEW_REFLECT_TRAITS(lazy_insert_view)

  typedef detail::view_iterator<lazy_insert_view>       iterator;
  typedef detail::const_view_iterator<lazy_insert_view> const_iterator;

public:

  lazy_insert_view(const C& c)
    : base_type(c, domain_type(), f_ident<size_t>() )
  { }

  lazy_insert_view(C* c)
    : base_type(c, domain_type(), f_ident<size_t>() )
  { }

  lazy_insert_view(view_container_type* vcont, domain_type const& dom,
                   map_func_type mfunc=map_func_type())
    : base_type(vcont, dom, mfunc)
  { }

  lazy_insert_view(view_container_type const& vcont, domain_type const& dom,
                   map_func_type mfunc=map_func_type())
    : base_type(vcont, dom, mfunc)
  { }

  lazy_insert_view(lazy_insert_view const& other)
    : base_type(other)
  { }

  void add(typename C::value_type const& val)
  {
    this->container().add(val);
  }

  void post_execute(void)
  {
    this->container().flush();
  }

  size_t size(void) const
  {
    return get_num_locations();
  }

  iterator begin(void)
  {
    return iterator(this,0);
  }

  iterator end(void)
  {
    return iterator(this,1);
  }

  reference operator[](index_type idx) const
  {
    return reference(this->container());
  }

  index_type next(index_type idx) { return idx+1; }

}; //class lazy_insert_view


template<typename C>
struct view_traits<lazy_insert_view<C> >
{
  typedef lazy_insert_view<C>                value_type;
  typedef value_type                         reference;
  typedef value_type                         const_reference;
  typedef C                                  container;
  typedef size_t                             index_type;
  typedef f_ident<index_type>                map_function;
  typedef infinite                           domain_type;
};


template<typename C>
lazy_insert_view<lazy_view_container<C> >
lazy_insert(C const& c)
{
  typedef lazy_view_container<C> container_t;
  return lazy_insert_view<container_t>(new container_t(&const_cast<C&>(c)));
}

} // namespace stapl

#endif // STAPL_VIEWS_LAZY_INSERT_VIEW_HPP
