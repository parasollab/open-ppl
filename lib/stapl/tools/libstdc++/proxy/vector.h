/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROXY_VECTOR_H
#define STAPL_PROXY_VECTOR_H

#include <stapl/views/proxy/proxy.hpp>
#include <stapl/views/iterator/member_iterator_fwd.h>
#include <stapl/views/iterator/iterator_core_access.h>
#include <stapl/views/proxy_macros.hpp>

#include <stapl/paragraph/edge_container/views/edge_accessor_fwd.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @note The value_type* data() method is not implemented because it
/// requires returning a pointer to data which can be remote.
/// @note The void swap(vector&) method is not supported because it does
/// not provide the same O(1) guarantees as the stl version. This is
/// because in sequential swap simply swap pointers, whereas in
/// a parallel environment swap requires copying everything over to
/// different address spaces resulting in an O(n) operation.
///
/// @todo Make reflected const_reference type different from reference
///   when const_member_accessor and associated helper macros exist.
//////////////////////////////////////////////////////////////////////
STAPL_PROXY_HEADER_TEMPLATE(std::vector, T)
{
  STAPL_PROXY_DEFINES(std::vector<T>)
  STAPL_PROXY_REFLECT_TYPE(size_type)

  STAPL_PROXY_METHOD(pop_back)
  STAPL_PROXY_METHOD(clear)
  STAPL_PROXY_METHOD(shrink_to_fit)

  STAPL_PROXY_METHOD_1(reserve, size_type)
  STAPL_PROXY_METHOD_2(assign, size_type, T)

  STAPL_PROXY_METHOD_RETURN(size, size_type)
  STAPL_PROXY_METHOD_RETURN(max_size, size_type)
  STAPL_PROXY_METHOD_RETURN(capacity, size_type)
  STAPL_PROXY_METHOD_RETURN(empty, bool)

  STAPL_PROXY_REFERENCE_METHOD_0(front, T)
  STAPL_PROXY_REFERENCE_METHOD_0(back, T)
  STAPL_PROXY_REFERENCE_METHOD_1(inner, operator[], T, std::size_t)
  STAPL_PROXY_REFERENCE_METHOD_1(inner2, at, T, std::size_t)

private:
  typedef typename target_t::iterator                  iter_t;
  typedef typename target_t::const_iterator            const_iter_t;

  typedef typename target_t::reverse_iterator          r_iter_t;
  typedef typename target_t::const_reverse_iterator    const_r_iter_t;

public:
  typedef member_iterator<iter_t, Accessor>            iterator;
  typedef member_iterator<const_iter_t, Accessor>      const_iterator;

  typedef member_iterator<r_iter_t, Accessor>          r_iterator;
  typedef member_iterator<const_r_iter_t, Accessor>    const_r_iterator;

  typedef inner_reference                              reference;
  typedef inner_reference                              const_reference;

  iterator begin()
  {
    return iterator(Accessor::invoke(&target_t::begin), *this);
  }

  iterator end()
  {
    return iterator(Accessor::invoke(&target_t::end), *this);
  }

  r_iterator rbegin()
  {
    return r_iterator(Accessor::invoke(&target_t::rbegin), *this);
  }

  r_iterator rend()
  {
    return r_iterator(Accessor::invoke(&target_t::rend), *this);
  }

  const_iterator cbegin() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::cbegin), *this);
  }

  const_iterator cend() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::cend), *this);
  }

  const_r_iterator crbegin() const
  {
    return const_r_iterator(Accessor::const_invoke(&target_t::crbegin), *this);
  }

  const_r_iterator crend() const
  {
    return const_r_iterator(Accessor::const_invoke(&target_t::crend), *this);
  }

  void resize(size_t size)
  {
    typedef void (std::vector<T>::* mem_fun_t)(size_t);

    const mem_fun_t mem_fun = &std::vector<T>::resize;

    Accessor::invoke(mem_fun, size);
  }

  void resize(size_t size, T const& val)
  {
    typedef void (std::vector<T>::* mem_fun_t)(size_t, T const&);

    const mem_fun_t mem_fun = &std::vector<T>::resize;

    Accessor::invoke(mem_fun, size, val);
  }

  void assign(iterator first, iterator last)
  {
    typedef void (target_t::* mem_fun_t)(iter_t, iter_t);
    const mem_fun_t mem_fun = &target_t::assign;

    Accessor::invoke(mem_fun, iterator_core_access::base(first),
      iterator_core_access::base(last));
  }

  void push_back(T const& val) const
  {
    typedef void (std::vector<T>::* mem_fun_t)(T const&);

    const mem_fun_t mem_fun = &std::vector<T>::push_back;

    Accessor::invoke(mem_fun, val);
  }

  iterator insert(iterator iter_position, T const& val)
  {
    typedef iter_t (target_t::* mem_fun_t)(iter_t, T const&);
    const mem_fun_t mem_fun = &target_t::insert;

    return iterator(Accessor::invoke(mem_fun,
      iterator_core_access::base(iter_position) , val), *this);
  }

  void insert(iterator iter_position, size_type n, T const& val)
  {
    typedef void (target_t::* mem_fun_t)(iter_t, size_type, T const&);
    const mem_fun_t mem_fun = &target_t::insert;

    Accessor::invoke(mem_fun, iterator_core_access::base(iter_position), n,
      val);
  }

  void insert(iterator iter_position, iterator first, iterator last)
  {
    typedef void (target_t::* mem_fun_t)(iter_t, iter_t, iter_t);
    const mem_fun_t mem_fun = &target_t::insert;

    Accessor::invoke(mem_fun,
      iterator_core_access::base(iter_position),
      iterator_core_access::base(first),
      iterator_core_access::base(last));
  }

  iterator erase(iterator iter_position)
  {
    typedef iter_t (target_t::* mem_fun_t)(iter_t);
    const mem_fun_t mem_fun = &target_t::erase;

    return iterator(Accessor::invoke(mem_fun,
      iterator_core_access::base(iter_position)), *this);
  }

  iterator erase(iterator first, iterator last)
  {
    typedef iter_t (target_t::* mem_fun_t)(iter_t, iter_t);
    const mem_fun_t mem_fun = &target_t::erase;

    return iterator(Accessor::invoke(mem_fun,
                      iterator_core_access::base(first),
                      iterator_core_access::base(last)),
                   *this);
  }

  template <class... Args>
  iterator emplace(const_iterator iter, Args&&... args)
  {
    typedef iter_t (target_t::* mem_fun_t)(const_iter_t, Args&&...);
    const mem_fun_t mem_fun = &target_t::emplace;

    return iterator(Accessor::invoke(mem_fun,
      iterator_core_access::base(iter), std::forward<Args>(args)...));
  }

  template <class... Args>
  void emplace_back(Args&&... args)
  {
    typedef void (target_t::* mem_fun_t)(Args&&...);
    const mem_fun_t mem_fun = &target_t::emplace_back;

    Accessor::invoke(mem_fun, std::forward<Args>(args)...);
  }

}; // struct proxy


template <typename T, typename View>
class proxy<std::vector<T>, edge_accessor<View> >
  : public edge_accessor<View>
{
private:
  typedef edge_accessor<View> Accessor;
  STAPL_PROXY_DEFINES(std::vector<T>)
  STAPL_PROXY_REFLECT_TYPE(size_type)
  STAPL_PROXY_METHOD_RETURN(size, size_type)
  STAPL_PROXY_REFERENCE_METHOD_1(inner, operator[], T, std::size_t)
  STAPL_PROXY_REFERENCE_METHOD_0(back, T)

  typedef typename target_t::const_iterator            iter_t;
  typedef typename target_t::const_iterator            const_iter_t;

  typedef member_iterator<const_iter_t, Accessor>      iterator;
  typedef member_iterator<const_iter_t, Accessor>      const_iterator;
  typedef inner_reference                              reference;
  typedef inner_reference const                        const_reference;

  typedef T                                            value_type;

  const_iterator begin() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::begin), *this);
  }

  const_iterator end() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::end), *this);
  }
}; // struct proxy


} // namespace stapl

#endif

