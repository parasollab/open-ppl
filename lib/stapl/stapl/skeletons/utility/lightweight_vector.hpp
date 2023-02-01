/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_LIGHTWEIGHT_VECTOR_HPP
#define STAPL_SKELETONS_UTILITY_LIGHTWEIGHT_VECTOR_HPP

#include <stapl/views/proxy.h>
#include <stapl/views/proxy/stl_vector_accessor.hpp>
#include <stapl/views/view_traits.hpp>
#include <stapl/skeletons/utility/lightweight_multiarray_base.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>

namespace stapl {


//////////////////////////////////////////////////////////////////////
/// @brief This container is lightweight wrapper around std::vector
/// and keeps a @c shared_ptr to the underlying std::vector.
///
/// This container is used in the skeletons in which copying large
/// vectors would result in performance degradation. As an example,
/// lightweight_vector is used in the NAS IS benchmark in order to
/// avoid intermediate copies made by the PARAGRAPH for filtering
/// the results of each task.
///
/// @tparam T the type of elements to store
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <typename T>
struct lightweight_vector
  : public lightweight_multiarray_base<T, 1>
{
private:
  using base_t          = lightweight_multiarray_base<T, 1>;
public:
  using reference       = typename base_t::reference;
  using const_reference = typename base_t::const_reference;
  using iterator        = typename base_t::iterator;
  using const_iterator  = typename base_t::const_iterator;
  using domain_type     = typename base_t::domain_type;
  using size_type       = std::size_t;
  using value_type      = T;

  lightweight_vector()
    : base_t()
  { }

  lightweight_vector(std::size_t size)
    : base_t(size)
  { }

  lightweight_vector(domain_type const& dom, value_type const& default_value)
    : base_t(dom, default_value)
  { }

  lightweight_vector(iterator begin, iterator end)
    : base_t(begin, end)
  { }

  lightweight_vector(const_iterator begin, const_iterator end)
    : base_t(begin, end)
  { }

  ~lightweight_vector() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an const_iterator to the beginning of
  /// the sequence of elements this container stores.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return this->m_content->begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an const_iterator to the beginning of
  /// the sequence of elements this container stores.
  //////////////////////////////////////////////////////////////////////
  const_iterator cbegin(void) const
  {
    return this->m_content->cbegin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the beginning of
  /// the sequence of elements this container stores.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return this->m_content->begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an const_iterator to the end (one past the last element) of
  /// the sequence of elements this container stores.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return this->m_content->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an const_iterator to the end (one past the last element) of
  /// the sequence of elements this container stores.
  //////////////////////////////////////////////////////////////////////
  const_iterator cend(void) const
  {
    return this->m_content->cend();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the end (one past the last element) of
  /// the sequence of elements this container stores.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return this->m_content->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return reference to element in buffer at index @p idx.
  //////////////////////////////////////////////////////////////////////
  reference operator[](std::size_t const& idx)
  {
    return this->m_content->operator[](idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return reference to element in buffer at index @p idx.
  //////////////////////////////////////////////////////////////////////
  const_reference operator[](std::size_t const& idx) const
  {
    return this->m_content->operator[](idx);
  }

  bool operator==(lightweight_vector<T> const& other) const
  {
    return (this->m_content == other.m_content);
  }

  void push_back(T const& t)
  {
    this->m_content->push_back(t);
  }

  void emplace_back(T&& t)
  {
    this->m_content->emplace_back(std::move(t));
  }

  void reserve(std::size_t size)
  {
    this->m_content->reserve(size);
    this->resize_domain(size);
  }

  void resize(std::size_t size)
  {
    this->m_content->resize(size);
    this->resize_domain(size);
  }

  size_type size() const
  {
    return this->m_content->size();
  }

  bool empty() const
  {
    return base_t::empty();
  }

  const_reference front(void) const
  {
    return this->m_content->front();
  }

  reference front(void)
  {
    return this->m_content->front();
  }

  template <class InputIterator>
  void insert(iterator position, InputIterator first, InputIterator last)
  {
    this->m_content->insert(position,first,last);
  }

  void insert(iterator position, size_type n, value_type const& val)
  {
    this->m_content->insert(position,n,val);
  }

  iterator insert(iterator position, value_type const& val)
  {
    return this->m_content->insert(position,val);
  }

  typename base_t::content_t* get()
  {
    return this->m_content.get();
  }

  void clear()
  {
    this->m_content->clear();
  }

  template <class InputIterator>
  void assign(InputIterator first, InputIterator last)
  {
    this->m_content->assign(first,last);
  }

  void assign(size_type n, value_type const& val)
  {
    this->m_content->assign(n,val);
  }

  domain_type domain(void) const
  {
    return domain_type(this->size());
  }

  void define_type(typer& t)
  {
    t.base<base_t>(*this);
  }
};


STAPL_PROXY_HEADER_TEMPLATE(lightweight_vector, T)
{
  STAPL_PROXY_DEFINES(lightweight_vector<T>)
  STAPL_PROXY_REFLECT_TYPE(size_type)
  STAPL_PROXY_REFLECT_TYPE(value_type)
  STAPL_PROXY_REFLECT_TYPE(domain_type)

  void push_back(T const& val) const
  {
    typedef void (lightweight_vector<T>::* mem_fun_t)(T const&);

    const mem_fun_t mem_fun = &lightweight_vector<T>::push_back;

    Accessor::invoke(mem_fun, val);
  }

  void resize(std::size_t size)
  {
    typedef void (lightweight_vector<T>::* mem_fun_t)(std::size_t);

    const mem_fun_t mem_fun = &lightweight_vector<T>::resize;

    Accessor::invoke(mem_fun, size);
  }

  domain_type domain(void) const
  {
    return Accessor::const_invoke(&target_t::domain);
  }

  STAPL_PROXY_METHOD_RETURN(size, size_type)
  STAPL_PROXY_REFERENCE_METHOD_1(inner, operator[], T, std::size_t)

  typedef typename target_t::iterator                  iter_t;
  typedef typename target_t::const_iterator            const_iter_t;

  typedef member_iterator<iter_t, Accessor>            iterator;
  typedef member_iterator<const_iter_t, Accessor>      const_iterator;
  typedef inner_reference                              reference;

  const_iterator begin() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::begin), *this);
  }

  const_iterator cbegin() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::cbegin), *this);
  }

  iterator begin()
  {
    return iterator(Accessor::invoke(&target_t::begin), *this);
  }

  const_iterator end() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::end), *this);
  }

  const_iterator cend() const
  {
    return const_iterator(Accessor::const_invoke(&target_t::cend), *this);
  }

  iterator end()
  {
    return iterator(Accessor::invoke(&target_t::end), *this);
  }

}; // struct proxy

template <typename T, typename C>
class proxy<lightweight_vector<T>, local_accessor<C>>
  : public local_accessor<C>
{
  using Accessor = local_accessor<C>;

  STAPL_PROXY_DEFINES(lightweight_vector<T>)
  STAPL_PROXY_REFLECT_TYPE(size_type)
  STAPL_PROXY_REFLECT_TYPE(value_type)
  STAPL_PROXY_REFLECT_TYPE(domain_type)

  void push_back(T const& val) const
  {
    typedef void (lightweight_vector<T>::* mem_fun_t)(T const&);

    const mem_fun_t mem_fun = &lightweight_vector<T>::push_back;

    Accessor::invoke(mem_fun, val);
  }

  void resize(std::size_t size)
  {
    typedef void (lightweight_vector<T>::* mem_fun_t)(std::size_t);

    const mem_fun_t mem_fun = &lightweight_vector<T>::resize;

    Accessor::invoke(mem_fun, size);
  }

  STAPL_PROXY_METHOD_RETURN(size, size_type)
  STAPL_PROXY_METHOD_RETURN(domain, domain_type)

  typedef typename target_t::iterator                  iterator;
  typedef typename target_t::const_iterator            const_iterator;

  typedef typename target_t::reference                 reference;
  typedef typename target_t::const_reference           const_reference;

  const_iterator begin() const
  {
    return Accessor::const_invoke(&target_t::begin);
  }

  iterator begin()
  {
    return Accessor::invoke(&target_t::begin);
  }

  const_iterator end() const
  {
    return Accessor::const_invoke(&target_t::end);
  }

  iterator end()
  {
    return Accessor::invoke(&target_t::end);
  }

  reference operator[](std::size_t const& idx)
  {
    return Accessor::invoke(&target_t::operator[], idx);
  }

  const_reference operator[](std::size_t const& idx) const
  {
    return Accessor::const_invoke(&target_t::operator[], idx);
  }

}; // struct proxy


template <typename C>
using lightweight_vector_accessor = stl_vector_accessor<C>;

template <typename C>
using lightweight_vector_const_accessor = stl_vector_const_accessor<C>;

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for lightweight_multiarray_base
///
/// @see container_traits
//////////////////////////////////////////////////////////////////////
template<typename T>
struct container_traits<lightweight_vector<T>>
{
  using gid_type        = size_t;
  using value_type      = T;
  using domain_type     = indexed_domain<gid_type>;
  using container_type  = lightweight_vector<T>;
  using iterator        = typename container_type::iterator;
  using reference       = typename container_type::reference;
  using const_reference = typename container_type::const_reference;
};

template<typename T, bool a, bool b>
struct extract_reference_type<lightweight_vector<T>, a, b>
{
  using type = proxy<T, lightweight_vector_accessor<lightweight_vector<T>>>;
};

template<typename T, bool a, bool b>
struct extract_const_reference_type<lightweight_vector<T>, a, b>
{
  using type = proxy<
                 T, lightweight_vector_const_accessor<lightweight_vector<T>>>;
};

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of make_referene for the cases that the
/// underlying container is a proxy of a lightweight_vector
//////////////////////////////////////////////////////////////////////
template<typename T, typename A>
struct container_make_reference<proxy<lightweight_vector<T>, A>>
{
  template<typename View>
  static
  typename view_traits<View>::reference
  apply(View const& view, typename view_traits<View>::index_type index)
  {
    return view.container()[index];
  }
};

} // namespace detail
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_LIGHTWEIGHT_VECTOR_HPP
