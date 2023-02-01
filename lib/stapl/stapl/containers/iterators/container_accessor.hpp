/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONTAINER_ACCESSOR_HPP
#define STAPL_CONTAINERS_CONTAINER_ACCESSOR_HPP

#include <stapl/views/proxy/accessor.hpp>
#include <stapl/views/proxy/accessor_base.hpp>
#include <stapl/views/view_packing.hpp>

#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/containers/type_traits/container_wrapper_ref.hpp>

#include <utility>
#include <boost/utility/result_of.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor for container_accessor::apply_get which
///   facilitates asynchronous return of @p Functor invocation via
///   a promise to the initiating location.  Properly packages views
///   for serialization if necessary.
//////////////////////////////////////////////////////////////////////
template<typename Functor>
class apply_get_wrapper
  : private Functor
{
private:
  using transfer_type =
    typename boost::result_of<
      transporter_packager(typename Functor::result_type)>::type;

public:
  using promise_type = promise<transfer_type>;

private:
  mutable promise_type m_promise;

  Functor const& functor(void) const
  {
    return static_cast<Functor const&>(*this);
  }

public:
  typedef void result_type;

  apply_get_wrapper() = default;

  template<typename F>
  apply_get_wrapper(F&& f, promise_type p)
    : Functor(std::forward<F>(f)), m_promise(std::move(p))
  { }

  template<typename T>
  void operator()(T&& t) const
  {
    if (std::is_same<typename Functor::result_type, transfer_type>::value)
    {
      auto v = this->functor()(std::forward<T>(t));
      m_promise.set_value(std::move(transporter_packager()(v)));
    }
    else
    {
      gang g;
      auto v = this->functor()(std::forward<T>(t));
      m_promise.set_value(std::move(transporter_packager()(v)));
      rmi_fence();
    }
  }

  void define_type(typer& t)
  {
    t.base<Functor>(*this);
    t.member(m_promise);
  }
}; // class apply_get_wrapper


//////////////////////////////////////////////////////////////////////
/// @brief Caches a pointer to the value referred to by a
///   @ref container_accessor when the value type is a nested container.
///   Otherwise, it's an empty class.
///
///  Primary template matches case when @p T is not a container.  One
///  specialization covers case when it is.  The class is an optimization
///  to avoid routing all requests to a nested container through the
///  single location in the outer container which stores this element
///  in its container manager.  Requests are instead routed to the
///  local representative of the nested @ref p_object, if present,
///  avoiding unnecessary communication.
//////////////////////////////////////////////////////////////////////
template<typename T, bool = is_container<T>::value>
class element_ref_cache
{
public:
  element_ref_cache(void) = default;

  template<typename Container, typename... Indices>
  element_ref_cache(Container*, Indices const&...)
  { }

  bool element_pointer_valid(void) const
  {
    return false;
  }

  T& element(void) const
  {
    abort("attempted to access element_ref_cache on non container value type");
    T* tmp = nullptr;
    return *tmp;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor calling @p get_rmi_handle on any object passed to it.
///
/// Used in @ref element_ref_cache constructor instead of lambda to reduce
/// compiler pressure.
//////////////////////////////////////////////////////////////////////
struct return_handle
{
  using result_type = rmi_handle::reference;

  template<typename T>
  result_type
  operator()(T& t) const
  { return t.get_rmi_handle(); }

  template<typename T>
  result_type operator()(container_wrapper_ref<T>& wrapper) const
  { return wrapper.get().get_rmi_handle(); }
}; // struct return_handle


template<typename T>
class element_ref_cache<T, true>
{
private:
  rmi_handle::reference  m_handle_ref;
  T*                     m_element_ptr;

public:
  element_ref_cache(void)
    : m_element_ptr(nullptr)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the cache with an attempt to resolve the pointer
  ///        to the element unconditionally.
  /// @param ct_ptr pointer to the outer container
  /// @param index index of the outer container element that will be
  ///        accessed by the accessor instance.
  ///
  /// @todo Replace unconditional call to apply_set in the initialization
  ///       of m_handle_ref with an asynchronous implementation that fires
  ///       the request to retrieve the handle at construction and then
  ///       doesn't wait on the handle to be returned until
  ///       @ref element_pointer_valid() is called.
  //////////////////////////////////////////////////////////////////////
  template<typename Container, typename Index>
  element_ref_cache(Container* ct_ptr, Index const& index)
    : m_handle_ref(ct_ptr->apply_get(index, return_handle())),
      m_element_ptr(resolve_handle<T>(m_handle_ref))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the cache with an attempt to resolve the pointer
  ///        to the element unconditionally.
  /// @param ct_ptr pointer to the outer container
  /// @param i Indices of the outer container element that will be
  ///        accessed by the accessor instance.
  //////////////////////////////////////////////////////////////////////
  template<typename Container, typename... Indices>
  element_ref_cache(Container* ct_ptr, Indices const&... i)
    : m_handle_ref(ct_ptr->apply_get(std::forward_as_tuple(i...),
                   return_handle())),
      m_element_ptr(resolve_handle<T>(m_handle_ref))
  { }

  bool element_pointer_valid(void) const
  {
    return m_element_ptr != nullptr;
  }

  T& element(void) const
  {
    stapl_assert(m_element_ptr != nullptr, "invalid dereference of nullptr");
    return *m_element_ptr;
  }

  void define_type(typer& t)
  {
    t.member(m_handle_ref);
    t.transient(m_element_ptr, resolve_handle<T>(m_handle_ref));
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Accessor for global proxies over pContainers.
/// @tparam Container Type of the container
/// @see proxy
//////////////////////////////////////////////////////////////////////
template <typename Container>
class container_accessor
  : public detail::element_ref_cache<
      typename container_traits<Container>::value_type>
{
private:
  template <typename Derived, typename A, typename C, typename D>
  friend class iterator_facade;

  friend class accessor_core_access;

public:
  //////////////////////////////////////////////////////////////////////
  /// @todo Propagate constness to container methods, and const qualifier
  ///   on reference return type.
  //////////////////////////////////////////////////////////////////////
  Container& container(void) const
  {
    stapl_assert(m_container != 0, "Container is not set");
    return *m_container;
  }

  rmi_handle::reference const& handle(void) const
  {
    return m_container.handle();
  }

  bool container_pointer_valid(void) const
  {
    return m_container != 0;
  }

public:
  typedef Container                                         container_type;
  typedef typename container_traits<Container>::gid_type    index_type;
  typedef typename container_traits<Container>::value_type  value_type;

protected: // private:
  p_object_pointer_wrapper<Container>                       m_container;
  index_type                                                m_index;

private:
  typedef detail::element_ref_cache<value_type>             base_type;

public:
  using base_type::element;

  template<typename Class, typename... Args>
  void invoke(void (Class::* const memberFuncPtr)(Args...),
              typename std::decay<Args>::type const&... args) const

  {
    this->apply_set(detail::apply_set_helper<
                      void (Class::*)(Args...),
                      typename std::decay<Args>::type...
                    >(memberFuncPtr, args...));
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn invoke(Rtn (Class::* const memberFuncPtr)(Args...),
             typename std::decay<Args>::type const&... args) const
  {
    return this->apply_get(detail::apply_get_helper<
                             Rtn (Class::*)(Args...),
                             typename std::decay<Args>::type...
                           >(memberFuncPtr, args...));
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(Args...) const,
                   typename std::decay<Args>::type const&... args) const

  {
    return this->apply_get(detail::const_apply_get_helper<
                             Rtn (Class::*)(Args...) const,
                             typename std::decay<Args>::type...
                           >(memberFuncPtr, args...));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not this is a null accessor
  //////////////////////////////////////////////////////////////////////
  bool is_null() const
  {
    if (m_index != index_bounds<index_type>::invalid())
      return false;

    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Serialization for the accessor
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_container);
    t.member(m_index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an accessor for a null reference
  //////////////////////////////////////////////////////////////////////
  container_accessor()
    : m_container(nullptr), m_index(index_bounds<index_type>::invalid())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an accessor for a null reference
  //////////////////////////////////////////////////////////////////////
  container_accessor(null_reference const&)
    : m_container(nullptr), m_index(index_bounds<index_type>::invalid())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a global accessor with the container and index
  /// @param container Pointer to the container
  /// @param index Index of the reference
  //////////////////////////////////////////////////////////////////////
  container_accessor(container_type* container, index_type const& index)
    : base_type(container, index),
      m_container(container),
      m_index(index)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a global accessor with the container and index
  /// @param container Pointer to the container
  /// @param i Indices of the reference
  //////////////////////////////////////////////////////////////////////
  template <typename... Indices>
  container_accessor(container_type* container, Indices const&... i)
    : base_type(container, i...),
      m_container(container),
      m_index(i...)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the index of the reference
  //////////////////////////////////////////////////////////////////////
  index_type index() const
  {
    return m_index;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return whether or not this is a local or remote reference
  //////////////////////////////////////////////////////////////////////
  bool is_local() const
  {
    return this->element_pointer_valid()
      || (container_pointer_valid() && container().is_local(m_index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Read the reference by returning a copy of the value in the
  /// container at the index
  //////////////////////////////////////////////////////////////////////
  value_type read() const
  {
    if (this->element_pointer_valid())
      return this->element();

    if (container_pointer_valid())
      return container().get_element(m_index);

    // else
    return sync_rmi(0, handle(), &Container::get_element, m_index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Write the reference by setting the value in the
  /// container at the index to a given value
  /// @param val An element that can be convertible to the reference's
  /// value_type that is to be written
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void write(T&& val) const
  {
    if (this->element_pointer_valid())
    {
      this->element() = std::forward<T>(val);
      return;
    }

    if (container_pointer_valid())
    {
      container().set_element(m_index, std::forward<T>(val));
      return;
    }

    //else
    async_rmi(0, handle(), &Container::set_element, m_index,
              std::forward<T>(val));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the reference element.
  /// @param f Functor to apply. The function object's function operator
  /// must be declared const.
  /// @return Result of applying the functor to the element
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void apply_set(const F& f) const
  {
    if (this->element_pointer_valid())
    {
      f(this->element());
      return;
    }

    if (container_pointer_valid())
    {
      container().apply_set(m_index,f);
      return;
    }

    // else
    typedef void (Container::* mem_fun_t)(index_type const&, F const&);

    constexpr mem_fun_t mem_fun = &Container::apply_set;

    async_rmi(0, handle(), mem_fun, m_index, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the reference element
  /// and returns the result.
  /// @param f Functor to apply. The function object must export a nested
  /// trait for result_type and its function operator must be declared const.
  /// @return Result of applying the functor to the element
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename boost::result_of<F(value_type)>::type
  apply_get(F const& f) const
  {
    if (this->element_pointer_valid())
      return f(this->element());

    if (container_pointer_valid())
      return container().apply_get(m_index, f);

    using result_type   = typename F::result_type;

    using transfer_type =
      typename boost::result_of<transporter_packager(result_type)>::type;

    using wrapper_type  = detail::apply_get_wrapper<F>;

    // else
    promise<transfer_type> p;
    auto ft = p.get_future();

    typedef void (Container::* mem_fun_t)
      (index_type const&, wrapper_type const&);

    constexpr mem_fun_t mem_fun = &Container::apply_set;

    async_rmi(0, handle(), mem_fun, m_index, wrapper_type{f, std::move(p)});

    transfer_type package = ft.get(); // sync_rmi() equivalent

    return transporter_unpackager()(package);
  }
}; // class container_accessor

} // namespace stapl

#endif // STAPL_CONTAINERS_CONTAINER_ACCESSOR_HPP
