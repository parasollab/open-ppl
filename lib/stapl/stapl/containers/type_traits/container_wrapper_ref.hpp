/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONTAINER_WRAPPER_REF_HPP
#define STAPL_CONTAINERS_CONTAINER_WRAPPER_REF_HPP

#include <memory>
#include <boost/type_traits/integral_constant.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief A wrapper for storing containers as the value_type of another
/// container. This is needed because we want to store a reference to the
/// container as a p_object in a base container.
///
/// This class is similar in spirit to boost::ref.
///
/// @tparam T The container to wrap.
//////////////////////////////////////////////////////////////////////
template<typename T>
class container_wrapper_ref
{
private:
  rmi_handle::reference  m_handle_ref;
  /// A pointer to the wrapped value
  T* m_ptr;

public:
  /// The type that this wrapper is holding
  typedef T type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an invalid reference, setting the pointer to NULL
  //////////////////////////////////////////////////////////////////////
  container_wrapper_ref(void)
    : m_ptr(nullptr)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initialize the reference with the value to wrap.
  /// @param t The value to wrap
  //////////////////////////////////////////////////////////////////////
  explicit
  container_wrapper_ref(T const& t)
    : m_handle_ref(const_cast<T&>(t).get_rmi_handle()),
      m_ptr(std::addressof(const_cast<T&>(t)))
  { }

  container_wrapper_ref& operator=(T const& t)
  {
    m_handle_ref = const_cast<T&>(t).get_rmi_handle();
    m_ptr = std::addressof(const_cast<T&>(t));
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Cast the value of the wrapper to its original reference type.
  //////////////////////////////////////////////////////////////////////
  operator T const&(void) const
  {
    return get();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Cast the value of the wrapper to its original reference type.
  //////////////////////////////////////////////////////////////////////
  operator T&(void)
  {
    return get();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Retrieve the original reference from the wrapper.
  //////////////////////////////////////////////////////////////////////
  T& get(void)
  {
    stapl_assert(m_ptr, "Wrapper not initialized");
    return *m_ptr;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Retrieve the original reference as a const reference from the
  /// wrapper.
  //////////////////////////////////////////////////////////////////////
  T const& get(void) const
  {
    stapl_assert(m_ptr, "Wrapper not initialized");
    return *m_ptr;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Retrieve the pointer to the wrapped value.
  //////////////////////////////////////////////////////////////////////
  T* get_pointer(void) const
  {
    stapl_assert(m_ptr, "Wrapper not initialized");
    return m_ptr;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Serialization of the reference
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.member(m_handle_ref);
    t.transient(m_ptr, resolve_handle<T>(m_handle_ref));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Retrieve the pointer from a wrapped reference.
/// @param t The reference
/// @return The pointer that is within the reference
//////////////////////////////////////////////////////////////////////
template <typename T>
T* get_pointer(container_wrapper_ref<T> const& t)
{
  return t.get_pointer();
}


//////////////////////////////////////////////////////////////////////
/// @brief Unwraps @p t from @ref container_wrapper_ref.
//////////////////////////////////////////////////////////////////////
template<typename T>
T& unwrap_container_wrapper(T& t)
{
  return t;
}


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref unwrap_container_wrapper for
///        @ref container_wrapper_ref.
//////////////////////////////////////////////////////////////////////
template<typename T>
T& unwrap_container_wrapper(container_wrapper_ref<T> t)
{
  return t.get();
}



//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a given type is a
/// @ref container_wrapper_ref.
///
/// @tparam T The type in question
//////////////////////////////////////////////////////////////////////
template<typename T>
struct is_container_wrapper_ref
  : boost::false_type
{ };

template<typename C>
struct is_container_wrapper_ref<container_wrapper_ref<C> >
  : boost::true_type
{ };

} // namespace stapl

#endif // STAPL_CONTAINERS_CONTAINER_WRAPPER_REF_HPP
