/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_REF_COUNTED_HPP
#define STAPL_RUNTIME_UTILITY_REF_COUNTED_HPP

#include "../exception.hpp"
#include <atomic>
#include <memory>
#include <type_traits>
#include <utility>
#include <boost/smart_ptr/intrusive_ptr.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Provides a reference counting mechanism for objects that derive from
///        it.
///
/// @tparam T       Reference counted object type.
/// @tparam Deleter Deleter type.
///
/// @ref ref_counted is compatible with @c boost::intrusive_ptr.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T, typename Deleter = std::default_delete<T>>
class ref_counted
: private Deleter
{
private:
  std::atomic<long> m_cnt;

public:
  constexpr ref_counted(void) noexcept
  : m_cnt(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref ref_counted with a custom deleter.
  //////////////////////////////////////////////////////////////////////
  explicit ref_counted(Deleter const& d) noexcept
  : Deleter(d),
    m_cnt(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc ref_counted(Deleter const&)
  //////////////////////////////////////////////////////////////////////
  explicit ref_counted(Deleter&& d) noexcept
  : Deleter(std::move(d)),
    m_cnt(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref ref_counted with an arbitrary initial count.
  //////////////////////////////////////////////////////////////////////
  constexpr explicit ref_counted(long count) noexcept
  : m_cnt(count)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref ref_counted with an arbitrary initial count and
  ///        a custom deleter.
  //////////////////////////////////////////////////////////////////////
  ref_counted(long count, Deleter const& d) noexcept
  : Deleter(d),
    m_cnt(count)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc ref_counted(long,Deleter const&)
  //////////////////////////////////////////////////////////////////////
  ref_counted(long count, Deleter&& d) noexcept
  : Deleter(std::move(d)),
    m_cnt(count)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increases the reference count by @p count.
  //////////////////////////////////////////////////////////////////////
  void add_ref(long count = 1,
               std::memory_order order = std::memory_order_relaxed) noexcept
  { m_cnt.fetch_add(count, order); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Decreases the reference count by @p count.
  ///
  /// @return The new reference count.
  //////////////////////////////////////////////////////////////////////
  long remove_ref(long count = 1,
                  std::memory_order order = std::memory_order_relaxed) noexcept
  {
    const long cnt = m_cnt.fetch_sub(count, order);
    STAPL_RUNTIME_ASSERT(cnt>0);
    return (cnt - count);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Decreases the reference count, destroying the object if it reached
  ///        0.
  ///
  /// @warning If @c release() returns @c true, then the object has been
  ///          destroyed.
  ///
  /// @return @c true if the object was destroyed, otherwise @c false.
  //////////////////////////////////////////////////////////////////////
  bool release(std::memory_order order = std::memory_order_relaxed) noexcept
  {
    const long cnt = m_cnt.fetch_sub(1, order);
    STAPL_RUNTIME_ASSERT(cnt>0);
    if (cnt>1)
      return false;
    // object released
    static_cast<Deleter&>(*this)(static_cast<T*>(this));
    return true;
  }

  long
  use_count(std::memory_order order = std::memory_order_relaxed) const noexcept
  { return m_cnt.load(order); }

  bool
  unique(std::memory_order order = std::memory_order_relaxed) const noexcept
  { return (use_count(order)==1); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Increases the reference count of the object pointed to by @p p.
///
/// Required for @c boost::intrusive_ptr compliance.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T, typename Deleter>
void intrusive_ptr_add_ref(ref_counted<T, Deleter>* p) noexcept
{
  p->add_ref();
}


//////////////////////////////////////////////////////////////////////
/// @brief Decreases the reference count of the object pointed to by @p p and
///        and deletes it if the count has reached @c 0.
///
/// Required for @c boost::intrusive_ptr compliance.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T, typename Deleter>
void intrusive_ptr_release(ref_counted<T, Deleter>* p) noexcept
{
  p->release();
}


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper over @p T to make it conformable with @ref ref_counted.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T>
class ref_counted_wrapper
: public T,
  public ref_counted<ref_counted_wrapper<T>>
{
public:
  template<typename... U>
  ref_counted_wrapper(U&&... u)
  : T(std::forward<U>(u)...)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Constructs an object of type @p T that provides the interface and
///        functionality of @ref ref_counter using @p args as the parameter list
///        for the constructor of @p T.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T, typename... Args>
boost::intrusive_ptr<T> make_ref_counted(Args&&... args)
{
  using type = typename std::conditional<
                 std::is_base_of<ref_counted<T>, T>::value,
                 T,
                 ref_counted_wrapper<T>>::type;
  return new type(std::forward<Args>(args)...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Provides a range-based interface for @ref ref_counted derived
///        objects.
///
/// @tparam T Reference counted object type.
///
/// The object stored in this range is also reference counted to prevent
/// dangling pointers in case the object is deleted prior to the range.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T>
class ref_counted_range
{
public:
  using size_type      = typename T::size_type;
  using const_iterator = typename T::const_iterator;
  using iterator       = typename T::iterator;

private:
  boost::intrusive_ptr<T> m_p;

public:
  explicit ref_counted_range(T& t)
  : m_p(&t)
  { }

  explicit ref_counted_range(boost::intrusive_ptr<T> p)
  : m_p(std::move(p))
  { }

  bool empty(void) const noexcept
  { return m_p->empty(); }

  size_type size(void) const noexcept
  { return m_p->size(); }

  const_iterator begin(void) const noexcept
  { return m_p->begin(); }

  const_iterator end(void) const noexcept
  { return m_p->end(); }

  iterator begin(void) noexcept
  { return m_p->begin(); }

  iterator end(void) noexcept
  { return m_p->end(); }
};

} // namespace runtime

} // namespace stapl

#endif
