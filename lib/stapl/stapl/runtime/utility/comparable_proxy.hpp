/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_COMPARABLE_PROXY_HPP
#define STAPL_RUNTIME_UTILITY_COMPARABLE_PROXY_HPP

#include "../serialization.hpp"
#include <cstdint>
#include <typeindex>
#include <typeinfo>
#include <type_traits>
#include <utility>
#include <stapl/utility/hash_fwd.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Function object to test if two objects are equal.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T, bool IsEmpty = std::is_empty<T>::value>
struct comparator
{
  static bool apply(T const& t1, T const& t2) noexcept
  { return (t1==t2); }
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref comparator for empty objects.
///
/// The result is always @c true.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T>
struct comparator<T, true>
{
  static constexpr bool apply(T const&, T const&) noexcept
  { return true; }
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref comparator for @c std::pair.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T1, typename T2>
struct comparator<std::pair<T1, T2>, false>
{
  static bool
  apply(std::pair<T1, T2> const& p1, std::pair<T1, T2> const& p2) noexcept
  {
    return (comparator<T1>::apply(p1.first, p2.first) &&
            comparator<T2>::apply(p1.second, p2.second));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Proxy object that allows type-erased objects or function pointers to
///        be compared between them.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class comparable_proxy
{
private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Base class for holding any object.
  //////////////////////////////////////////////////////////////////////
  class placeholder
  {
  public:
    placeholder(void) = default;
    placeholder(placeholder const&) = delete;
    placeholder& operator=(placeholder const&) = delete;
    virtual ~placeholder(void) = default;
    virtual placeholder* clone(void) const = 0;
    virtual std::type_info const& type(void) const noexcept = 0;
    virtual bool equal_to(placeholder const&) const = 0;
    virtual bool equal_to(void const*) const = 0;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of @ref placeholder for objects of type @p T.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  class holder final
  : public placeholder
  {
  private:
    T m_t;

  public:
    explicit holder(T const& t)
    : m_t(t)
    { }

    explicit holder(T&& t)
    : m_t(std::move(t))
    { }

    T const& get(void) const noexcept
    { return m_t; }

    placeholder* clone(void) const final
    { return new holder(m_t); }

    std::type_info const& type(void) const noexcept final
    { return typeid(T); }

    bool equal_to(placeholder const& t) const final
    {
      return ((type()==t.type()) &&
              (comparator<T>::apply(m_t, static_cast<holder const&>(t).m_t)));
    }

    bool equal_to(void const* p) const final
    { return comparator<T>::apply(m_t, *static_cast<T const*>(p)); }

    void define_type(typer& t)
    { t.member(m_t); }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Packs an @ref placeholder.
  ///
  /// This happens by @c reinterpret_cast to the derived class, which in this
  /// instance it is correct, as the @ref placeholder is the only base class of
  /// @c holder<T>.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  static void define_type_impl(typer& t, placeholder*& p)
  {
    holder<T>*& r = reinterpret_cast<holder<T>*&>(p);
    STAPL_RUNTIME_ASSERT(static_cast<void*>(p)==static_cast<void*>(r));
    t.member(r);
  }

  placeholder*                             m_p;
  decltype(&define_type_impl<holder<int>>) m_define_type_fn;
  std::size_t                              m_hash_code;

public:
  template<typename T>
  explicit comparable_proxy(T&& t)
  : m_p(new holder<typename std::decay<T>::type>(std::forward<T>(t))),
    m_define_type_fn(&define_type_impl<typename std::decay<T>::type>),
    m_hash_code(
      std::type_index(typeid(typename std::decay<T>::type)).hash_code()
    )
  { }

  comparable_proxy(comparable_proxy&& other) noexcept
  : m_p(other.m_p),
    m_define_type_fn(other.m_define_type_fn),
    m_hash_code(other.m_hash_code)
  {
    other.m_p              = nullptr;
    other.m_define_type_fn = nullptr;
    other.m_hash_code      = 0;
  }

  comparable_proxy(comparable_proxy const& other)
  : m_p(other.m_p->clone()),
    m_define_type_fn(other.m_define_type_fn),
    m_hash_code(other.m_hash_code)
  { }

  ~comparable_proxy(void)
  { delete m_p; }

  comparable_proxy& operator=(comparable_proxy const&) = delete;

  template<typename T>
  T const& get(void) const noexcept
  {
    STAPL_RUNTIME_ASSERT(typeid(typename std::decay<T>::type)==m_p->type());
    return static_cast<holder<typename std::decay<T>::type>*>(m_p)->get();
  }

  template<typename T>
  bool operator==(T const& t) const
  {
    return ((m_p->type()==typeid(typename std::decay<T>::type)) &&
            (m_p->equal_to(static_cast<void const*>(&t))));
  }

  bool operator==(comparable_proxy const& t) const
  { return m_p->equal_to(*(t.m_p)); }

  template<typename T>
  bool operator!=(T const& t) const
  { return !(*this==t); }

  std::size_t hash_code(void) const noexcept
  { return m_hash_code; }

  void define_type(typer& t)
  {
    m_define_type_fn(t, m_p);
    t.member(bitwise(m_define_type_fn));
    t.member(m_hash_code);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns the hash value for @p t.
///
/// @related comparable_proxy
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
inline std::size_t hash_value(comparable_proxy const& t) noexcept
{
  return t.hash_code();
}

} // namespace runtime

} // namespace stapl


namespace std {

//////////////////////////////////////////////////////////////////////
/// @brief Hash value creation functor for
///        @ref stapl::runtime::comparable_proxy.
///
/// @related stapl::runtime::comparable_proxy
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<>
struct hash<stapl::runtime::comparable_proxy>
{
  typedef stapl::runtime::comparable_proxy argument_type;
  typedef std::size_t                      result_type;

  result_type operator()(argument_type const& t) const noexcept
  { return t.hash_code(); }
};

} // namespace std

#endif
