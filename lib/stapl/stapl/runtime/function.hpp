/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */


#ifndef STAPL_RUNTIME_FUNCTION_HPP
#define STAPL_RUNTIME_FUNCTION_HPP

#include "serialization/typer_traits.hpp"
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief A general-purpose polymorphic function wrapper.
///
/// @tparam Signature Stored callable signature.
///
/// @ref stapl::function is similar to @c std::function, as it can store and
/// call any target. In addition, it has the necessary functions to allow
/// packing/unpacking.
///
/// @warning Any target stored in a @ref stapl::function has to support some
///          form of packing/unpacking.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename Signature>
class function
: public std::function<Signature>
{
private:
  typedef std::function<Signature> base_type;

  static_assert(sizeof(base_type)>=sizeof(std::uintptr_t),
                "Not enough space to serialize stapl::function");

  template<typename T>
  static void packed_size(void* result, function const& f) noexcept
  {
    STAPL_RUNTIME_ASSERT(bool(f));

    typedef typer_traits<T*> traits_type;

    auto& s = *static_cast<std::size_t*>(result);
    T* t    = const_cast<T*>(f.template target<T>());
    s       = traits_type::packed_size(t);
  }

  template<typename T>
  static void meets_requirements(void* result,
                                 const typer::pass_type p,
                                 function const& f) noexcept
  {
    STAPL_RUNTIME_ASSERT(bool(f));

    typedef typer_traits<T*> traits_type;

    auto& r = *static_cast<std::pair<bool, std::size_t>*>(result);
    T* t    = const_cast<T*>(f.template target<T>());
    r       = traits_type::meets_requirements(p, t);
  }

  template<typename T>
  static void pack(void* result,
                   function& dest,
                   void* base,
                   const std::size_t offset,
                   function const& src) noexcept
  {
    STAPL_RUNTIME_ASSERT(bool(src));

    typedef typer_traits<T*> traits_type;

    T* t_src = const_cast<T*>(src.template target<T>());
    T* t     = nullptr;
    auto& s  = *static_cast<std::size_t*>(result);
    s        = traits_type::pack(t, base, offset, t_src);
    std::memcpy(static_cast<void*>(&dest), &t, sizeof(t));
  }

  template<typename T>
  static void unpack(void* result, function& f, void* base)
  {
    typedef typer_traits<T*> traits_type;

    T* t = nullptr;
    std::memcpy(&t, &f, sizeof(t));
    STAPL_RUNTIME_ASSERT(t);
    auto& s = *static_cast<std::size_t*>(result);
    s       = traits_type::unpack(t, base);
    ::new(&f) function(*t);
  }

  template<typename T>
  static void helper(void* result,
                     const typer::pass_type p,
                     function& f,
                     void* base,
                     const std::size_t offset,
                     function const* src)
  {
    switch (p) {
      case typer::SIZE:
        packed_size<T>(result, f);
        break;
      case typer::COPY:
      case typer::MOVE:
      case typer::NO_MARSHAL:
        meets_requirements<T>(result, p, f);
        break;
      case typer::PACK:
        pack<T>(result, f, base, offset, *src);
        break;
      case typer::UNPACK:
        unpack<T>(result, f, base);
        break;
      default:
        STAPL_RUNTIME_ERROR("Incorrect state.");
        break;
    }
  }

  void (*m_helper)(void*, const typer::pass_type,
                   function&, void*, const std::size_t, function const*);

  friend class typer_traits<function<Signature>>;

public:
  function(void)
  : m_helper(nullptr)
  { }

  template<typename Function>
  function(Function f)
  : base_type(std::move(f)),
    m_helper(&helper<Function>)
  { }

  function(function const&) = default;
  function(function&&) = default;

  function& operator=(function const&) = default;
  function& operator=(function&&) = default;

private:
  std::size_t packed_size(void) const noexcept
  {
    if (!m_helper)
      return 0;
    std::size_t result = 0;
    (*m_helper)(
      &result, typer::SIZE, const_cast<function&>(*this), nullptr, 0, nullptr
    );
    return result;
  }

  std::pair<bool, std::size_t>
  meets_requirements(const typer::pass_type p) const noexcept
  {
    if (!m_helper)
      return std::make_pair(true, std::size_t(0));
    std::pair<bool, std::size_t> result;
    (*m_helper)(&result, p, const_cast<function&>(*this), nullptr, 0, nullptr);
    return result;
  }

  std::size_t pack(function& dest,
                   void* base,
                   const std::size_t offset) const noexcept
  {
    if (!m_helper)
      return 0;
    std::size_t result = 0;
    (*m_helper)(&result, typer::PACK, dest, base, offset, this);
    return result;
  }

  std::size_t unpack(void* base)
  {
    if (!m_helper) {
      ::new(static_cast<void*>(this)) function;
      return 0;
    }
    std::size_t result = 0;
    (*m_helper)(&result, typer::UNPACK, *this, base, 0, nullptr);
    return result;
  }

  void destroy(void) noexcept
  { this->~function(); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits for @ref stapl::function.
///
/// @tparam Signature Stored callable signature.
///
/// Callables that are stored in a @ref stapl::function are packed as the
/// respective object (for example function pointer, function object etc).
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename Signature>
class typer_traits<function<Signature>>
{
public:
  typedef function<Signature> value_type;

  static std::size_t packed_size(function<Signature> const& f) noexcept
  { return f.packed_size(); }

  static std::pair<bool, std::size_t>
  meets_requirements(const typer::pass_type p,
                     function<Signature> const& f) noexcept
  { return f.meets_requirements(p); }

  static void prepack(function<Signature>* dest,
                      function<Signature> const* src,
                      const std::size_t num = 1) noexcept
  {
    std::memcpy(static_cast<void*>(dest),
                static_cast<void const*>(src),
                (sizeof(function<Signature>) * num));
  }

  static std::size_t pack(function<Signature>& dest,
                          void* base,
                          const std::size_t offset,
                          function<Signature> const& src) noexcept
  { return src.pack(dest, base, offset); }

  static std::size_t unpack(function<Signature>& f, void* base)
  { return f.unpack(base); }

  static void destroy(function<Signature>& f) noexcept
  { f.destroy(); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits_specialization for
///        @ref stapl::function.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename Signature>
struct typer_traits_specialization<function<Signature>>
: public std::true_type
{ };

} // namespace stapl

#endif
