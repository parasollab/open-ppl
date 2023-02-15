/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_POINTER_HPP
#define STAPL_RUNTIME_SERIALIZATION_POINTER_HPP

#include "typer_traits.hpp"
#include "../exception.hpp"
#include "../type_traits/polymorphic.hpp"
#include "../type_traits/is_basic.hpp"
#include "../type_traits/is_p_object.hpp"
#include "../type_traits/aligned_storage.hpp"
#include <cstring>
#include <type_traits>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Copy constructs a new @p T object on heap allocated memory.
///
/// @tparam T Object type to be cloned.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T, typename = void>
struct cloner
{
  static T* apply(T const& t)
  { return new T(t); }
};

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Clones @p t.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
T* clone(T const& t)
{
  return runtime::cloner<T>::apply(t);
}


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits for pointers to objects.
///
/// @tparam T Pointer to object type.
///
/// In order to pack a pointer to an object, the static part of the object is
/// first copied to the buffer, followed by its dynamic part.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
class typer_traits<T*,
                   typename std::enable_if<
                     !runtime::is_known_polymorphic<T>::value &&
                     !is_p_object<T>::value                   &&
                     !std::is_function<T>::value
                   >::type>
{
private:
  using U           = typename std::remove_cv<T>::type;
  using traits_type = typer_traits<U>;
public:
  using value_type  = T*;

  static std::size_t packed_size(T* t, const std::size_t num = 1) noexcept
  {
    using runtime::aligned_size;

    if (!t || num==0)
      return 0;

    // static part
    std::size_t s = aligned_size(num * sizeof(T));

    if (is_basic<U>::value)
      return s;

    // dynamic part
    std::size_t tsz = 0;
    for (std::size_t i=0; i<num; ++i, ++t) {
      tsz += traits_type::packed_size(*const_cast<U*>(t));
    }

    return (s + aligned_size(tsz));
  }

  static std::pair<bool, std::size_t>
  meets_requirements(const typer::pass_type p,
                     T* t,
                     const std::size_t num = 1) noexcept
  {
    using runtime::aligned_size;

    if (!t || num==0)
      return std::make_pair(true, std::size_t(0));

    // static part
    std::size_t s = aligned_size(num * sizeof(T));

    if (is_basic<U>::value)
      return std::make_pair(true, s);

    // dynamic part
    bool b = true;
    std::size_t tsz = 0;
    for (std::size_t i=0; i<num; ++i, ++t) {
      auto r = traits_type::meets_requirements(p, *const_cast<U*>(t));
      if (!r.first)
        b = false;
      tsz += r.second;
    }

    return std::make_pair(b, (s + aligned_size(tsz)));
  }

  static void prepack(T**, T**, const std::size_t = 1) noexcept
  { }

  static std::size_t pack(T*& dest,
                          void* base,
                          const std::size_t offset,
                          T* src,
                          const std::size_t num = 1) noexcept
  {
    using runtime::aligned_size;

    if (!src || num==0) {
      dest = nullptr;
      return 0;
    }

    // pointer to free space
    U* p = reinterpret_cast<U*>(static_cast<char*>(base) + offset);

    // static part
    traits_type::prepack(p, const_cast<U*>(src), num);
    std::size_t noffset = (offset + aligned_size(num * sizeof(T)));

    // dynamic part
    if (!is_basic<U>::value) {
      std::size_t tsz = 0;
      for (std::size_t i=0; i<num; ++i, ++p, ++src) {
        tsz += traits_type::pack(*p, base, (noffset+tsz), *const_cast<U*>(src));
      }
      noffset += aligned_size(tsz);
    }

    // save the offset to the packed object
    const std::uintptr_t tmp = offset;
    std::memcpy(&dest, &tmp, sizeof(tmp));

    return (noffset - offset);
  }

  static std::size_t unpack(T*& t, void* base, const std::size_t num = 1)
  {
    using runtime::aligned_size;

    if (!t || num==0)
      return 0;

    // find pointer to packed object
    std::uintptr_t offset = 0;
    std::memcpy(&offset, &t, sizeof(offset));
    t = reinterpret_cast<T*>(static_cast<char*>(base) + offset);

    // static part
    std::size_t s = aligned_size(num * sizeof(T));

    if (is_basic<U>::value)
      return s;

    // dynamic part
    U* p = const_cast<U*>(t);
    std::size_t tsz = 0;
    for (std::size_t i=0; i<num; ++i, ++p) {
      tsz += traits_type::unpack(*p, base);
    }

    return (s + aligned_size(tsz));
  }

  static void destroy(T* t, const std::size_t num = 1) noexcept
  {
    if (!t || num==0)
      return;

    for (std::size_t i=0; i<num; ++i, ++t) {
      traits_type::destroy(*const_cast<U*>(t));
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits_specialization for pointers to
///        objects.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
struct typer_traits_specialization<T*,
                                   typename std::enable_if<
                                     !runtime::is_known_polymorphic<T>::value &&
                                     !is_p_object<T>::value                   &&
                                     !std::is_function<T>::value
                                   >::type>
: public std::true_type
{ };

} // namespace stapl

#endif
