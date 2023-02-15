/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_POLYMORPHIC_HPP
#define STAPL_RUNTIME_SERIALIZATION_POLYMORPHIC_HPP

#include "pointer.hpp"
#include "typer_traits.hpp"
#include "../exception.hpp"
#include "../type_traits/polymorphic.hpp"
#include <cstring>
#include <type_traits>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref cloner for polymorphic types.
///
/// @tparam T Object type to be cloned.
///
/// Find the correct type, cast to it and then copy it.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
struct cloner<T,
              typename std::enable_if<
                is_known_polymorphic<T>::value &&
                !is_p_object<T>::value
              >::type>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Clones the object when the correct type is found.
  //////////////////////////////////////////////////////////////////////
  struct clone_wf
  {
    T const& m_t;

    constexpr explicit clone_wf(T const& t) noexcept
    : m_t(t)
    { }

    template<typename U>
    T* operator()(void) const
    { return new U(dynamic_cast<U const&>(m_t)); }
  };

  static T* apply(T const& t)
  {
    using typelist_type = typename derived_types<T>::typelist_type;

    const auto id = t.get_polymorphic_type_id();
    return polymorphic_apply<T*, typelist_type>(clone_wf{t}, id);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Calls @ref typer_traits::packed_size() for polymorphic types.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
class poly_packed_size
{
private:
  T* const          m_t;
  const std::size_t m_num;

public:
  constexpr poly_packed_size(T* t, const std::size_t num) noexcept
  : m_t(t),
    m_num(num)
  { }

  template<typename U>
  std::size_t operator()(void) const noexcept
  {
    using traits_type = typer_traits<U*>;
    U* u = dynamic_cast<U*>(m_t);
    STAPL_RUNTIME_ASSERT(u);
    return traits_type::packed_size(u, m_num);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Calls the requested function from @ref typer_traits for polymorphic
///        types.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
class poly_meets_requirements
{
private:
  const typer::pass_type m_pass;
  T* const               m_t;
  const std::size_t      m_num;

public:
  constexpr poly_meets_requirements(const typer::pass_type p,
                                    T* t,
                                    const std::size_t num) noexcept
  : m_pass(p),
    m_t(t),
    m_num(num)
  { }

  template<typename U>
  std::pair<bool, std::size_t> operator()(void) noexcept
  {
    using traits_type = typer_traits<U*>;
    U* u = dynamic_cast<U*>(m_t);
    STAPL_RUNTIME_ASSERT(u);
    return traits_type::meets_requirements(m_pass, u, m_num);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Calls @ref typer_traits::pack() for polymorphic types.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
class poly_pack
{
private:
  T*&               m_dest;
  void* const       m_base;
  const std::size_t m_offset;
  T* const          m_src;
  const std::size_t m_num;

public:
  constexpr poly_pack(T*& dest,
                      void* base,
                      const std::size_t offset,
                      T* src,
                      const std::size_t num) noexcept
  : m_dest(dest),
    m_base(base),
    m_offset(offset),
    m_src(src),
    m_num(num)
  { }

  template<typename U>
  std::size_t operator()(void) noexcept
  {
    using traits_type = typer_traits<U*>;
    U* dest = static_cast<U*>(m_dest);
    U* src  = dynamic_cast<U*>(m_src);
    STAPL_RUNTIME_ASSERT(src);
    const std::size_t s = traits_type::pack(dest, m_base, m_offset, src, m_num);
    std::memcpy(&m_dest, &dest, sizeof(dest));
    return s;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Calls for @ref typer_traits::unpack() for polymorphic types.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
class poly_unpack
{
private:
  T*&               m_t;
  void* const       m_base;
  const std::size_t m_num;

public:
  constexpr poly_unpack(T*& t, void* base, const std::size_t num) noexcept
  : m_t(t),
    m_base(base),
    m_num(num)
  { }

  template<typename U>
  std::size_t operator()(void) noexcept
  {
    using traits_type = typer_traits<U*>;
    U* u = static_cast<U*>(m_t);
    const std::size_t s = traits_type::unpack(u, m_base, m_num);
    std::memcpy(&m_t, &u, sizeof(u));
    return s;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Calls @ref typer_traits::destroy() for polymorphic types.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
class poly_destroy
{
private:
  T*&               m_t;
  const std::size_t m_num;

public:
  constexpr poly_destroy(T*& t, const std::size_t num) noexcept
  : m_t(t),
    m_num(num)
  { }

  template<typename U>
  void operator()(void) noexcept
  {
    using traits_type = typer_traits<U*>;
    U* dest = dynamic_cast<U*>(m_t);
    STAPL_RUNTIME_ASSERT(dest);
    traits_type::destroy(dest, m_num);
  }
};

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits for pointers to polymorphic
///        objects.
///
/// The packing/unpacking relies on a deferred pointer to object packing.
/// Function objects are used (@ref pointer_impl::poly_packed_size,
/// @ref pointer_impl::poly_pack, @ref pointer_impl::poly_unpack and
/// @ref pointer_impl::poly_destroy) to find the packed size, pack, unpack and
/// correctly cleanup a pointer to a polymorphic object.
///
/// The procedure is similar in all cases: first the type id of the object is
/// retrieved. A functor is created with the necessary pointers to the base
/// object pointer, the buffer and any other relevant information. The type id
/// list is being traversed and once the type id corresponding to the one that
/// has been retrieved is found, then the functor is invoked with the type that
/// corresponds to the type id.
///
/// Since it is using the default @ref typer_traits for pointers to regular
/// objects, it makes the latter think that the type id does not exist by fixing
/// the offset accordingly (see @ref unpack()).
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
class typer_traits<T*,
                   typename std::enable_if<
                     runtime::is_known_polymorphic<T>::value &&
                     !is_p_object<T>::value                  &&
                     !std::is_function<T>::value
                   >::type>
{
private:
  using typelist_type = typename derived_types<T>::typelist_type;
public:
  using value_type    = T*;

  static std::size_t packed_size(T* t, const std::size_t num = 1) noexcept
  {
    using namespace runtime;

    if (!t || num==0)
      return 0;

    // size of type id
    std::size_t s = sizeof(type_id);

    // size of objects
    const auto id = t->get_polymorphic_type_id();
    s += polymorphic_apply<
           std::size_t, typelist_type
         >(poly_packed_size<T>{t, num}, id);

    return s;
  }

  static std::pair<bool, std::size_t>
  meets_requirements(const typer::pass_type p,
                     T* t,
                     const std::size_t num = 1) noexcept
  {
    using namespace runtime;

    if (!t || num==0)
      return std::make_pair(true, std::size_t(0));

    // size of type id
    std::size_t s = sizeof(type_id);

    // meets requirements of objects
    const auto id = t->get_polymorphic_type_id();
    const auto r = polymorphic_apply<
                     std::pair<bool, std::size_t>, typelist_type
                   >(poly_meets_requirements<T>{p, t, num}, id);
    s += r.second;

    return std::make_pair(r.first, s);
  }

  static void prepack(T**, T**, const std::size_t = 1) noexcept
  { }

  static std::size_t pack(T*& dest,
                          void* base,
                          const std::size_t offset,
                          T* src,
                          const std::size_t num = 1) noexcept
  {
    using namespace runtime;

    if (!src || num==0) {
      dest = nullptr;
      return 0;
    }

    // save type id
    const auto id = src->get_polymorphic_type_id();
    std::memcpy((static_cast<char*>(base) + offset), &id, sizeof(id));

    // pack objects - dest will be set to the start of the packed object
    std::size_t noffset = (offset + sizeof(type_id));
    noffset += polymorphic_apply<
                 std::size_t, typelist_type
               >(poly_pack<T>{dest, base, noffset, src, num}, id);

    return (noffset - offset);
  }

  static std::size_t unpack(T*& t,
                            void* base,
                            const std::size_t num = 1) noexcept
  {
    using namespace runtime;

    if (!t || num==0)
      return 0;

    // find offset to the start of the packed object
    std::uintptr_t tmp = 0;
    std::memcpy(&tmp, &t, sizeof(tmp));
    std::size_t offset = tmp;

    // backtrack pointer to get the type id
    offset -= sizeof(type_id);
    auto id = type_id{};
    std::memcpy(&id, (static_cast<char*>(base) + offset), sizeof(id));

    // advance pointer and unpack objects
    offset += sizeof(type_id);
    const std::size_t s = polymorphic_apply<
                            std::size_t, typelist_type
                          >(poly_unpack<T>{t, base, num}, id);

    // type id size is not counted in the unpacked objects' size
    return (s + sizeof(type_id));
  }

  static void destroy(T* t, const std::size_t num = 1) noexcept
  {
    using namespace runtime;

    if (!t || num==0)
      return;

    const auto id = t->get_polymorphic_type_id();
    polymorphic_apply<void, typelist_type>(poly_destroy<T>{t, num}, id);
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits_specialization for pointers to
///        polymorphic objects.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
struct typer_traits_specialization<T*,
                                   typename std::enable_if<
                                     runtime::is_known_polymorphic<T>::value &&
                                     !is_p_object<T>::value                  &&
                                     !std::is_function<T>::value
                                   >::type>
: public std::true_type
{ };

} // namespace stapl

#endif
