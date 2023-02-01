/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_SHARED_PTR_HPP
#define STAPL_RUNTIME_SERIALIZATION_SHARED_PTR_HPP

#include "pointer.hpp"
#include "../type_traits/is_p_object.hpp"
#include "../type_traits/is_shared_ptr.hpp"
#include "../type_traits/polymorphic.hpp"
#include <cstring>
#include <type_traits>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Creates a new @c shared_ptr from the given object.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T, typename = void>
struct create_shared_ptr;


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref create_shared_ptr for @c std::shared_ptr.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
struct create_shared_ptr<std::shared_ptr<T>,
                         typename std::enable_if<
                           !is_known_polymorphic<T>::value &&
                           !is_p_object<T>::value
                         >::type>
{
  static std::shared_ptr<T> apply(T& t)
  { return std::make_shared<T>(t); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref create_shared_ptr for @c std::shared_ptr of
///        polymorphic types.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
struct create_shared_ptr<std::shared_ptr<T>,
                         typename std::enable_if<
                           is_known_polymorphic<T>::value &&
                           !is_p_object<T>::value
                         >::type>
{
  static std::shared_ptr<T> apply(T& t)
  { return clone(t); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref create_shared_ptr for @c std::shared_ptr of
///        distributed objects.
///
/// @warning Distributed objects that are managed through a @c std::shared_ptr
///          are required to provide a function @c shared_from_this(), e.g., by
///          extending from @c std::enable_shared_from_this.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
struct create_shared_ptr<std::shared_ptr<T>,
                         typename std::enable_if<
                           is_p_object<T>::value
                         >::type>
{
  static std::shared_ptr<T> apply(T& t)
  { return t.shared_from_this(); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref create_shared_ptr for @c boost::shared_ptr.
///
/// @warning There is no specialization of @ref apply() that calls
///          @c boost::make_shared(). Therefore, serialization of
///          @c boost::shared_ptr is not as fast as that of @c std::shared_ptr.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
struct create_shared_ptr<boost::shared_ptr<T>,
                         typename std::enable_if<
                           !is_p_object<T>::value
                         >::type>
{
  static boost::shared_ptr<T> apply(T& t)
  { return boost::shared_ptr<T>(clone(t)); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref create_shared_ptr for @c boost::shared_ptr of
///        distributed objects.
///
/// @warning Distributed objects that are managed through a @c boost::shared_ptr
///          are required to provide a function @c shared_from_this(), e.g., by
///          extending from @c boost::enable_shared_from_this.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
struct create_shared_ptr<boost::shared_ptr<T>,
                         typename std::enable_if<
                           is_p_object<T>::value
                         >::type>
{
  static boost::shared_ptr<T> apply(T& t)
  { return t.shared_from_this(); }
};

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits for @ref std::shared_ptr and
///        @ref boost::shared_ptr.
///
/// Objects that are managed by a @c shared_ptr are packed as pointers to the
/// objects and the offset to the packed object is stored in the space
/// allocated for the @c shared_ptr. This is safe, since
/// @c sizeof(shared_ptr<T>)>=sizeof(T*)
///
/// @ingroup serialization
///
/// @todo It does not support a custom deleter.
//////////////////////////////////////////////////////////////////////
template<typename T>
class typer_traits<T,
                   typename std::enable_if<
                     runtime::is_shared_ptr<T>::value
                   >::type>
{
private:
  using element_type = typename T::element_type;
  using pointer_type = element_type*;
  using traits_type  = typer_traits<pointer_type>;

  static_assert(sizeof(T)>=sizeof(pointer_type),
                "Not enough space for shared_ptr packing.");

public:
  using value_type = T;

  static std::size_t packed_size(T const& t) noexcept
  { return traits_type::packed_size(t.get()); }

  static std::pair<bool, std::size_t>
  meets_requirements(const typer::pass_type p, T const& t) noexcept
  {
    // the decision of whether a shared_ptr object has to be marshaled belongs
    // to the object it points to
    const auto r = traits_type::meets_requirements(typer::NO_MARSHAL, t.get());

    constexpr bool is_const = std::is_const<element_type>::value;

    switch (p) {
      case typer::COPY:
        // shared_ptr objects can be copied if they point to a const object and
        // that object does not require being marshaled, otherwise race
        // conditions may be introduced in shared memory
        return std::make_pair((is_const && r.first), r.second);

      case typer::MOVE:
        // shared_ptr objects can be moved if they point to a const object or
        // they are the only shared_ptr object to that object and that object
        // does not require being marshaled, otherwise race conditions may be
        // introduced in shared memory
        return std::make_pair(((is_const || t.unique()) && r.first), r.second);

      case typer::NO_MARSHAL:
        return r;

      default:
        std::abort();
    }
  }

  static void prepack(T*, T const*, const std::size_t = 1) noexcept
  { }

  static std::size_t pack(T& dest,
                          void* base,
                          const std::size_t offset,
                          T const& src) noexcept
  {
    // packing as a pointer, writing the offset to the packed object into the
    // space of the shared_ptr
    auto* p = reinterpret_cast<pointer_type>(&dest);
    const std::size_t s = traits_type::pack(p, base, offset, src.get());
    std::memcpy(static_cast<void*>(&dest), &p, sizeof(p));
    return s;
  }

  static std::size_t unpack(T& t, void* base)
  {
    // get the offset from the shared_ptr object and unpack as a pointer to the
    // object
    auto* tp       = &t;
    pointer_type p = nullptr;
    std::memcpy(&p, tp, sizeof(p));
    const std::size_t s = traits_type::unpack(p, base);
    // placement new to create a correct shared_ptr
    if (p)
      ::new(tp) T(runtime::create_shared_ptr<T>::apply(*p));
    else
      ::new(tp) T;
    return s;
  }

  static void destroy(T& t) noexcept
  { t.~value_type(); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits_specialization for
///        @ref std::shared_ptr and @ref boost::shared_ptr.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
struct typer_traits_specialization<T,
                                   typename std::enable_if<
                                     runtime::is_shared_ptr<T>::value
                                   >::type>
: public std::true_type
{ };

} // namespace stapl

#endif
