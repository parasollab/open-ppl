/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_REFERENCE_WRAPPER_HPP
#define STAPL_RUNTIME_SERIALIZATION_REFERENCE_WRAPPER_HPP

#include "typer_traits.hpp"
#include "../type_traits/is_reference_wrapper.hpp"
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits for @c std::reference_wrapper and
///        and @c boost::reference_wrapper.
///
/// @c std::reference_wrapper is mostly a wrapper around a pointer. Therefore,
/// this implementation calls the functions required to pack a pointer.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
class typer_traits<T,
                   typename std::enable_if<
                     runtime::is_reference_wrapper<T>::value
                   >::type>
{
private:
  using pointer_type = typename T::type*;
  using traits_type  = typer_traits<pointer_type>;
public:
  using value_type   = T;

  static std::size_t packed_size(T const& t) noexcept
  { return traits_type::packed_size(&(t.get())); }

  static std::pair<bool, std::size_t>
  meets_requirements(const typer::pass_type p, T const& t) noexcept
  {
    // reference_wrapper objects can never be copied or moved, as this would
    // introduce unexpected sharing in shared memory; however deciding if it has
    // to be marshaled depends on the object it points to
    const auto r = traits_type::meets_requirements(typer::NO_MARSHAL,
                                                   &(t.get()));
    return ((p==typer::NO_MARSHAL)
             ? r
             : std::make_pair(false, r.second));
  }

  static void prepack(T*, T const*, const std::size_t = 1) noexcept
  { }

  static std::size_t pack(T& dest,
                          void* base,
                          const std::size_t offset,
                          T const& src) noexcept
  {
    // pack the pointer to the object in p
    pointer_type p         = nullptr;
    const std::size_t size = traits_type::pack(p, base, offset, &(src.get()));
    // save p in the reference_wrapper object
    std::memcpy(&dest, &p, sizeof(p));
    return size;
  }

  static std::size_t unpack(T& t, void* base) noexcept
  {
    // load the pointer from the reference_wrapper object
    pointer_type p = nullptr;
    std::memcpy(&p, &t, sizeof(p));
    // unpack pointer
    const std::size_t size = traits_type::unpack(p, base);
    // restore the reference_wrapper object
    t = *p;
    return size;
  }

  static void destroy(T& t) noexcept
  { traits_type::destroy(&(t.get())); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits_specialization for
///        @c std::reference_wrapper and @c boost::reference_wrapper.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
struct typer_traits_specialization<T,
                                   typename std::enable_if<
                                     runtime::is_reference_wrapper<T>::value
                                   >::type>
: public std::true_type
{ };

} // namespace stapl

#endif
