/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_P_OBJECT_HPP
#define STAPL_RUNTIME_SERIALIZATION_P_OBJECT_HPP

#include "pointer.hpp"
#include "../exception.hpp"
#include "../rmi_handle.hpp"
#include "../type_traits/is_p_object.hpp"
#include <cstring>
#include <type_traits>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref cloner for distributed object types.
///
/// @tparam T Object type to be cloned.
///
/// @warning Distributed objects (@ref p_object) cannot be cloned. This
///          specialization will throw a compile-time error.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
struct cloner<T,
              typename std::enable_if<is_p_object<T>::value>::type>
{
  static_assert(true, "p_objects cannot be cloned.");
};

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits for pointers to distributed
///        objects.
///
/// The packing/unpacking relies on a @ref rmi_handle::light_reference of the
/// object. While packed, the pointer to the object is transformed to a pointer
/// to a @ref rmi_handle::light_reference. Upon unpacking, it is transformed
/// back to a pointer to the object.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
class typer_traits<T*,
                   typename std::enable_if<
                     is_p_object<T>::value &&
                     !std::is_function<T>::value
                   >::type>
{
private:
  using handle_type = rmi_handle::const_light_reference;
  using traits_type = typer_traits<handle_type*>;
public:
  using value_type  = T*;

  static std::size_t packed_size(T* t, const std::size_t num = 1) noexcept
  {
    STAPL_RUNTIME_ASSERT_MSG(num<2,
                             "Packing of arrays of p_objects is not allowed.");

    if (!t || num==0)
      return 0;

    // temporary handle
    handle_type h  = t->get_rmi_handle();
    handle_type* p = &h;

    // size of pointer to temporary handle
    return traits_type::packed_size(p, num);
  }

  static std::pair<bool, std::size_t>
  meets_requirements(const typer::pass_type,
                     T* t,
                     const std::size_t num = 1) noexcept
  {
    if (!t || num==0)
      return std::make_pair(true, std::size_t(0));

    // non-null pointers to p_objects have to be marshaled, as the
    // transformation to rmi_handle::light_reference has to happen to retrieve
    // the correct p_object on the destination; consequently they cannot be
    // copied or moved
    return std::make_pair(false, packed_size(t, num));
  }

  static void prepack(T**, T**, const std::size_t = 1) noexcept
  { }

  static std::size_t pack(T*& dest,
                          void* base,
                          const std::size_t offset,
                          T* src,
                          const std::size_t num = 1) noexcept
  {
    STAPL_RUNTIME_ASSERT_MSG(num<2,
                             "Packing of arrays of p_objects is not allowed.");

    if (!src || num==0) {
      dest = nullptr;
      return 0;
    }

    // temporary handle
    handle_type h  = src->get_rmi_handle();
    handle_type* p = &h;

    // pack pointer to temporary handle
    const std::size_t s = traits_type::pack(p, base, offset, p, num);

    // save pointer to temporary handle in pointer to object space
    std::memcpy(&dest, &p, sizeof(p));
    return s;
  }

  static std::size_t unpack(T*& t, void* base, const std::size_t num = 1)
  {
    STAPL_RUNTIME_ASSERT_MSG(num<2,
                             "Packing of arrays of p_objects is not allowed.");

    if (!t || num==0)
      return 0;

    // retrieve pointer to temporary handle
    handle_type* p = nullptr;
    std::memcpy(&p, &t, sizeof(t));

    // unpack pointer to temporary handle
    const std::size_t s = traits_type::unpack(p, base, num);

    // find object and store its pointer; unpacking is done
    t = const_cast<T*>(std::addressof(get_p_object<T>(*p)));

    // clean-up after the temporary handle
    traits_type::destroy(p, num);

    return s;
  }

  static void destroy(T*, const std::size_t = 1) noexcept
  { }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits_specialization for pointers to
///        distributed objects.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
struct typer_traits_specialization<T*,
                                   typename std::enable_if<
                                     is_p_object<T>::value &&
                                     !std::is_function<T>::value
                                   >::type>
: public std::true_type
{ };

} // namespace stapl

#endif
