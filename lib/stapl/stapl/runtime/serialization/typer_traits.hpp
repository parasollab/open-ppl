/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_TYPER_TRAITS_HPP
#define STAPL_RUNTIME_SERIALIZATION_TYPER_TRAITS_HPP

#include "typer_fwd.hpp"
#include <cstring>
#include <type_traits>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Provides methods for assisting in packing and unpacking objects.
///
/// @tparam T Object type to be packed.
///
/// It has functions to determine the packing requirements, additional packed
/// size requirements (e.g., objects that contain pointers require extra space
/// to serialize the data pointed to), and implements the packing and
/// unpacking functions.
///
/// The default @ref typer_traits uses the class's @c define_type().
///
/// @see typer
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T,
         typename Enable>
class typer_traits
{
public:
  using value_type = T;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the packed (dynamic) size of the given object.
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T const& t) noexcept
  {
    typer ct{typer::SIZE};
    runtime::define_type_cast(t).define_type(ct);
    return ct.offset();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the packing requirements for pass @p p.
  //////////////////////////////////////////////////////////////////////
  static std::pair<bool, std::size_t>
  meets_requirements(const typer::pass_type p, T const& t) noexcept
  {
    typer ct{std::is_move_constructible<T>::value ? p : typer::COPY};
    runtime::define_type_cast(t).define_type(ct);
    return ct.meets_requirements();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Performs any operations that need to be done before calling
  ///        @ref typer_traits::pack().
  ///
  /// The default version does a memcpy from source object to destination,
  /// effectively packing the static part of the object.
  ///
  /// @param dest Destination to pack the object to.
  /// @param src  Source to pack the object from.
  /// @param num  Number of objects to pack.
  //////////////////////////////////////////////////////////////////////
  static void prepack(T* dest,
                      T const* src,
                      const std::size_t num = 1) noexcept
  {
    // cast to avoid warnings for doing a memcpy when T has a vtable
    std::memcpy(static_cast<void*>(dest),
                static_cast<void const*>(src),
                (sizeof(T) * num));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Packs the object to the given buffer, returning its packed size.
  ///
  /// This function takes care of the dynamic part of the object.
  ///
  /// @param dest   Destination to pack the object to.
  /// @param base   Start of the buffer where the dynamic part of the object
  ///               will be written to.
  /// @param offset Offset from the base to write the dynamic part of the
  ///               object.
  /// @param src    Source to pack the object from.
  ///
  /// @return The dynamic size of the object in bytes.
  //////////////////////////////////////////////////////////////////////
  static std::size_t pack(T& dest,
                          void* base,
                          const std::size_t offset,
                          T const& src) noexcept
  {
    typer ct{dest, src, base, offset};
    runtime::define_type_cast(dest).define_type(ct);
    return (ct.offset() - offset); // actual packed size is needed, not total
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Unpacks the object from the buffer, returning its packed size.
  ///
  /// @param t    Object to be unpacked.
  /// @param base Start of the buffer where the dynamic part of the object was
  ///             written to.
  ///
  /// @return The dynamic size of the object in bytes.
  //////////////////////////////////////////////////////////////////////
  static std::size_t unpack(T& t, void* base)
  {
    typer ct{base};
    runtime::define_type_cast(t).define_type(ct);
    return ct.offset();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Cleans up after the unpacked object.
  ///
  /// This function will recursively call all the @c destroy() functions to
  /// release any memory allocated while unpacking.
  ///
  /// @param t Object to clean up after it.
  //////////////////////////////////////////////////////////////////////
  static void destroy(T& t) noexcept
  {
    typer ct{typer::DESTROY};
    runtime::define_type_cast(t).define_type(ct);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Traits class to detect which @ref typer_traits is a specialization
///        and which is the default.
///
/// The default @ref typer_traits inherits from @c false_type.
///
/// @see typer_traits, runtime::supports_stapl_packing
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T, typename Enable>
struct typer_traits_specialization
: public std::false_type
{ };

} // namespace stapl

#endif
