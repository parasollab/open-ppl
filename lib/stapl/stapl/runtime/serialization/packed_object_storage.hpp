/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_PACKED_OBJECT_STORAGE_HPP
#define STAPL_RUNTIME_SERIALIZATION_PACKED_OBJECT_STORAGE_HPP

#include "../config.hpp"
#include "typer_traits.hpp"
#include "../type_traits/aligned_storage.hpp"
#include "../type_traits/is_p_object.hpp"
#include <type_traits>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Storage for object of type @p T used in communication.
///
/// @tparam T Object type.
///
/// This class uses a properly aligned @c char array to store an object of type
/// @p T. The static part of the object is copied in the array, while the
/// dynamic part (e.g. any heap allocated space that a member of @p T points to)
/// will be copied in the extra space during packing in the constructor.
///
/// An explicit call to @ref get() is necessary to retrieve the stored object.
///
/// @warning Calling the destructor after @ref get() is necessary to avoid
///          memory leaks.
///
/// @warning There can be only a single call to the @ref get(void*const) and
///          @ref get(void*const,std::size_t&) functions.
///
/// @see typer_traits
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T,
         bool Empty = (std::is_empty<T>::value ||
                       std::is_same<T, std::nullptr_t>::value)>
class packed_object_storage
{
private:
  using value_type  = typename std::remove_cv<T>::type;
  using traits_type = typer_traits<value_type>;

  static_assert(!is_p_object<T>::value, "p_objects cannot be packed.");

  union
  {
    /// Stored object.
    typename std::aligned_storage<
      sizeof(value_type),
      std::alignment_of<value_type>::value
    >::type m_storage;

    /// @todo Remove when correct alignment is enforced for messages.
    aligned_storage_t<sizeof(value_type)> m_force_alignment;
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the packed size of @p t.
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T const& t) noexcept
  { return aligned_size(traits_type::packed_size(const_cast<value_type&>(t))); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new object by packing @p t in it.
  ///
  /// This constructor will copy the dynamic part of the object at
  /// <tt>base + size</tt>, which should be big enough to fit it.
  ///
  /// @param t    Object to pack.
  /// @param base Pointer to buffer start where the dynamic part of @p t can be
  ///             stored in.
  /// @param size Offset from @p base where space is available.
  //////////////////////////////////////////////////////////////////////
  packed_object_storage(T const& t,
                        void* const base,
                        std::size_t& size) noexcept
  {
    auto* p = reinterpret_cast<value_type*>(&m_storage);
    traits_type::prepack(p, std::addressof(const_cast<value_type&>(t)));
    const std::size_t s =
      traits_type::pack(*p, base, size, const_cast<value_type&>(t));
    size += aligned_size(s);
  }

  packed_object_storage(packed_object_storage const&) = delete;
  packed_object_storage& operator=(packed_object_storage const&) = delete;

  ~packed_object_storage(void)
  {
    auto* p = reinterpret_cast<value_type*>(&m_storage);
    traits_type::destroy(*p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the stored object after unpacking it.
  ///
  /// @param base Buffer where the dynamic part of the object is stored.
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base)
  {
    auto* p = reinterpret_cast<value_type*>(&m_storage);
    traits_type::unpack(*p, base);
    return *p;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the stored object after unpacking it.
  ///
  /// @param base Buffer where the dynamic part of the object is stored.
  /// @param size Variable to store how many bytes were unpacked.
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base, std::size_t& size)
  {
    auto* p = reinterpret_cast<value_type*>(&m_storage);
    const std::size_t s = traits_type::unpack(*p, base);
    size += aligned_size(s);
    return *p;
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref packed_object_storage for empty classes and
///        @c std::nullptr_t.
///
/// In order to avoid extra space for empty classes,
/// @ref get(void*const,std::size_t) casts the pointer to the buffer as a
/// pointer to @p T and dereferences it.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
class packed_object_storage<T, true>
{
public:
  static constexpr std::size_t packed_size(T const&) noexcept
  { return 0; }

  template<typename U>
  constexpr packed_object_storage(U&&, void* const, std::size_t) noexcept
  { }

  packed_object_storage(packed_object_storage const&) = delete;
  packed_object_storage& operator=(packed_object_storage const&) = delete;

  T& get(void* const base, std::size_t = 0) const noexcept
  { return *static_cast<T*>(base); }
};

} // namespace runtime

} // namespace stapl

#endif
