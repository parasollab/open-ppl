/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_OBJECT_STORAGE_HPP
#define STAPL_RUNTIME_SERIALIZATION_OBJECT_STORAGE_HPP

#include "../config.hpp"
#include "typer_traits.hpp"
#include "../type_traits/aligned_storage.hpp"
#include "../type_traits/is_p_object.hpp"
#include <type_traits>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Storage for objects used in communication over shared memory.
///
/// @tparam T Object type.
///
/// This class uses a properly aligned @c char array to store an object of type
/// @p T. Determined by @ref typer_traits::meets_requirements(), the object is
/// either copied or moved in this array. Otherwise, the static part of the
/// object is copied in the array, while the dynamic part (e.g. any heap
/// allocated space that a member of @p T points to) will be copied in the extra
/// space during packing in @ref construct().
///
/// @warning One of the @ref destroy() or @ref destroy_packed() functions has to
///          be called prior to the destructor to avoid memory leaks.
///
/// @warning There can be only a single call to the @ref get(), @ref moveout(),
///          @ref get(void*const) or @ref get(void*const,std::size_t&)
///          functions.
///
/// @see packed_object_storage
/// @ingroup serialization
///
/// @todo Cache the result of @ref meets_requirements() to avoid calling at both
///       @ref packed_size() and @ref construct() to determine if packing can be
///       elided.
//////////////////////////////////////////////////////////////////////
template<typename T>
class object_storage
{
private:
  using value_type  = typename std::remove_cv<T>::type;
  using traits_type = typer_traits<value_type>;

  static_assert(!is_p_object<T>::value, "p_objects cannot be packed.");

  union
  {
    /// Stored object.
    typename std::aligned_storage<
      sizeof(T),
      std::alignment_of<T>::value
    >::type m_storage;

    /// @todo Remove when correct alignment is enforced for messages.
    aligned_storage_t<sizeof(T)> m_force_alignment;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the packing requirements for moving @p t.
  //////////////////////////////////////////////////////////////////////
  static std::pair<bool, std::size_t> meets_requirements(T&& t) noexcept
  {
    return traits_type::meets_requirements(typer::MOVE,
                                           const_cast<value_type&>(t));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the packing requirements for copying @p t.
  //////////////////////////////////////////////////////////////////////
  static std::pair<bool, std::size_t> meets_requirements(T const& t) noexcept
  {
    return traits_type::meets_requirements(typer::COPY,
                                           const_cast<value_type&>(t));
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the packed size of @p t.
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T&& t) noexcept
  {
    const auto r = meets_requirements(std::move(t));
    return (r.first ? 0 : aligned_size(r.second));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the packed size of @p t.
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T const& t) noexcept
  {
    const auto r = meets_requirements(t);
    return (r.first ? 0 : aligned_size(r.second));
  }

  object_storage(void) noexcept = default;
  object_storage(object_storage const&) = delete;
  object_storage& operator=(object_storage const&) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes this object by moving @p t in it.
  ///
  /// @warning This function can only be called if the object has been default
  ///          constructed.
  ///
  /// @return @c true if @p t was moved, otherwise @c false.
  //////////////////////////////////////////////////////////////////////
  bool construct(T&& t, void* const base, std::size_t& size)
  {
    const bool moved = meets_requirements(std::move(t)).first;
    if (moved) {
      ::new(&m_storage) T(std::move(t));
    }
    else {
      auto* p = reinterpret_cast<value_type*>(&m_storage);
      traits_type::prepack(p, std::addressof(const_cast<value_type&>(t)));
      const std::size_t s =
        traits_type::pack(*p, base, size, const_cast<value_type&>(t));
      size += aligned_size(s);
    }
    return moved;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes this object by copying @p t in it.
  ///
  /// @warning This function can only be called if the object has been default
  ///          constructed.
  ///
  /// @return @c true if @p t was copied, otherwise @c false.
  //////////////////////////////////////////////////////////////////////
  bool construct(T const& t, void* const base, std::size_t& size)
  {
    const bool copied = meets_requirements(t).first;
    if (copied) {
      ::new(&m_storage) T(t);
    }
    else {
      auto* p = reinterpret_cast<value_type*>(&m_storage);
      traits_type::prepack(p, std::addressof(const_cast<value_type&>(t)));
      const std::size_t s =
        traits_type::pack(*p, base, size, const_cast<value_type&>(t));
      size += aligned_size(s);
    }
    return copied;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroys the internal storage as if it holds a moved or copied
  ///        object.
  //////////////////////////////////////////////////////////////////////
  void destroy(void) noexcept
  {
    auto* p = reinterpret_cast<T*>(&m_storage);
    p->~T();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroys the internal storage as if it holds a packed object.
  //////////////////////////////////////////////////////////////////////
  void destroy_packed(void) noexcept
  {
    auto* p = reinterpret_cast<value_type*>(&m_storage);
    traits_type::destroy(*p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the moved or copied object.
  //////////////////////////////////////////////////////////////////////
  T& get(void) noexcept
  { return *reinterpret_cast<T*>(&m_storage); }

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

  //////////////////////////////////////////////////////////////////////
  /// @brief Moves the object out if it was moved or copied in.
  //////////////////////////////////////////////////////////////////////
  T moveout(void)
  { return std::move(*reinterpret_cast<T*>(&m_storage)); }
};

} // namespace runtime

} // namespace stapl

#endif
