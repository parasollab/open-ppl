/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_REQUEST_PACKED_VALUE_HPP
#define STAPL_RUNTIME_REQUEST_PACKED_VALUE_HPP

#include "arg_storage.hpp"
#include "../exception.hpp"
#include "../message.hpp"

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Handles unpacking objects stored in an @ref arg_storage.
///
/// @tparam T Object type.
///
/// @warning If a @ref message or value is set, then @ref release() or
///          @ref get() has to be called to ensure proper resource reclamation.
///
/// @see arg_storage
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
struct packed_value
{
  using storage_type = arg_storage_t<T, T>;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Properly destroys the storage.
  //////////////////////////////////////////////////////////////////////
  struct deleter
  {
    packed_value& m_ref;

    constexpr explicit deleter(packed_value& p) noexcept
    : m_ref(p)
    { }

    ~deleter(void)
    {
      m_ref.m_storage->~storage_type();
      m_ref.release();
    }
  };

  storage_type* m_storage;
  void*         m_base;
  message*      m_msg;
  bool          m_ref_counted;

public:
  constexpr packed_value(void) noexcept
  : m_storage(nullptr),
    m_base(nullptr),
    m_msg(nullptr),
    m_ref_counted(false)
  { }

  packed_value(storage_type* const p,
               void* const base,
               message_ptr m) noexcept
  { set(p, base, std::move(m)); }

  packed_value(storage_type* const p,
               void* const base,
               message_shared_ptr& m) noexcept
  { set(p, base, m); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets the information required to unpack and retrieve an object.
  ///
  /// This function stores a pointer to the buffer where the object remains
  /// packed until @ref get() is called.
  ///
  /// @param p    Pointer to the @ref arg_storage in @p msg.
  /// @param base Address in @p msg that the @ref arg_storage was created in.
  /// @param msg  Buffer that contains the @ref arg_storage.
  ////////////////////////////////////////////////////////////////////
  void set(storage_type* const p,
           void* const base,
           message_ptr m) noexcept
  {
    STAPL_RUNTIME_ASSERT(p && base && m);
    m_storage     = p;
    m_base        = base;
    m_msg         = m.release();
    m_ref_counted = false;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets the information required to unpack and retrieve an object.
  ///
  /// This function stores a pointer to the buffer where the object remains
  /// packed until @ref get() is called.
  ///
  /// @warning The reference count of the buffer will be increased to protect
  ///          against accidental deletion.
  ///
  /// @param p    Pointer to the @ref arg_storage in @p msg.
  /// @param base Address in @p msg that the @ref arg_storage was created in.
  /// @param msg  Buffer that contains the @ref arg_storage.
  ////////////////////////////////////////////////////////////////////
  void set(storage_type* const p,
           void* const base,
           message_shared_ptr& m) noexcept
  {
    STAPL_RUNTIME_ASSERT(p && base && m);
    m_storage     = p;
    m_base        = base;
    m_msg         = m.get();
    m_ref_counted = true;
    intrusive_ptr_add_ref(m_msg);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets a value in packed form that will be unpacked through
  ///        @ref get().
  ///
  /// This function will create a new @ref message to store the value and stores
  /// a pointer to the buffer where the object remains packed until @ref get()
  /// is called.
  ///
  /// @param t Value to set.
  ////////////////////////////////////////////////////////////////////
  void set(T const& t)
  {
    std::size_t static_size = sizeof(storage_type);
    const std::size_t size  = (static_size + storage_type::packed_size(t));

    auto m  = message::create(size);
    char* p = static_cast<char*>(m->reserve(size));
    auto* a = new(p) storage_type{t, p, static_size};

    set(a, a, std::move(m));
  }

  ////////////////////////////////////////////////////////////////////
  /// @copydoc set_value(T const&)
  ////////////////////////////////////////////////////////////////////
  void set(T&& t)
  {
    std::size_t static_size = sizeof(storage_type);
    const std::size_t size  = (static_size +
                               storage_type::packed_size(std::move(t)));

    auto m  = message::create(size);
    char* p = static_cast<char*>(m->reserve(size));
    auto* a = new(p) storage_type{std::move(t), p, static_size};

    set(a, a, std::move(m));
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Unpacks and returns the packed object.
  ///
  /// @warning This function will clean-up after the unpacking and will decrease
  ///          the reference count of the buffer, which may lead to its
  ///          destruction.
  ////////////////////////////////////////////////////////////////////
  T get(void)
  {
    STAPL_RUNTIME_ASSERT(m_storage);
    deleter d{*this};
    return m_storage->get(m_base);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Releases the buffer without unpacking.
  ///
  /// @warning The reference count of the buffer will be decreased if it was
  ///          assigned through
  ///    @ref set(storage_type* const,void* const,message_shared_ptr&) noexcept,
  ///          which may lead to its destruction.
  ////////////////////////////////////////////////////////////////////
  void release(void) noexcept
  {
    if (m_ref_counted)
      intrusive_ptr_release(m_msg);
    else
      message::destroy(m_msg);
    m_storage     = nullptr;
    m_base        = nullptr;
    m_msg         = nullptr;
    m_ref_counted = false;
  }
};

} // namespace runtime

} // namespace stapl

#endif
