/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_P_OBJECT_HPP
#define STAPL_RUNTIME_P_OBJECT_HPP

#include "rmi_handle.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Implements the base class for distributed objects.
///
/// A user class can extend from @ref p_object to have automatic registration
/// and unregistration, as well as access to information describing the location
/// it was created in without the need to call the stand-alone functions.
///
/// @see rmi_handle
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
class p_object
{
public:
  using size_type = rmi_handle::size_type;

private:
  rmi_handle m_handle;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref p_object.
  ///
  /// @param flags Registration flags.
  //////////////////////////////////////////////////////////////////////
  explicit p_object(const unsigned int flags = 0)
  : m_handle(this, flags)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructs a new @ref p_object in the gang of the caller.
  ///
  /// @warning The new object is registered in the current gang, not in the
  ///          gang of @p other.
  //////////////////////////////////////////////////////////////////////
  p_object(p_object const& other)
  : m_handle(this, other.m_handle.get_flags())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Move constructs a new @ref p_object in the gang of the caller.
  ///
  /// @warning The new object is registered in the current gang, not in the
  ///          gang of @p other.
  //////////////////////////////////////////////////////////////////////
  p_object(p_object&& other)
  : m_handle(this, other.m_handle.get_flags())
  { }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Unregisters and destroys this @ref p_object.
  //////////////////////////////////////////////////////////////////////
  virtual ~p_object(void) = default;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Assigns @p other to @p *this.
  ///
  /// @warning The gang associated with @p *this does not change.
  //////////////////////////////////////////////////////////////////////
  p_object& operator=(p_object const& other) noexcept
  {
    m_handle.set_flags(other.m_handle.get_flags());
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Move assigns @p other to @p *this.
  ///
  /// @warning The gang associated with @p *this does not change.
  //////////////////////////////////////////////////////////////////////
  p_object& operator=(p_object&& other) noexcept
  {
    m_handle.set_flags(other.m_handle.get_flags());
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the location metadata of the location this object
  ///        registered in.
  //////////////////////////////////////////////////////////////////////
  runtime::location_md const& get_location_md(void) const noexcept
  { return m_handle.get_location_md(); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc get_location_md() const noexcept
  //////////////////////////////////////////////////////////////////////
  runtime::location_md& get_location_md(void) noexcept
  { return m_handle.get_location_md(); }

  /// @name Distributed Object Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the associated @ref rmi_handle.
  //////////////////////////////////////////////////////////////////////
  rmi_handle::const_reference const& get_rmi_handle(void) const noexcept
  { return m_handle; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the associated @ref rmi_handle.
  //////////////////////////////////////////////////////////////////////
  rmi_handle::reference const& get_rmi_handle(void) noexcept
  { return m_handle; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the location id of the local sub-object.
  //////////////////////////////////////////////////////////////////////
  size_type get_location_id(void) const noexcept
  { return m_handle.get_location_id(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of locations of the gang of this @ref p_object.
  //////////////////////////////////////////////////////////////////////
  size_type get_num_locations(void) const noexcept
  { return m_handle.get_num_locations(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Advances the epoch of the object.
  ///
  /// Advancing the epoch will flush any pending RMIs. It will also increase the
  /// epoch of the current gang if the object is not a named object.
  //////////////////////////////////////////////////////////////////////
  void advance_epoch(void)
  { m_handle.advance_epoch(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used by @p lock_guard functionality in method of derived
  /// classes to request atomicity with respect to incoming RMIs.
  ///
  /// Mutates counter in the associated runqueue.
  //////////////////////////////////////////////////////////////////////
  void unlock(void)
  { m_handle.unlock(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used by @p lock_guard functionality in method of derived
  /// classes to request atomicity with respect to incoming RMIs.
  ///
  /// Mutates counter in the associated runqueue.
  //////////////////////////////////////////////////////////////////////
  void lock(void)
  { m_handle.lock(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Attempt to maintain atomicity with respect to incoming RMIs.
  ///
  /// Mutates counter in the associated runqueue.
  /// @return @c true if the lock was able to be acquired. @c false if it
  /// is already locked.
  //////////////////////////////////////////////////////////////////////
  bool try_lock(void)
  { return m_handle.try_lock(); }

  /// @}
};

} // namespace stapl

#endif
