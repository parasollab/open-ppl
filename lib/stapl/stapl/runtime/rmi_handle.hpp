/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_HANDLE_HPP
#define STAPL_RUNTIME_RMI_HANDLE_HPP

#include "rmi_handle_fwd.hpp"
#include "context.hpp"
#include "p_object_registry.hpp"
#include "type_traits/p_object_ptr_cast.hpp"
#include <utility>

namespace stapl {

// Returns the location id this object registered in
inline rmi_handle::size_type rmi_handle::get_location_id(void) const noexcept
{
  return m_location->get_id();
}


// Registers an object in the location of the given execution context
template<typename T>
inline void rmi_handle::register_object(runtime::context& ctx,
                                        T* const t,
                                        const unsigned int flags)
{
  using namespace runtime;

  STAPL_RUNTIME_ASSERT(!this->valid() && ctx.is_base());

  m_location    = &(ctx.get_location_md());
  this->m_flags = flags;
  this->m_nlocs = m_location->get_gang_md().size();

  if ((this->m_nlocs==1) && ((flags & allow_try_rmi)==0)) {
    static_cast<rmi_handle_info&>(*this) =
      rmi_handle_info{m_location->register_object(t, true),
                      m_location->get_gang_md().get_id()};
  }
  else {
    ctx.flush_requests(); // epoch will change, flush aggregated requests
    static_cast<rmi_handle_info&>(*this) =
      rmi_handle_info{m_location->register_object(t, false),
                      m_location->get_gang_md().get_id()};
  }

  if (get_debug_level() > 0) {
    p_object_registry::register_object(this, t, typeid(*t));
  }
}


// Unregisters the object.
inline void rmi_handle::unregister_object(void)
{
  using namespace runtime;

  if (!this->valid())
    return;

  if (get_debug_level() > 0) {
    p_object_registry::unregister_object(this);
  }

  m_location->unregister_object(this->internal_handle());
  m_location = nullptr;
  static_cast<rmi_handle_base&>(*this) = rmi_handle_base{};
}


// Advances the epoch of the object.
inline void rmi_handle::advance_epoch(void)
{
  using namespace runtime;

  auto& ctx = this_context::get();
  STAPL_RUNTIME_ASSERT(this->valid() && ctx.is_base());

  if (this->m_nlocs>1) {
    // epoch advance has repercussions only for gangs with >1 location
    ctx.flush_requests();
    const epoch_type e = m_location->advance_epoch();
    this->set_epoch(e);
  }
}

// Reenable processing of incoming RMIs on this location.
inline void rmi_handle::unlock(void)
{
  STAPL_RUNTIME_ASSERT(this->valid());

  m_location->undefer_requests();
}

// Disable processing of incoming RMIs on this location so that local
// computation is guaranteed to run atomically with respect to incoming
// requests.
inline void rmi_handle::lock(void)
{
  STAPL_RUNTIME_ASSERT(this->valid());

  m_location->defer_requests();
}

// Attempt to disable processing of incoming RMIs on this location so that
// local computation is guaranteed to run atomically with respect to incoming
// requests. If successful, return true. Otherwise return false
inline bool rmi_handle::try_lock(void)
{
  STAPL_RUNTIME_ASSERT(this->valid());

  return m_location->try_defer_requests();
}

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Returns a reference to the object that @p h refers to.
///
/// @ingroup runtimeUtility
///
/// @todo If the @ref p_object is in a future epoch, then delay unpacking but do
///       not throw an error.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Handle>
T& retrieve_object(Handle&& h, location_md& l) noexcept
{
  if (!h.valid())
    STAPL_RUNTIME_ERROR("Invalid handle.");

  if (h.get_gang_id()!=l.get_gang_md().get_id())
    STAPL_RUNTIME_ERROR("Reference to p_object cannot be retrieved outside the "
                        "gang it was created.");

  auto p = l.get_object(h.internal_handle(), h.get_registration_epoch());
  if (!p) {
    if (h.get_epoch()>l.get_epoch())
      STAPL_RUNTIME_ERROR("p_object is in a future epoch.");
    STAPL_RUNTIME_ERROR("p_object does not exist.");
  }

  if (get_debug_level() > 0) {
    using U = typename std::conditional<
                std::is_base_of<p_object, T>::value, p_object, T>::type;
    p_object_registry::verify_object_type(p, typeid(U));
  }

  return *p_object_ptr_cast<T>(p);
}


//////////////////////////////////////////////////////////////////////
/// @brief Attempts to return a pointer to the object that @p h refers to.
///
/// This function will return a pointer to the object even if the requesting
/// code executes in a gang other than the one that the object was registered
/// in. The only requirement is that the location the object registered in and
/// the location that the caller executes in are scheduled on the same thread.
///
/// @warning If the function is given an invalid handle, it will return
///          @c nullptr.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T, typename Handle>
T* try_retrieve_object(Handle&& h, location_md* const l) noexcept
{
  if (!l || (h.get_epoch()>l->get_epoch())) {
    // location metadata does not exist (e.g location on different thread) or
    // handle is invalid (epoch of invalid always less than any other epoch) or
    // object is in a future epoch
    return nullptr;
  }

  auto p = l->get_object(h.internal_handle(), h.get_registration_epoch());
  if (!p)
    return nullptr;

  if (get_debug_level() > 0) {
    using U = typename std::conditional<
                std::is_base_of<p_object, T>::value, p_object, T>::type;
    p_object_registry::verify_object_type(p, typeid(U));
  }

  return p_object_ptr_cast<T>(p);
}


//////////////////////////////////////////////////////////////////////
/// @brief Packs @ref rmi_handle::reference and @ref rmi_handle::const_reference
///        for use in requests.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class packed_handle
{
private:
  rmi_handle::internal_handle_type m_handle;

public:
  template<typename Handle>
  explicit packed_handle(Handle const& h) noexcept
  : m_handle(h.internal_handle())
  { }

  template<typename T>
  T* get(location_md& l) const noexcept
  {
    auto p = l.get_object(m_handle);
    if (get_debug_level() > 0) {
      using U = typename std::conditional<
                  std::is_base_of<p_object, T>::value, p_object, T>::type;
      p_object_registry::verify_object_type(p, typeid(U));
    }
    return p_object_ptr_cast<T>(p);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Packs @ref rmi_handle::reference and @ref rmi_handle::const_reference
///        and the epoch for use in requests.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class packed_handle_epoch
{
private:
  rmi_handle::internal_handle_type m_handle;
  const rmi_handle::epoch_type     m_registration_epoch;

public:
  template<typename Handle>
  explicit packed_handle_epoch(Handle const& h) noexcept
  : m_handle(h.internal_handle()),
    m_registration_epoch(h.get_registration_epoch())
  { }

  template<typename T>
  T* get(location_md& l) const noexcept
  {
    auto p = l.get_object(m_handle, m_registration_epoch);
    if (get_debug_level() > 0) {
      using U = typename std::conditional<
                  std::is_base_of<p_object, T>::value, p_object, T>::type;
      p_object_registry::verify_object_type(p, typeid(U));
    }
    return p_object_ptr_cast<T>(p);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Default handle type for requests.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
using packed_handle_type =
#ifdef STAPL_RUNTIME_DEBUG
 packed_handle_epoch;
#else
 packed_handle;
#endif

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Returns a reference to the object registered with the @ref rmi_handle
///        that @p h refers to.
///
/// This function can only return references to objects that have been
/// registered in the same gang that the caller executes in.
///
/// @param h Reference to an @ref rmi_handle.
///
/// @return A reference to the object associated with the @ref rmi_handle.
///
/// @warning Requesting an object that has been unregistered or was never
///          registered is undefined behavior.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
T const& get_p_object(rmi_handle::const_reference const& h)
{
  using namespace runtime;
  return retrieve_object<T>(h, this_context::get().get_location_md());
}


//////////////////////////////////////////////////////////////////////
/// @copydoc get_p_object()
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
T const& get_p_object(rmi_handle::const_light_reference const& h)
{
  using namespace runtime;
  return retrieve_object<T>(h, this_context::get().get_location_md());
}


//////////////////////////////////////////////////////////////////////
/// @copydoc get_p_object()
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
T& get_p_object(rmi_handle::reference const& h)
{
  using namespace runtime;
  return retrieve_object<T>(h, this_context::get().get_location_md());
}


//////////////////////////////////////////////////////////////////////
/// @copydoc get_p_object()
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
T& get_p_object(rmi_handle::light_reference const& h)
{
  using namespace runtime;
  return retrieve_object<T>(h, this_context::get().get_location_md());
}



//////////////////////////////////////////////////////////////////////
/// @brief Attempts to return a pointer to the object registered with the
///        @ref rmi_handle that @p h refers to.
///
/// This function will return a pointer to the object even if the requesting
/// code executes in a gang other than the one that the object was registered
/// in. The only requirement is that the location the object registered in and
/// the location that the caller executes in are scheduled on the same thread.
///
/// @warning If @p h is an invalid handle, @c nullptr is returned instead of an
///          error.
///
/// @param h Reference to an @ref rmi_handle.
///
/// @return A pointer to the object associated with the @ref rmi_handle.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
T const* resolve_handle(rmi_handle::const_reference const& h) noexcept
{
  using namespace runtime;
  location_md* const l = this_context::try_get_location_md(h.get_gang_id());
  return try_retrieve_object<T>(h, l);
}


//////////////////////////////////////////////////////////////////////
/// @copydoc resolve_handle()
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
T* resolve_handle(rmi_handle::reference const& h) noexcept
{
  using namespace runtime;
  location_md* const l = this_context::try_get_location_md(h.get_gang_id());
  return try_retrieve_object<T>(h, l);
}


//////////////////////////////////////////////////////////////////////
/// @copydoc resolve_handle()
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
T const* resolve_handle(rmi_handle::const_light_reference const& h) noexcept
{
  using namespace runtime;
  location_md* const l = this_context::try_get_location_md(h.get_gang_id());
  return try_retrieve_object<T>(h, l);
}


//////////////////////////////////////////////////////////////////////
/// @copydoc resolve_handle()
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
T* resolve_handle(rmi_handle::light_reference const& h) noexcept
{
  using namespace runtime;
  location_md* const l = this_context::try_get_location_md(h.get_gang_id());
  return try_retrieve_object<T>(h, l);
}

} // namespace stapl

#endif
