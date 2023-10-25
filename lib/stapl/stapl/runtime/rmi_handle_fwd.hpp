/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_HANDLE_FWD_HPP
#define STAPL_RUNTIME_RMI_HANDLE_FWD_HPP

#include "config.hpp"
#include "exception.hpp"
#include "this_context.hpp"
#include "instrumentation.hpp"
#include "serialization_fwd.hpp"
#include "utility/logical_clock.hpp"
#include <climits>
#include <functional>
#include <iosfwd>
#include <sstream>
#include <string>
#include <utility>
#include <stapl/utility/hash_fwd.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Flags for registering objects.
///
/// @see rmi_handle, p_object
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
enum rmi_handle_flags
{
  /// Disables aggregation for the object.
  no_aggregation       = 0x1,

  /// Disables fence bookkeeping for the object.
  no_fence_information = 0x2,

  /// Enables support for @ref try_rmi().
  allow_try_rmi        = 0x4
};


class rmi_handle;


namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Distributed object virtual address.
///
/// This class abstracts the address of a distributed object.
///
/// @see rmi_handle
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class object_virtual_address
{
public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Internal handle type.
  //////////////////////////////////////////////////////////////////////
  enum address_type
  {
    /// Virtual address not associated with an object.
    UNKNOWN = 0x0,
    /// Direct address to the registered object.
    DIRECT,
    /// Indirect address to the registered object through an @ref spmd_registry.
    INDIRECT
  };

  using member_types = std::tuple<address_type>;

private:
  union
  {
    void*      m_direct;
    object_id  m_indirect;
  };
  address_type m_type;

public:
  constexpr object_virtual_address(void) noexcept
  : m_direct(nullptr),
    m_type(UNKNOWN)
  { }

  constexpr explicit object_virtual_address(void* const p) noexcept
  : m_direct(p),
    m_type(DIRECT)
  { }

  constexpr explicit object_virtual_address(const object_id h) noexcept
  : m_indirect(h),
    m_type(INDIRECT)
  { }

  constexpr bool valid(void) const noexcept
  { return (m_type!=UNKNOWN); }

  constexpr address_type get_type(void) const noexcept
  { return m_type; }

  void* direct(void) const noexcept
  {
    STAPL_RUNTIME_ASSERT(m_type==DIRECT);
    return m_direct;
  }

  object_id indirect(void) const noexcept
  {
    STAPL_RUNTIME_ASSERT(m_type==INDIRECT);
    return m_indirect;
  }

  std::size_t abstract(void) const noexcept
  {
    switch (m_type) {
      case DIRECT:
        return std::uintptr_t(direct());
      case INDIRECT:
        return indirect();
      default:
        STAPL_RUNTIME_ERROR("Cannot generate abstract representation.");
    }
    return 0;
  }

  std::size_t hash_code(void) const noexcept
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, get_type());
    switch (get_type()) {
      case DIRECT:
        boost::hash_combine(seed, direct());
        break;
      case INDIRECT:
        boost::hash_combine(seed, indirect());
        break;
      case UNKNOWN:
        break;
    }
    return seed;
  }

  friend constexpr bool operator==(object_virtual_address const& x,
                                   object_virtual_address const& y) noexcept
  {
    return (x.get_type()==y.get_type() &&
            (x.get_type()==UNKNOWN                             ||
            (x.get_type()==DIRECT   && x.direct()==y.direct()) ||
            (x.get_type()==INDIRECT && x.indirect()==y.indirect())));
  }

  friend constexpr bool operator!=(object_virtual_address const& x,
                                   object_virtual_address const& y) noexcept
  {
    return !(x==y);
  }

  friend std::ostream& operator<<(std::ostream&, object_virtual_address const&);
};


//////////////////////////////////////////////////////////////////////
/// @brief Distributed object handle information.
///
/// This class provides basic information about a registered distributed object,
/// such as the @ref object_virtual_address, the registration epoch, the current
/// epoch and the gang id.
///
/// This class is the basis of the implementation of
/// @ref rmi_handle::light_reference and @ref rmi_handle::const_light_reference.
/// It is also the base information class for @ref rmi_handle,
/// @ref rmi_handle::reference and @ref rmi_handle::const_reference.
///
/// @see rmi_handle
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class rmi_handle_info
{
public:
  using internal_handle_type = object_virtual_address;
  using epoch_type           = logical_clock::time_type;

private:
  internal_handle_type m_internal_handle;
  epoch_type           m_registration_epoch;
  epoch_type           m_epoch;
  gang_id              m_gid;

public:
  constexpr rmi_handle_info(void) noexcept
  : m_registration_epoch{logical_clock::no_time},
    m_epoch{logical_clock::no_time},
    m_gid{invalid_gang_id}
  { }

  constexpr
  rmi_handle_info(std::pair<object_virtual_address, epoch_type> const& p,
                  const gang_id gid) noexcept
  : m_internal_handle{p.first},
    m_registration_epoch{p.second},
    m_epoch{p.second},
    m_gid{gid}
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if this handle is associated with an object.
  //////////////////////////////////////////////////////////////////////
  constexpr bool valid(void) const noexcept
  { return m_internal_handle.valid(); }

  constexpr internal_handle_type const& internal_handle(void) const noexcept
  { return m_internal_handle; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the epoch that the object was registered in.
  //////////////////////////////////////////////////////////////////////
  constexpr epoch_type get_registration_epoch(void) const noexcept
  { return m_registration_epoch; }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the epoch that the object is currently in.
  //////////////////////////////////////////////////////////////////////
  void set_epoch(epoch_type e) noexcept
  { m_epoch = e; }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the epoch that the object is currently in.
  //////////////////////////////////////////////////////////////////////
  constexpr epoch_type get_epoch(void) const noexcept
  { return m_epoch; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the gang id of the gang this object registered in.
  //////////////////////////////////////////////////////////////////////
  constexpr gang_id get_gang_id(void) const noexcept
  { return m_gid; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a unique id of the registered object.
  ///
  /// @warning This should not be used to deduce locality information about a
  ///          handle.
  //////////////////////////////////////////////////////////////////////
  std::string get_uid(void) const
  {
    if (!valid())
      return std::string{"invalid"};
    std::ostringstream os;
    os << 'G' << m_gid
       << 'O' << m_internal_handle
       << 'R' << m_registration_epoch;
    return os.str();
  }

  void define_type(typer& t)
  {
    t.member(m_internal_handle);
    t.member(m_registration_epoch);
    t.member(m_epoch);
    t.member(m_gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true of the @p x and @p y are associated with the same
  ///        registered object.
  ///
  /// @warning While this function compares the epoch the object was registered
  ///          in, it does not compare the object's current epoch.
  //////////////////////////////////////////////////////////////////////
  friend constexpr bool operator==(rmi_handle_info const& x,
                                   rmi_handle_info const& y) noexcept
  {
    return (x.m_internal_handle==y.m_internal_handle       &&
            x.m_registration_epoch==y.m_registration_epoch &&
            x.m_gid==y.m_gid);
  }

  friend constexpr bool operator!=(rmi_handle_info const& x,
                                   rmi_handle_info const& y) noexcept
  {
    return !(x==y);
  }

  friend std::ostream& operator<<(std::ostream&, rmi_handle_info const&);
};


//////////////////////////////////////////////////////////////////////
/// @brief Distributed object base handle.
///
/// This class provides information about a registered distributed object, such
/// as the registration id, the registration epoch, the current epoch,
/// communication related flags, gang id and number of locations in the gang.
///
/// This class is also the implementation of @ref rmi_handle::const_reference.
///
/// @see rmi_handle
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class rmi_handle_base
: public rmi_handle_info
{
public:
  using size_type = location_id;

private:
  unsigned int m_flags;
  size_type    m_nlocs;

  friend class stapl::rmi_handle;

public:
  constexpr rmi_handle_base(void) noexcept
  : m_flags(0),
    m_nlocs(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the flags for this object.
  //////////////////////////////////////////////////////////////////////
  void set_flags(const unsigned int flags) noexcept
  { m_flags = flags; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the flags of this object.
  //////////////////////////////////////////////////////////////////////
  constexpr unsigned int get_flags(void) const noexcept
  { return m_flags; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the size of the gang this object registered in.
  //////////////////////////////////////////////////////////////////////
  constexpr size_type get_num_locations(void) const noexcept
  { return m_nlocs; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns if the given location id is valid for the gang this object
  ///        registered in.
  //////////////////////////////////////////////////////////////////////
  constexpr bool is_valid(const size_type n) const noexcept
  { return (n<get_num_locations()); }

  void define_type(typer& t)
  {
    t.base<rmi_handle_info>(*this);
    t.member(m_flags);
    t.member(m_nlocs);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Extended distributed object handle information.
///
/// This class is the implementation of @ref rmi_handle::reference. This class
/// provides the same information as @ref rmi_handle_base but allows to
/// differentiate between @ref rmi_handle::reference and
/// @ref rmi_handle::const_reference.
///
/// @see rmi_handle
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class rmi_handle_extended_base
: public rmi_handle_base
{
public:
  void define_type(typer& t)
  { t.base<rmi_handle_base>(*this); }
};


class location_md;

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Provides a registration mechanism for RMI communication.
///
/// It is the user's responsibility to register an object with the handle.
/// Trying to communicate with an unregistered object or calling any other
/// function except @ref rmi_handle::register_object() and
/// @ref rmi_handle::valid() is undefined behavior.
///
/// Registration assumes an SPMD model. Specifically, when an object, comprised
/// of one sub-object per location, tries to register itself, the model expects
/// each location to perform its sub-object registrations in the same order,
/// hence allowing each corresponding sub-object to have the same id. It is a
/// local operation.
///
/// This assumption alleviates concerns with scoping and forcing unique names
/// for objects, and allows for faster address translation.
///
/// Unregistration is not required to be SPMD, however all the sub-objects of a
/// registered object have to unregister before an @ref rmi_fence().
///
/// Registration has amortized \f$O(1)\f$ time complexity whereas unregistration
/// has \f$O(1)\f$.
///
/// @see p_object
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
class rmi_handle
: public runtime::rmi_handle_extended_base
{
public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Const reference to an object that was registered through
  ///        @ref rmi_handle.
  ///
  /// It can be constructed from any reference to @ref rmi_handle, except
  /// @ref rmi_handle::light_reference and
  /// @ref rmi_handle::const_light_reference.
  ///
  /// Once an @ref rmi_handle::const_reference is created, one can retrieve a
  /// @c const reference to the object and do RMIs to const functions through
  /// it.
  ///
  /// The size of an @ref rmi_handle::const_reference is bigger than that of an
  /// @ref rmi_handle::const_light_reference.
  //////////////////////////////////////////////////////////////////////
  using const_reference = runtime::rmi_handle_base;

  //////////////////////////////////////////////////////////////////////
  /// @brief Reference to an object that was registered through @ref rmi_handle.
  ///
  /// It can be constructed from any non-@c const reference to @ref rmi_handle,
  /// except @ref rmi_handle::light_reference.
  ///
  /// Once an @ref rmi_handle::reference is created, one can retrieve a
  /// reference to the object and do RMIs through it.
  ///
  /// The size of an @ref rmi_handle::reference is bigger than that of an
  /// @ref rmi_handle::light_reference.
  //////////////////////////////////////////////////////////////////////
  using reference = runtime::rmi_handle_extended_base;

  //////////////////////////////////////////////////////////////////////
  /// @brief Light reference to an object that was registered through
  ///        @ref rmi_handle.
  ///
  /// It can be constructed from any non-@c const reference to @ref rmi_handle.
  ///
  /// Once an @ref rmi_handle::light_reference is created, one can retrieve a
  /// reference to the object, but cannot do an RMI through it.
  ///
  /// The size of an @ref rmi_handle::light_reference is smaller than that of a
  /// @ref rmi_handle::reference.
  //////////////////////////////////////////////////////////////////////
  class light_reference
  : public runtime::rmi_handle_info
  {
  public:
    light_reference(void) = default;

    light_reference(rmi_handle& h)
    : runtime::rmi_handle_info(h)
    { }

    light_reference(reference const& h)
    : runtime::rmi_handle_info(h)
    { }

    void define_type(typer& t)
    { t.base<runtime::rmi_handle_info>(*this); }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Const light reference to an object that was registered through
  ///        @ref rmi_handle.
  ///
  /// It can be constructed from any reference to @ref rmi_handle.
  ///
  /// Once an @ref rmi_handle::const_light_reference is created, one can
  /// retrieve a @c const reference to the object, but cannot do an RMI through
  /// it.
  ///
  /// The size of an @ref rmi_handle::const_light_reference is smaller than that
  /// of a @ref rmi_handle::const_reference.
  //////////////////////////////////////////////////////////////////////
  class const_light_reference
  : public runtime::rmi_handle_info
  {
  public:
    const_light_reference(void) = default;

    template<typename Handle>
    const_light_reference(Handle&& h)
    : runtime::rmi_handle_info(std::forward<Handle>(h))
    { }

    void define_type(typer& t)
    { t.base<runtime::rmi_handle_info>(*this); }
  };

private:
  runtime::location_md* m_location;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref rmi_handle without registering any object.
  //////////////////////////////////////////////////////////////////////
  constexpr rmi_handle(void) noexcept
  : m_location(nullptr)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref rmi_handle which registers @p t in the
  ///        current gang.
  ///
  /// @param t     Object to be registered.
  /// @param flags Registration flags.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  explicit rmi_handle(T* const t,
                      const unsigned int flags = 0)
  : m_location(nullptr)
  {
    using namespace stapl::runtime;
    STAPL_RUNTIME_PROFILE( "rmi_handle::rmi_handle()",
                           (primitive_traits::non_blocking |
                            primitive_traits::sync) );
    register_object(this_context::get(), t, flags);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref rmi_handle which registers @p t in the gang
  ///        of the given execution context.
  ///
  /// @param ctx   Execution context the registration happens in.
  /// @param t     Object to be registered.
  /// @param flags Registration flags.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  rmi_handle(runtime::context& ctx,
             T* const t,
             const unsigned int flags = 0)
  : m_location(nullptr)
  { register_object(ctx, t, flags); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Unregisters the object if it is not yet unregistered.
  //////////////////////////////////////////////////////////////////////
  ~rmi_handle(void)
  { unregister_object(); }

  rmi_handle(rmi_handle const&) = delete;
  rmi_handle& operator=(rmi_handle const&) = delete;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Registers the object with this handle in the given execution
  ///        context.
  ///
  /// If the object exists in a gang with only one location and @ref try_rmi()
  /// support is not required, then it will not be inserted into a registry,
  /// avoiding any potential allocations.
  ///
  /// @param ctx   Execution context the registration happens in.
  /// @param t     Object to be registered.
  /// @param flags Registration flags.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void register_object(runtime::context& ctx,
                       T* const t,
                       const unsigned int flags = 0);

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata of the location this object registered in.
  //////////////////////////////////////////////////////////////////////
  runtime::location_md const& get_location_md(void) const noexcept
  { return *m_location; }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc get_location_md() const noexcept
  //////////////////////////////////////////////////////////////////////
  runtime::location_md& get_location_md(void) noexcept
  { return *m_location; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the location id this object registered in.
  //////////////////////////////////////////////////////////////////////
  size_type get_location_id(void) const noexcept;

  //////////////////////////////////////////////////////////////////////
  /// @brief Unregisters the object.
  ///
  /// If @c valid()==true before calling this, then @c valid()==false after
  /// calling it. Otherwise, behavior is undefined.
  //////////////////////////////////////////////////////////////////////
  void unregister_object(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Advances the epoch of the object.
  ///
  /// Advancing the epoch will flush any pending RMIs. It will also increase the
  /// epoch of the current gang if the object is not a named object.
  //////////////////////////////////////////////////////////////////////
  void advance_epoch(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Disable processing of incoming RMIs on this location so that local
  /// computation is guaranteed to run atomically with respect to incoming
  /// requests.
  //////////////////////////////////////////////////////////////////////
  void lock(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Attempt to disable processing of incoming RMIs on this location
  /// so that local computation is guaranteed to run atomically with respect
  /// to incoming requests.
  ///
  /// @return @c true if the lock was able to be acquired. @c false if it
  /// is already locked.
  //////////////////////////////////////////////////////////////////////
  bool try_lock(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Reenable processing of incoming RMIs on this location.
  //////////////////////////////////////////////////////////////////////
  void unlock(void);
};


namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Returns an @ref rmi_handle reference type that preserves the
///        qualifiers of the given member function pointer.
///
/// The handle passed to any primitive has to be convertible to the handle
/// reference type returned.
///
/// @see rmi_handle
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Signature>
struct appropriate_handle;

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref appropriate_handle for non-qualified member
///        functions.
///
/// @see rmi_handle
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R, typename C, typename... T>
struct appropriate_handle<R(C::*)(T...)>
{
  using type = rmi_handle::reference;
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref appropriate_handle for @c const member
///        functions.
///
/// @see rmi_handle
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R, typename C, typename... T>
struct appropriate_handle<R(C::*)(T...) const>
{
  using type = rmi_handle::const_reference;
};

} // namespace runtime

} // namespace stapl


namespace std {

//////////////////////////////////////////////////////////////////////
/// @brief Hasher for @ref stapl::runtime::object_virtual_address.
///
/// @related stapl::runtime::object_virtual_address
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
template<>
struct hash<stapl::runtime::object_virtual_address>
{
  using argument_type = stapl::runtime::object_virtual_address;
  using result_type   = std::size_t;

  std::size_t
  operator()(stapl::runtime::object_virtual_address const& t) const noexcept
  { return t.hash_code(); }
};

} // namespace std

#endif
