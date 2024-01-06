/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_DIRECTORY_HPP
#define STAPL_UTILITY_DIRECTORY_HPP

#include <stapl/runtime.hpp>
#include <stapl/runtime/utility/pool_allocator.hpp>
#include <stapl/utility/loc_qual.hpp>
#include <stapl/utility/down_cast.hpp>
#include <stapl/utility/directory_request.hpp>
#include <stapl/utility/hash.hpp>
#include <stapl/utility/use_default.hpp>
#include <stapl/containers/distribution/is_distribution_view.hpp>

#include <boost/function.hpp>
#include <boost/unordered_map.hpp>
//#include <boost/mpl/has_xxx.hpp>

#include <queue>

namespace stapl {

namespace detail {

//BOOST_MPL_HAS_XXX_TRAIT_DEF(partition_type)

//////////////////////////////////////////////////////////////////////
/// @brief Define default key to location mapping strategy used in the
///   directory class used by containers and the PARAGRAPH.
/// @ingroup directory
/// @tparam Key Key type which is mapped to a location
//////////////////////////////////////////////////////////////////////
template<typename Key>
class default_key_mapper
{
private:
  stapl::hash<Key>    m_key_hasher;
  location_type       m_num_locations;
  std::size_t         m_block_size;

public:
  default_key_mapper(void)
    : m_num_locations(get_num_locations()), m_block_size(1)
  { }

  explicit
  default_key_mapper(std::size_t domain_size)
    : m_num_locations(get_num_locations()),
      m_block_size(domain_size < m_num_locations ?
        1 : domain_size/m_num_locations)
  { }

  void set_num_locations(size_t nlocs)
  {
    m_num_locations = nlocs;
  }

  void define_type(typer& t)
  {
    t.member(m_key_hasher);
    t.member(m_num_locations);
    t.member(m_block_size);
  }

  constexpr bool is_perfect_mapper(void) const
  { return false; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Map key value to a location in this space
  /// @param key identifier to map
  /// @return location identifier
  //////////////////////////////////////////////////////////////////////
  std::pair<location_type, loc_qual> operator()(Key const& key) const
  {
    return std::make_pair(m_key_hasher(key) / m_block_size % m_num_locations,
                          LQ_CERTAIN);
  }
}; // class default_key_mapper


//////////////////////////////////////////////////////////////////////
/// @brief Trivial class used to unconditionally signal certainty about
///   presence of key on a location when migration is disabled.
/// @ingroup directory
/// @tparam Key Key type of derived directory class.
//////////////////////////////////////////////////////////////////////
template<typename Key>
class non_migratable_base
{
protected:
  constexpr bool requires_retry(Key const&) const
  {
    return false;
  }
}; // class non_migratable_base


//////////////////////////////////////////////////////////////////////
/// @brief Class used by @ref detail::directory_base to verify a key is
///  still registered on a location when migration is enabled (it's
///  possible key has moved while this request was in flight).
/// @ingroup directory
/// @tparam Key Key type of derived directory class.
//////////////////////////////////////////////////////////////////////
template<typename Key>
class migratable_base
{
private:
  using contains_func_t = boost::function<bool (Key)>;

  /// @brief Function to test if key exists at the current location.  Used
  /// to detect messages that arrive at location while migration is in
  /// progress and bounce messages back to manager location for forwarding
  /// to new location.
  contains_func_t                                    m_contains;

  /// @brief Migration can be disabled at runtime.
  bool                                               m_b_migration_enabled;

protected:
  template<typename Functor>
  migratable_base(Functor&& f, bool b_migration_enabled)
    : m_contains(std::forward<Functor>(f)),
      m_b_migration_enabled(b_migration_enabled)
  { }

  bool requires_retry(Key const& key) const
  {
    return m_b_migration_enabled && !m_contains(key);
  }
}; // class migratable_base

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Transmitter for @ref directory that sends requests using
///   @ref async_rmi().
/// @ingroup directory
//////////////////////////////////////////////////////////////////////
struct async_transmitter
{
  static unsigned int registration_flags(void)
  {
    return 0;
  }

  template<typename PMF, typename... Args>
  static void transmit(const location_type destination,
                       rmi_handle::reference const& h,
                       PMF const& pmf, Args&&... args)
  {
    async_rmi(destination, h, pmf, std::forward<Args>(args)...);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Transmitter for @ref directory that sends requests using
///   @ref unordered::async_rmi().
/// @ingroup directory
//////////////////////////////////////////////////////////////////////
struct unordered_async_transmitter
{
  static unsigned int registration_flags(void)
  {
    return 0;
  }

  template<typename PMF, typename... Args>
  static void transmit(const location_type destination,
                       rmi_handle::reference const& h,
                       PMF const& pmf, Args&&... args)
  {
    unordered::async_rmi(destination, h, pmf, std::forward<Args>(args)...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Transmitter for @ref directory that sends requests with
///   @ref try_rmi().
/// @ingroup directory
//////////////////////////////////////////////////////////////////////
struct try_transmitter
{
  static unsigned int registration_flags(void)
  {
    return stapl::get_num_locations() == 1 ? 0 : allow_try_rmi;
  }

  template<typename PMF, typename... Args>
  static void transmit(const location_type destination,
                       rmi_handle::reference const& h,
                       PMF const& pmf, Args&&... args)
  {
    try_rmi(destination, h, pmf, std::forward<Args>(args)...);
  }
};


namespace detail {
#if 0
template <typename Mapper, typename Key,
          bool = is_view_based<typename Mapper::partition_type>::type::value>
struct get_key_manager_impl
{
  static std::pair<location_type, loc_qual>
  apply(Mapper& mapper, Key const& key)
  {
    return std::make_pair(mapper(key), LQ_CERTAIN);
  }
};

template <typename Mapper, typename Key>
struct get_key_manager_impl<Mapper, Key, true>
{
  static std::pair<location_type, loc_qual>
  apply(Mapper& mapper, Key const& key)
  {
    return mapper(key);
  }
};

template <typename Mapper, typename Key, bool>
struct get_key_manager
{
  std::pair<location_type, loc_qual>
  operator()(Mapper& mapper, Key const& key);
};

template <typename Mapper, typename Key>
struct get_key_manager<Mapper, Key, false>
{
  std::pair<location_type, loc_qual>
  operator()(Mapper& mapper, Key const& key)
  {
    return std::make_pair(mapper(key), LQ_CERTAIN);
  }
};

template <typename Mapper, typename Key>
struct get_key_manager<Mapper, Key, true>
{
  std::pair<location_type, loc_qual>
  operator()(Mapper& mapper, Key const& key)
  {
    return get_key_manager_impl<Mapper, Key>::apply(mapper, key);
  }
};
#endif


//////////////////////////////////////////////////////////////////////
/// @brief Basic request queue used by the @ref directory to buffer
/// pending messages to an unregistered key.  Includes functionality
/// to guard against re-entrant, duplicated traversals of the structure.
//////////////////////////////////////////////////////////////////////
template<typename Request>
class request_queue
  : public std::queue<Request>
{
private:
  bool m_b_flush_in_progress;

public:
  request_queue(void)
   : m_b_flush_in_progress(false)
  { }

  bool flush_in_progress(void) const
  { return m_b_flush_in_progress; }

  void start_flush(void)
  {
    stapl_assert(!m_b_flush_in_progress, "queue traversal already in process");
    m_b_flush_in_progress = true;
  }

  void stop_flush(void)
  {
    stapl_assert(m_b_flush_in_progress, "no queue traversal in process");
    m_b_flush_in_progress = false;
  }
}; // class request_queue


//////////////////////////////////////////////////////////////////////
/// @brief Base class of @ref directory that provides the bulk of the
///   non migration specific implementation.
/// @tparam Key Key types tracked by this directory.
/// @tparam Transmitter Type Encapsulting underlying ARMI primitives for
///   communication.
/// @tparam Mapper Functor type that maps keys to location that manages them.
/// @tparam Registry Holds key to location mapping.
/// @ingroup directory
///
/// @todo Migration should probably be a static / type parameter instead of
/// a runtime trait, as it adds some members (e.g., m_contains) and cycles.
///
/// @todo use intrusive equivalents of std::queue and boost::unordered_map
/// to implement m_pending to reduce heap allocs.  Need a little massaging
/// in container usage of class.
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Transmitter, typename Mapper, typename Registry,
         bool EnableMigration>
class directory_base
  : public p_object,
    private std::conditional<
      EnableMigration,
      detail::migratable_base<Key>,
      detail::non_migratable_base<Key>
    >::type
{
private:
  using base_t =
    typename std::conditional<
      EnableMigration,
      detail::migratable_base<Key>,
      detail::non_migratable_base<Key>
    >::type;

protected:
  template<typename Directory, typename Functor, typename IntrusiveHook>
  friend class detail::directory_request;

  using mapper_t   =
    typename select_parameter<Mapper, detail::default_key_mapper<Key>>::type;

  using registry_t =
    typename select_parameter<
      Registry,
      boost::unordered_map<
        Key, location_type, stapl::hash<Key>,
        std::equal_to<Key>
#ifndef STAPL_RUNTIME_MANAGED_ALLOC_DISABLE
        , pool_allocator<std::pair<Key const, location_type>>
#endif
      >
    >::type;

  using transmitter_type =
    typename select_parameter<Transmitter, async_transmitter>::type;

public:
  using manager_type  = mapper_t;
  using registry_type = registry_t;
  using key_type      = Key;

private:
  using request_t = detail::directory_request_base<Key>*;

protected:
  using queue_t   = request_queue<request_t>;

  using queues_t  =
    boost::unordered_map<
      Key, queue_t, stapl::hash<Key>,
      std::equal_to<Key>
#ifndef STAPL_RUNTIME_MANAGED_ALLOC_DISABLE
      , pool_allocator<std::pair<Key const, queue_t>>
#endif
    >;

  /// Stores key to locations mapping.
  registry_t                                                 m_registry;

  /// @brief Map of queues, indexed by @p Key.  Entry for keys exists when
  /// messages arrive and must be buffered prior to registration, during
  /// migration, etc.
  queues_t                                                   m_pending;

  /// @brief Map a key to location that manages it
  /// (i.e., where it has entry in m_registry).
  mapper_t                                                   m_key_mapper;

  bool                                                       m_b_perfect_mapper;

public:
  bool has_perfect_mapper(void) const
  {
    return m_b_perfect_mapper;
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Internal target of RMI used by @ref register_key(). Invoked on
  /// location managing @p key.  Insert into registry and flush any
  /// pending message to location @p loc.
  //////////////////////////////////////////////////////////////////////
  void register_key_impl(Key const& key,
                         const location_type loc)
  {
    stapl_assert(!m_b_perfect_mapper || loc == this->get_location_id(),
                 "Perfect mapping failed");

    stapl_assert(m_registry.find(key) == m_registry.end(),
                 "tried to register a key twice");

    m_registry.insert(std::make_pair(key, loc));

    auto it = m_pending.find(key);

    if (it != m_pending.end())
      flush_pending(loc, key, it->second);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Internal target of RMI used by @ref register_apply().
  ///
  /// Invoked on location managing @p key.  Insert into registry and flush any
  /// pending message to location @p loc. Functor @p f is applied after
  /// registration.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void register_apply_impl(Key const& key,
                           const location_type loc,
                           Functor&& f)
  {
    stapl_assert(!m_b_perfect_mapper || loc == this->get_location_id(),
                 "Perfect mapping failed");

    stapl_assert(m_registry.find(key) == m_registry.end(),
                 "tried to register a key twice");

    m_registry.insert(std::make_pair(key, loc));
    f(*this, key);

    auto it = m_pending.find(key);

    if (it != m_pending.end())
      flush_pending(loc, key, it->second);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Internal target of RMI used by @ref unregister_apply.  Invoked on
  /// location managing @p key.  Remove from registry. Functor @p f is applied
  /// after unregistration.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void unregister_apply_impl(const Key& key,
                             Functor&& f)
  {
    stapl_assert(m_registry.find(key) != m_registry.end(),
                 "tried to unregister non-registered key");

    m_registry.erase(key);

    f(key);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Method used as the target of RMIs which forward @ref invoke_where()
  /// functor requests to the location where a key has been registered.
  ///
  /// @param key Key value that was migrated.
  /// @param f   Nullary functor to apply.
  ///
  /// @sa directory_request::send_message_rmi()
  ///
  /// @todo I believe some containers handle redirection internally
  /// (i.e., unconditionally return true from m_contains invocations.  This
  /// semantic should be made symmetric with the PARAGRAPH.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void execute(Key const& key,
               Functor&& f)
  {
    // This was the manager's estimate of where the Key is. The key could,
    // however, have been migrated. In that case bounce this request back
    // to the manager.
    //
    // NOTE - this is an error if this location is itself the manager, or if
    // we are nested on one location.
    if (this->requires_retry(key))
    {
      const std::pair<location_type, loc_qual> manager_loc =
          m_key_mapper(key);

      stapl_assert(manager_loc.second != LQ_LOOKUP,
                   "directory::execute given location to forward to.");

      stapl_assert(this->get_num_locations() != 1
                     && manager_loc.first != this->get_location_id(),
                   "bouncing key back to itself!");

      // bounce the request back to the manager
      using mem_fun_t = void (directory_base::*)(Key const&, decltype(f));

      constexpr mem_fun_t mem_fun = &directory_base::request_forward;

      transmitter_type::transmit(
        manager_loc.first, this->get_rmi_handle(), mem_fun,
        key, std::forward<Functor>(f));

      return;
    }

    f(*this, key);
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Internal method, target of RMI called via migrate_impl to invoke
  /// functor @p f at new registration location, post registry update.
  ///
  /// @param f Nullary functor to apply.
  ///
  /// @sa migrate_impl()
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void execute(Functor&& f)
  {
    f();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Forwards functor and arguments to location managing @p key.
  /// The location managing the key will invoke the method with the supplied
  /// arguments.
  /// @param key Key value whose managing location (i.e., where registry
  ///        entry is stored) is where @p pmf will be invoked.
  /// @param pmf Directory member function to be invoked.
  /// @param args variadic list of arguments to be applied.
  ///
  /// This is used to internally encapsulate to forward various method
  /// invocations from the requester (e.g., registration, invoke_where, etc)
  /// to the location where they can be serviced, forwarded, etc.
  ///
  /// @ingroup directory
  /// @sa directory_base::invoke_where
  //////////////////////////////////////////////////////////////////////
  template<typename Directory, typename... PMFArgs, typename... Args>
  void invoke_at_manager(Key const& key,
                         void (Directory::* const pmf)(PMFArgs...),
                         Args&&... args)
  {
    this->invoke_at_manager_impl<transmitter_type>(
      key, pmf, std::forward<Args>(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of @see invoke_at_manager with a given transmitter
  ///
  /// @tparam Trans The transmitter type
  //////////////////////////////////////////////////////////////////////
  template <typename Trans,
            typename Directory,
            typename... PMFArgs,
            typename... Args>
  void invoke_at_manager_impl(Key const& key,
                              void (Directory::*const pmf)(PMFArgs...),
                              Args&&... args)
  {
    const std::pair<location_type, loc_qual> location = m_key_mapper(key);

    if (location.first == this->get_location_id()
        && location.second == LQ_CERTAIN)
    {
      (down_cast<Directory*>(this)->*pmf)(std::forward<Args>(args)...);
      return;
    }

    // else
    if (location.second == LQ_CERTAIN)
    {
      Trans::transmit(
        location.first, this->get_rmi_handle(), pmf,
        std::forward<Args>(args)...);

      return;
    }

    // else
    stapl_assert(location.first != this->get_location_id(),
      "bouncing key back to itself");

    using mem_fun_t =
      void (directory_base::*)(Key const&, decltype(pmf), decltype(args)...);

    constexpr mem_fun_t mem_fun = &directory_base::invoke_at_manager;

    Trans::transmit(
      location.first, this->get_rmi_handle(), mem_fun, key, pmf,
      std::forward<Args>(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Forwards const functor and arguments to location managing @p key.
  /// The location managing the key will invoke the method with the supplied
  /// arguments.
  /// @param key Key value whose managing location (i.e., where registry
  ///        entry is stored) is where @p pmf will be invoked.
  /// @param pmf Directory const member function to be invoked.
  /// @param args variadic list of arguments to be applied.
  ///
  /// This is used to internally encapsulate to forward various method
  /// invocations from the requester (e.g., registration, invoke_where, etc)
  /// to the location where they can be serviced, forwarded, etc.
  ///
  /// @ingroup directory
  /// @sa directory_base::invoke_where
  //////////////////////////////////////////////////////////////////////
  template<typename Directory, typename... PMFArgs, typename... Args>
  void invoke_at_manager(Key const& key,
                         void (Directory::* const pmf)(PMFArgs...) const,
                         Args&&... args) const
  {
    this->invoke_at_manager_impl<transmitter_type>(
      key, pmf, std::forward<Args>(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of @see invoke_at_manager with a given transmitter
  ///
  /// @tparam Trans The given transmitter type
  //////////////////////////////////////////////////////////////////////
  template <typename Trans,
            typename Directory,
            typename... PMFArgs,
            typename... Args>
  void invoke_at_manager_impl(Key const& key,
                              void (Directory::*const pmf)(PMFArgs...) const,
                              Args&&... args) const
  {
    const std::pair<location_type, loc_qual> location = m_key_mapper(key);

    if (location.first == this->get_location_id()
        && location.second == LQ_CERTAIN)
    {
      (down_cast<Directory const*>(this)->*pmf)(std::forward<Args>(args)...);
      return;
    }

    //else
    if (location.second == LQ_CERTAIN)
    {
      Trans::transmit(
        location.first, this->get_rmi_handle(), pmf,
        std::forward<Args>(args)...);
      return;
    }

    // else
    stapl_assert(location.first != this->get_location_id(),
     "bouncing key back to itself");

    using mem_fun_t =
      void (directory_base::*)
        (Key const&, decltype(pmf), decltype(args)...) const;

    constexpr mem_fun_t mem_fun = &directory_base::invoke_at_manager;

    Trans::transmit(
      location.first, this->get_rmi_handle(), mem_fun, key, pmf,
      std::forward<Args>(args)...);
  }

public:
  mapper_t const& key_mapper(void) const
  {
    return m_key_mapper;
  }

  mapper_t& key_mapper(void)
  {
    return m_key_mapper;
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Forward all buffered requests for @p key to location @p
  /// @p loc.
  ///
  /// @param loc     Location to send buffered requests to.
  /// @param key     Key value buffered requests are for.
  /// @param pending Pending queue associated with @p Key.
  ///
  /// This is internal method only called on the directory location managing
  /// @p key.
  //////////////////////////////////////////////////////////////////////
  void flush_pending(const location_type loc,
                     Key const& key,
                     queue_t& pending)
  {
    // Avoid re-entrant flush of pending queue for this key.
    if (pending.flush_in_progress())
      return;

    pending.start_flush();

    // if key is on this location, invoke directly and avoid RMI.
    if (loc == this->get_location_id())
    {
      while (!pending.empty())
      {
        request_t req = pending.front();
        pending.pop();

        req->send_message_local(*this, key);
        delete req;
      }
    }
    else
    {
      stapl_assert(!m_b_perfect_mapper, "Perfect mapping failed");

      // deliver message via RMI
      while (!pending.empty())
      {
        request_t req = pending.front();
        pending.pop();

        req->send_message_rmi(key, loc, this->get_rmi_handle());
        delete req;
      }
    }

    pending.stop_flush();

    m_pending.erase(key);
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Internal method used to facilitate invocation of @p f on location
  /// where @p key is currently registered.  Called via RMI on directory
  /// location where @p key is managed.
  ///
  /// @param key Key whose registered location determines where @p f should
  /// be executed.
  ///
  /// @param f Functor to invoke (unary, @p key passed as parameter).
  ///
  /// @todo avoid heap allocs on found key by searching, flushing, then
  /// forward this message.  Consider ordering...
  ///
  /// Called by invoke_where and execute (the latter when migration enabled).
  ///
  /// @bug Unguarded call to flush_pending can cause ordering violations on
  /// messages (i.e., RMI call inside a current flush_pending()) causes
  /// reentrant call.  Need to add a per key flushing_in_progress bit and guard
  /// flushing if this is true (just add, return, and let in progress flush
  /// handle it).  This isn't a problem for the PARAGRAPH but containers are
  /// supposed to guarantee ordering by default.
  ///
  /// @todo consider checking return existence of m_pending entry + flushing
  /// flag instead of find on m_registry.  In theory, shouldn't be an entry
  /// in both simultaneously.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor, typename Trans=transmitter_type>
  void request_forward(Key const& key,
                       Functor&& f)
  {
    // Clearly we must buffer the message if the key isn't registered. However,
    // if the key is registered AND the queue is non empty, we must still
    // buffer this request at the end of the queue if we wish to ensure
    // ordering.
    //
    // A flush may be in process for this key below on the call stack via
    // register_key().  That flush will service this request after previous
    // messages.
    //
    // We can avoid this heap allocated for the unordered case
    // and forward immediately.  Shouldn't drastically affect QOS /
    // ordering as this is just a corner case (but pedantic, heavy request
    // cases could.
    //
    auto rit                  = m_registry.find(key);
    const bool key_registered = (rit != m_registry.end());
    auto it                   = m_pending.find(key);

    if (it == m_pending.end())
    {
      if (key_registered)
      {
        const location_type loc = rit->second;

        if (loc == this->get_location_id())
        {
          f(*this, key);
        }
        else
        {
          stapl_assert(this->get_num_locations() != 1,
            "invalid target location");

          stapl_assert(!m_b_perfect_mapper, "Perfect mapping failed");

          using mem_fun_t =
            void (directory_base::*)(key_type const&, decltype(f));

          const mem_fun_t mem_fun = &directory_base::execute;

          Trans::transmit(loc, this->get_rmi_handle(),
                                     mem_fun, key, std::forward<Functor>(f));
        }
        return;
      }

      it = m_pending.insert(std::make_pair(key, queue_t())).first;
    }
    else
    {
     // If the key is registered and there is a pending queue, we'd expect
     // that register_key() is in the process of flushing the queue and
     // we are above this call on the stack.
     stapl_assert(
       !key_registered || it->second.flush_in_progress(),
       "expected pending queue flush to be in progress for this key");
    }

    using derived_request_t =
      detail::directory_request<
        directory_base, typename std::decay<Functor>::type>;

    request_t request = new derived_request_t{std::forward<Functor>(f)};

    queue_t& pending = it->second;
    pending.push(request);

    if (!key_registered)
      return;

    flush_pending(rit->second, key, pending);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Internal method used to facilitate invocation of @p f on location
  /// where @p key is currently registered. If @p key is not registered, the
  /// request is dropped; otherwise, it is called via RMI on directory
  /// location where @p key is managed. This is directed at situations where
  /// it is legal for a key to not exist, i.e. erasing non-existent keys in
  /// the unordered set container.
  ///
  /// @param key Key whose registered location determines where @p f should
  /// be executed.
  ///
  /// @param f Functor to invoke (unary, @p key passed as parameter).
  ///
  /// @todo avoid heap allocs on found key by searching, flushing, then
  /// forward this message.  Consider ordering...
  ///
  /// Called by invoke_where and execute (the latter when migration enabled).
  ///
  /// @bug Unguarded call to flush_pending can cause ordering violations on
  /// messages (i.e., RMI call inside a current flush_pending()) causes
  /// reentrant call.  Need to add a per key flushing_in_progress bit and guard
  /// flushing if this is true (just add, return, and let in progress flush
  /// handle it).  This isn't a problem for the PARAGRAPH but containers are
  /// supposed to guarantee ordering by default.
  ///
  /// @todo consider checking return existence of m_pending entry + flushing
  /// flag instead of find on m_registry.  In theory, shouldn't be an entry
  /// in both simultaneously.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void try_request_forward(Key const& key,
                           Functor&& f)
  {
    const auto rit            = m_registry.find(key);
    const bool key_registered = (rit != m_registry.end());

    if (!key_registered)
      return;

    auto it = m_pending.find(key);

    if (it == m_pending.end())
    {
      const location_type loc = rit->second;

      if (loc == this->get_location_id())
        f(*this, key);
      else
      {
        stapl_assert(!m_b_perfect_mapper, "Perfect mapping failed");

        using mem_fun_t =
          void (directory_base::*)(key_type const&, decltype(f));

        constexpr mem_fun_t mem_fun = &directory_base::execute;

        transmitter_type::transmit(loc, this->get_rmi_handle(),
                                   mem_fun, key, std::forward<Functor>(f));
      }
      return;
    }

    using derived_request_t =
      detail::directory_request<
        directory_base, typename std::decay<Functor>::type>;

    request_t request = new derived_request_t{std::forward<Functor>(f)};

    queue_t& pending = it->second;
    pending.push(request);

    flush_pending(rit->second, key, pending);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Registration flags for @ref directory.
  ///
  /// @param enable_buffering Specifies RMIs should be aggregated.
  //////////////////////////////////////////////////////////////////////
  static unsigned int registration_flags(const bool enable_buffering)
  {
    return (enable_buffering
              ? transmitter_type::registration_flags()
              : (transmitter_type::registration_flags() | no_aggregation));
  }

protected:
  directory_base(bool b_enable_buffering)
    : p_object(registration_flags(b_enable_buffering)),
      m_b_perfect_mapper(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used by PARAGRAPH
  /// @param f Functor that, given a key, returns whether the diretory
  ///          contains the key.
  /// @param mapper Maps keys to location that manages them.
  /// @param b_migration_enabled Specifies whether migration is enabled
  /// @param b_enable_buffering Specifies whether directory requests using ARMI
  ///        are buffered.
  /// @param b_perfect_mapper Indicates whether mapper map keys perfectly
  ///        to the location that will eventually register them.  If this is,
  ///        true additional assertions are enabled to check that this indeed
  ///        the case.
  //////////////////////////////////////////////////////////////////////
  template<typename Function>
  directory_base(Function&& f,
                 mapper_t mapper,
                 bool b_migration_enabled,
                 bool b_enable_buffering,
                 bool b_perfect_mapper)
    : p_object(registration_flags(b_enable_buffering)),
      base_t(std::forward<Function>(f), b_migration_enabled),
      m_key_mapper(std::move(mapper)),
      m_b_perfect_mapper(b_perfect_mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor specifying custom mapper and registry.  Used by
  ///        container framework.
  //////////////////////////////////////////////////////////////////////
  directory_base(mapper_t mapper, registry_t registry)
    : p_object(registration_flags(true)),
      m_registry(std::move(registry)),
      m_key_mapper(std::move(mapper)),
      m_b_perfect_mapper(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @todo Directory should probably just be moveable, not copy constructible.
  /// It's difficult to take an accurate snapshot with requests in flight.
  /// At the very least, assert on appropriate conditions
  /// (e.g., m_pending is empty).
  //////////////////////////////////////////////////////////////////////
  directory_base(directory_base const& other)
    : p_object(registration_flags(true)),
      m_registry(other.m_registry),
      m_key_mapper(other.m_key_mapper),
      m_b_perfect_mapper(other.m_b_perfect_mapper)
  {
    // Only PARAGRAPH assumes this...
    //
    // stapl_assert(other.empty(),
    //              "directory::directory(directory const&) other not empty");

    stapl_assert(other.get_rmi_handle().get_flags() == 0,
                 "directory_base copy constructor found non_buffer flags");

    stapl_assert(compare_gangs(this->get_rmi_handle(),
                               other.get_rmi_handle()) >= 0,
                 "gangs not conformable");

    stapl_assert(other.m_pending.empty(),
                 "requests resident in the directory");
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Directory should probably just be moveable, not assignable.
  //////////////////////////////////////////////////////////////////////
  directory_base& operator=(directory_base const& other)
  {
    stapl_assert(m_pending.empty() && other.m_pending.empty(),
                 "requests resident in the directory");

    if (this != &other) {
      base_t::operator=(other);
      m_registry         = other.m_registry;
      m_key_mapper       = other.m_key_mapper;
      m_b_perfect_mapper = other.m_b_perfect_mapper;
    }
    return *this;
  }

  ~directory_base(void)
  {
    stapl_assert(m_pending.empty(), "requests resident in the directory");
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if there are no entries in directory's registry on
  /// this location.
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return m_registry.empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear all local registrations and pending messages in directory.
  /// Used by @p clear() methods of container distributions.
  //////////////////////////////////////////////////////////////////////
  void reset(void)
  {
    stapl_assert(m_pending.empty(), "requests resident in the directory");
    m_registry.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Attempt registration from the managing location.  Return true
  /// if successful (i.e., not already registered).
  //////////////////////////////////////////////////////////////////////
  bool try_register_key_local(Key const& key)
  {
    const location_type loc = this->get_location_id();

    stapl_assert(m_key_mapper(key).first == this->get_location_id(),
      "try_register called on key not mapped locally");

    if (m_registry.find(key) != m_registry.end())
      return false;

    m_registry.insert(std::make_pair(key, loc));

    typename queues_t::iterator it = m_pending.find(key);

    if (it != m_pending.end())
      flush_pending(loc, key, it->second);

    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Associate @p key in directory's registry with the location
  /// invoking this method.
  ///
  /// @todo Make implementation symmetric with unregister_key().
  //////////////////////////////////////////////////////////////////////
  void register_key(Key const& key)
  {
    const location_type src_loc = this->get_location_id();

    invoke_at_manager(key, &directory_base::register_key_impl, key, src_loc);
  }

  void register_keys(std::pair<Key, Key> const& keys)
  {
    const location_type src_loc = this->get_location_id();

    for (Key key = keys.first; key <= keys.second; ++key)
      invoke_at_manager(key, &directory_base::register_key_impl, key, src_loc);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Associate @p key in directory's registry with location
  ///   invoking this method. Registration location is implicitly set
  ///   to be caller of this method.
  /// @param key Key to register.
  /// @param f Unary function object (receives @p key) as parameter to
  ///   call after registration.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void register_apply(Key const& key, Functor&& f)
  {
    using mem_fun_t =
      void (directory_base::*)(Key const&, const location_type, decltype(f));

    constexpr mem_fun_t mem_fun = &directory_base::register_apply_impl;
    const location_type src_loc = this->get_location_id();

    invoke_at_manager(key, mem_fun, key, src_loc, std::forward<Functor>(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Associate @p key in directory's registry with location invoking
  /// this method.
  ///
  /// @param key Key to registered.
  /// @param f Unary function object (receives @p key) as parameter to
  ///   call after registration.
  /// @param location Location this key should be mapped to in the registry.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void register_apply(Key const& key, Functor&& f, location_type location)
  {
    using mem_fun_t =
      void (directory_base::*)(Key const&, const location_type, decltype(f));

    constexpr mem_fun_t mem_fun = &directory_base::register_apply_impl;

    invoke_at_manager(key, mem_fun, key, location, std::forward<Functor>(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Unregister @p key in directory's registry.
  //////////////////////////////////////////////////////////////////////
  void unregister_key(Key const& key)
  {
    unregister_apply(key, identity<Key>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Unregister @p key and apply a function object @p f at location
  /// where directory's registry manages @p key.
  //
  /// @param key Key to unregister.
  ///
  /// @param f Unary function object (receives @p key) as parameter to call
  /// after unregistration.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void unregister_apply(Key const& key,
                        Functor&& f)
  {
    using mem_fun_t = void (directory_base::*)(Key const&, decltype(f));

    constexpr mem_fun_t mem_fun = &directory_base::unregister_apply_impl;

    invoke_at_manager(key, mem_fun, key, std::forward<Functor>(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke function object at the location where key is currently
  /// registered.
  ///
  /// @param f Functor to apply. Unary operator, receives key as parameter.
  /// @param key Registered key
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void invoke_where(Functor&& f, Key const& key)
  {
    using mem_fun_t = void (directory_base::*)(Key const&, decltype(f));

    constexpr mem_fun_t mem_fun = &directory_base::request_forward;

    invoke_at_manager(key, mem_fun, key, std::forward<Functor>(f));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke function object at the location where key is currently
  /// registered if the key is registered.
  ///
  /// @param f Functor to apply.  Unary operator, receives key as parameter.
  /// @param key Key that may not be registered when method is invoked
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void try_invoke_where(Functor&& f, Key const& key)
  {
    using mem_fun_t = void (directory_base::*)(Key const&, decltype(f));

    constexpr mem_fun_t mem_fun = &directory_base::try_request_forward;
    invoke_at_manager(key, mem_fun, key, std::forward<Functor>(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke function object at the location where key is currently
  /// registered, disregarding RMI causal ordering.
  ///
  /// @param f Functor to apply. Unary operator, receives key as parameter.
  /// @param key Registered key
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void unordered_invoke_where(Functor&& f, Key const& key)
  {
    using mem_fun_t = void (directory_base::*)(Key const&, decltype(f));

    constexpr mem_fun_t mem_fun
      = &directory_base::template request_forward<decltype(f),
                                                  unordered_async_transmitter>;

    invoke_at_manager_impl<unordered_async_transmitter>(
      key, mem_fun, key, std::forward<Functor>(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Provide locality information about a key managed by the directory.
  /// @param key registered key
  ///
  /// @note Used by containers that inherit / customize directory behavior.
  /// @todo The block_until is used for the transient case where the locality
  ///       information is supposed to be on this location, but is not
  ///       yet registered.
  //////////////////////////////////////////////////////////////////////
  locality_info locality(Key const& key) // const
  {
    typename registry_t::const_iterator reg_it = m_registry.find(key);

    if (reg_it != m_registry.end())
    {
      return locality_info(LQ_CERTAIN, invalid_affinity_tag,
                           this->get_rmi_handle(), reg_it->second);
    }

    // else
    const std::pair<location_type, loc_qual> loc = m_key_mapper(key);

    // don't allow the method to return if it's supposed to
    // be here but it's not registered yet
    if (this->get_location_id() == loc.first)
      block_until([&]() {
        return m_registry.find(key) != m_registry.end();
      });

    return locality_info(LQ_LOOKUP, invalid_affinity_tag,
                         this->get_rmi_handle(), loc.first);
  }
}; // class directory_base

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Provides basic functionality to forward computation (via a generic
///   functor) to a location determined by the distributed registration of key
///   values.  The class handles heterogeneous work requests and supports
///   relaxed consistency, placing no synchronization requirements on the
///   registration of key and functor forwarding requests employing that key.
/// @tparam Key Key types tracked by this directory.
/// @tparam Transmitter Type Encapsulting underlying ARMI primitives for
///   communication.
/// @tparam Mapper Functor type that maps keys to location that manages them.
/// @tparam Registry Holds key to location mapping.
/// @ingroup directory
///
/// Primary template used for directories without internal migration support
/// (i.e., containers where it is handled externally for now).
//////////////////////////////////////////////////////////////////////
template<typename Key,
         typename Transmitter = use_default,
         typename Mapper      = use_default,
         typename Registry    = use_default,
         bool EnableMigration = false>
class directory
  : public detail::directory_base<
      Key, Transmitter, Mapper, Registry, EnableMigration>
{
private:
  using base_t =
    detail::directory_base<Key, Transmitter, Mapper, Registry, EnableMigration>;

public:
  directory(bool b_enable_buffering)
    : base_t(b_enable_buffering)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor specifying custom mapper and registry.  Used by
  ///   container framework.
  /// @param mapper Maps keys to location that manages them.
  /// @param registry Holds key to location mapping.
  //////////////////////////////////////////////////////////////////////
  directory(typename base_t::mapper_t mapper,
            typename base_t::registry_t registry =
              typename base_t::registry_t())
    : base_t(std::move(mapper), std::move(registry))
  { }
}; // class directory (without migration)


//////////////////////////////////////////////////////////////////////
/// @brief Specialization with migration support.
/// @ingroup directory
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Transmitter, typename Mapper, typename Registry>
class directory<Key, Transmitter, Mapper, Registry, true>
  : public detail::directory_base<Key, Transmitter, Mapper, Registry, true>
{
private:
  using base_t =
    detail::directory_base<Key, Transmitter, Mapper, Registry, true>;

public:
  using base_directory_type = base_t;

  directory(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used by PARAGRAPH
  ///
  /// @param f Unary functor passed key and return true iff current location
  ///        has key registered here (in this context, a wrapper around an
  ///        edge_container method).
  ///
  /// @param b_migration_enabled Specifies whether object allows key migration.
  /// @param b_enable_buffering Specifies whether directory requests using ARMI
  ///        are buffered.
  //////////////////////////////////////////////////////////////////////
  template<typename Function, typename MapperParam>
  directory(Function&& f,
            MapperParam&& mapper_param,
            bool b_migration_enabled,
            bool b_enable_buffering)
    : base_t(std::forward<Function>(f),
             std::forward<MapperParam>(mapper_param),
             b_migration_enabled, b_enable_buffering,
             mapper_param.is_perfect_mapper())
  { }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Internal method, target of RMI from @ref migrate, invoked on
  /// location managing @p key.
  //
  /// @param key Key to migrate.
  /// @param loc Location to migrate @p key to.
  /// @param f   Unary functor (passed @p key) to apply post migration on
  ///            location @p loc.
  ///
  /// @warning Migration is performed using @ref stapl::async_rmi() for
  ///          communication. This means that the container has to be alive
  ///          until the migration finishes.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void migrate_impl(Key const& key,
                    const location_type loc,
                    Functor&& f)
  {
    stapl_assert(this->m_registry.find(key) != this->m_registry.end(),
                 "tried to migrate a nonexistent key");

    stapl_assert(this->m_registry[key] != loc,
                 "tried to migrate a key to its existing location");

    // update the entry in the registry
    this->m_registry.erase(key);
    this->m_registry.insert(std::make_pair(key, loc));

    // execute the functor on the correct location
    if (loc == this->get_location_id())
    {
      f();
      return;
    }

    using mem_fun_t = void (directory::*)(decltype(f));

    constexpr mem_fun_t mem_fun = &directory::execute;

    async_rmi(loc, this->get_rmi_handle(), mem_fun, std::forward<Functor>(f));

    auto it = this->m_pending.find(key);

    if (it != this->m_pending.end())
      this->flush_pending(loc, key, it->second);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Change location a key is registered / associated with.  After
  /// registry is updated, apply f is applied a new associated location.
  ///
  /// @param key Key whose registered location is to be updated.
  /// @param loc Location which @p key should now be associated with.
  /// @param f Nullary function object applied at @p loc after the registry
  ///        is updated.
  ///
  /// @bug I believe that this can only be called right now on the location
  /// where @p key is currently registered.  Verify this is case and add an
  /// appropriate assertion.
  ///
  /// @todo move to @ref detail::migratable_base.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void migrate(Key const& key,
               const location_type loc,
               Functor&& f)
  {
    using mem_fun_t =
      void (directory::*)(Key const&, const location_type, decltype(f));

    constexpr mem_fun_t mem_fun = &directory::migrate_impl;

    this->invoke_at_manager(key, mem_fun, key, loc, std::forward<Functor>(f));
  }
}; // class directory (with migration)

} // namespace stapl

#endif // STAPL_UTILITY_DIRECTORY_HPP
