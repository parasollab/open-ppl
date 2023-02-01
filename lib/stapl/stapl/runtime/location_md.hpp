/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_LOCATION_HPP
#define STAPL_RUNTIME_LOCATION_HPP

#include "config.hpp"
#include "context_id.hpp"
#include "gang_md.hpp"
#include "rmi_handle_fwd.hpp"
#include "runqueue.hpp"
#include "executor/executor_base.hpp"
#include "utility/logical_clock.hpp"
#include "utility/ref_counted.hpp"
#include "utility/spmd_registry.hpp"
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <boost/container/flat_set.hpp>
#include <boost/intrusive/set.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Location metadata.
///
/// The metadata includes fence metadata for the intragang requests of the
/// contexts of the associated location, objects registered through
/// @ref rmi_handle, @ref location_specific_storage objects and a @ref runqueue
/// for incoming requests.
///
/// All @ref location_md objects are reference-counted and are destroyed when
/// the count becomes @c 0. The associated @ref gang_md object is notified when
/// the @ref location_md object is destroyed.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class location_md
: public ref_counted<location_md>
{
public:
  using id            = location_id;
  using epoch_type    = logical_clock::time_type;
  using size_type     = gang_md::size_type;
  using runqueue_type = runqueue;

  //////////////////////////////////////////////////////////////////////
  /// @brief Fence metadata for intragang requests.
  //////////////////////////////////////////////////////////////////////
  class fence_md
  {
  private:
    size_type m_sent;
    size_type m_proc;

  public:
    constexpr fence_md(void) noexcept
    : m_sent(0),
      m_proc(0)
    { }

    void reset(void) noexcept
    {
      m_sent = 0;
      m_proc = 0;
    }

    void add_pending(const size_type sent = 1) noexcept
    { m_sent += sent; }

    void add(const size_type sent, const size_type proc) noexcept
    {
      m_sent += sent;
      m_proc += proc;
    }

    constexpr size_type get_sent(void) const noexcept
    { return m_sent; }

    constexpr int pending(void) const noexcept
    { return (m_sent - m_proc); }
  };

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Base class for location specific storage elements.
  //////////////////////////////////////////////////////////////////////
  class lss_element_base
  : public boost::intrusive::set_base_hook<
             boost::intrusive::constant_time_size<false>
           >
  {
  private:
    void* m_p;

  public:
    explicit lss_element_base(void* p) noexcept
    : m_p(p)
    { }

    virtual ~lss_element_base(void) = default;

    friend constexpr bool operator<(lss_element_base const& x,
                                    lss_element_base const& y) noexcept
    { return (x.m_p<y.m_p); }

    friend constexpr bool operator==(lss_element_base const& x,
                                     lss_element_base const& y) noexcept
    { return (x.m_p==y.m_p); }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Storage class for location specific storage elements.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  class lss_element final
  : public lss_element_base
  {
  private:
    T m_value;

  public:
    explicit lss_element(void* p)
    : lss_element_base(p),
      m_value()
    { }

    template<typename U>
    lss_element(void* p, U&& u)
    : lss_element_base(p),
      m_value(std::forward<U>(u))
    { }

    template<typename CreatorFunction>
    lss_element(void* p, CreatorFunction&& f, int)
    : lss_element_base(p),
      m_value(f())
    { }

    template<typename U>
    T& reset(U&& u)
    {
      m_value.~T();
      ::new(&m_value) T(std::forward<U>(u));
      return m_value;
    }

    T const& get(void) const noexcept
    { return m_value; }

    T& get(void) noexcept
    { return m_value; }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Hash function object that ignores destination location id and magic
  ///        number.
  //////////////////////////////////////////////////////////////////////
  struct hasher
  {
    std::size_t operator()(context_id const& id) const noexcept
    {
      std::size_t seed = 0;
      boost::hash_combine(seed, id.initiator);
      boost::hash_combine(seed, id.current.get_gang_id());
      boost::hash_combine(seed, id.source);
      boost::hash_combine(seed, id.intragang);
      boost::hash_combine(seed, id.nesting);
      return seed;
    }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Equality function object that ignores destination location id and
  ///        magic number.
  //////////////////////////////////////////////////////////////////////
  struct key_equal
  {
    constexpr bool
    operator()(context_id const& x, context_id const& y) const noexcept
    {
      return ((x.initiator             == y.initiator)             &&
              (x.current.get_gang_id() == y.current.get_gang_id()) &&
              (x.source                == y.source)                &&
              (x.intragang             == y.intragang)             &&
              (x.nesting               == y.nesting));
    }
  };


  using lss_container_type =
    typename boost::intrusive::make_set<lss_element_base>::type;

  using magic_nums_map_type =
    std::unordered_map<
      context_id, std::unordered_map<context_id, magic_id>, hasher, key_equal
    >;

  using banned_md_set_type = boost::container::flat_set<gang_md::id>;


  const id                       m_id;
  boost::intrusive_ptr<gang_md>  m_gang_ptr;
  /// Index in @ref gang_md object.
  const size_type                m_idx;
  /// Epoch.
  logical_clock                  m_clock;
  /// SPMD object registry.
  spmd_registry                  m_registry;
  /// Number of registered objects.
  size_type                      m_num_objects;
  fence_md                       m_fence_md;
  /// Location specific storage object container.
  lss_container_type             m_lss;
  runqueue_type                  m_runqueue;
  /// Numbers for @ref context_id disambiguation.
  magic_nums_map_type            m_magic_nums;
  /// Banned gang metadata.
  banned_md_set_type             m_banned_gang_md;
  /// @ref gang_executor of the location.
  std::unique_ptr<executor_base> m_executor;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a pointer to storage for doing placement new of the
  ///        @ref runqueue.
  ///
  /// This is not exactly PIMPL, but very close: it returns a pointer to
  /// storage that is properly aligned and sized to hold a @ref runqueue. It
  /// differs from PIMPL since by overriding the @c new operator, it does not
  /// require heap allocation.
  //////////////////////////////////////////////////////////////////////
  void* get_ptr(void) noexcept;

  //////////////////////////////////////////////////////////////////////
  /// @brief Allocates a new @ref gang_executor to be associated with this
  ///        @ref location_md.
  //////////////////////////////////////////////////////////////////////
  std::unique_ptr<executor_base> make_executor(void);

public:
  void* operator new(std::size_t);
  void operator delete(void*);

  void* operator new[](std::size_t) = delete;
  void operator delete[](void*) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref location_md object.
  ///
  /// @param lid Id of the location.
  /// @param g   Metadata of the gang the location belongs to.
  //////////////////////////////////////////////////////////////////////
  location_md(const id lid,
              gang_md& g)
  : m_id(lid),
    m_gang_ptr(&g, false),
    m_idx(lid),
    m_num_objects(0),
    m_runqueue(get_ptr(), *this)
  { STAPL_RUNTIME_ASSERT(m_id<g.size()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref location_md object.
  ///
  /// @param lid       Id of the location.
  /// @param local_idx Index in the locally managed locations list of @p g.
  /// @param g         Metadata of the gang the location belongs to.
  //////////////////////////////////////////////////////////////////////
  location_md(const id lid,
              const size_type local_idx,
              gang_md& g)
  : m_id(lid),
    m_gang_ptr(&g, false),
    m_idx(local_idx),
    m_num_objects(0),
    m_runqueue(get_ptr(), *this)
  { STAPL_RUNTIME_ASSERT((m_id<g.size()) && (m_idx<g.local_size())); }

  location_md(location_md const&) = delete;
  location_md& operator=(location_md const&) = delete;

  ~location_md(void)
  { m_lss.clear_and_dispose(std::default_delete<lss_element_base>{}); }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if this is the only location in the gang.
  //////////////////////////////////////////////////////////////////////
  bool is_single(void) const noexcept
  { return (get_gang_md().size()==1); }

public:
  id get_id(void) const noexcept
  { return m_id; }

  gang_md const& get_gang_md(void) const noexcept
  { return *m_gang_ptr; }

  gang_md& get_gang_md(void) noexcept
  { return *m_gang_ptr; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if this location is the leader among all the
  ///        locally managed locations.
  //////////////////////////////////////////////////////////////////////
  bool is_leader(void) const noexcept
  { return (m_idx==0); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of this location among all the locally managed
  ///        locations.
  //////////////////////////////////////////////////////////////////////
  size_type local_index(void) const noexcept
  { return m_idx; }

  runqueue_type const& get_runqueue(void) const noexcept
  { return m_runqueue; }

  runqueue_type& get_runqueue(void) noexcept
  { return m_runqueue; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the executor of this @ref location_md.
  //////////////////////////////////////////////////////////////////////
  void set_executor(std::unique_ptr<executor_base> ex) noexcept
  {
    if (m_executor) {
      if (!m_executor->empty()) {
        STAPL_RUNTIME_ERROR("Attempting to replace a gang_executor that has"
                            " entries.");
      }
      if (m_executor->is_bound()) {
        STAPL_RUNTIME_ERROR("Attempting to replace a gang_executor that is"
                            " inserted to the parent parallel section.");
      }
    }
    m_executor = std::move(ex);
  }

  executor_base* try_get_executor(void) noexcept
  { return m_executor.get(); }

  executor_base& get_executor(void)
  {
    if (!m_executor)
      m_executor = make_executor();
    return *m_executor;
  }

// ----------------------------------------------------------------------------
// Metadata management
// ----------------------------------------------------------------------------

  //////////////////////////////////////////////////////////////////////
  /// @brief Generates the magic number to disambiguate between @ref context_id
  ///        objects that should not be the same.
  ///
  /// @see context_id
  //////////////////////////////////////////////////////////////////////
  magic_id disambiguate(context_id const& src, context_id const& dest)
  {
    STAPL_RUNTIME_ASSERT(dest.nesting>=3); // collisions happen when nesting >=3
    std::unordered_map<context_id, magic_id>& m = m_magic_nums[dest];
    magic_id& i = m[src];
    if (i==0) {
      if (m.size() >= std::numeric_limits<magic_id>::max())
        STAPL_RUNTIME_ERROR("Disambiguation numbers depleted.");
      i = m.size();
    }
    return i;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a pointer to the cached gang metadata if it is available,
  ///        otherwise @c nullptr.
  //////////////////////////////////////////////////////////////////////
  gang_md* get_cached_gang_md(const gang_md::id gid)
  {
    STAPL_RUNTIME_ASSERT(gid!=get_gang_md().get_id());
    if (gid==0 || gang_md_registry::id_owner(gid)==runqueue::get_process_id()) {
      // 1. Gang id 0 corresponds to the gang of stapl_main(). Therefore, all
      //    processes have access to it.
      //    If this is no longer true (i.e stapl_main() starts on some processes
      //    only) then the check for gang id 0 has to be removed.
      // 2. If the owner of the gang id is this process, then the metadata
      //    should be available.
      auto& g = gang_md_registry::get(gid);
      return &g;
    }

    if (m_banned_gang_md.count(gid)!=0) {
      // There has been a previous request to this gang without gang metadata
      // available, so the metadata is now banned. Every request has to be
      // forwarded to the gang id owner.
      return nullptr;
    }

    auto* const g = gang_md_registry::try_get(gid);
    if (!g) {
      // Gang metadata is not known. Add the gang id to the banned ones so that
      // that future requests cannot use the metadata even if it becomes
      // available, as doing so could break request ordering.
      m_banned_gang_md.insert(gid);
      return nullptr;
    }

    // Gang metadata has been found and it is not banned.
    return g;
  }

// ----------------------------------------------------------------------------
// Epoch support
// ----------------------------------------------------------------------------

  epoch_type advance_epoch(void)
  {
    const auto e = m_clock.tick();
    m_runqueue.advance_epoch(e);
    return e;
  }

  epoch_type get_epoch(void) const noexcept
  { return m_clock.time(); }

  void undefer_requests(void)
  {
    m_runqueue.undefer_requests();
  }

  void defer_requests(void)
  {
    m_runqueue.defer_requests();
  }

  bool try_defer_requests(void)
  {
    return m_runqueue.try_defer_requests();
  }

// ----------------------------------------------------------------------------
// Object management
// ----------------------------------------------------------------------------

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a handle and registration epoch upon registering @p p.
  ///
  /// If @p avoid_registry is @c true, then the object will not be inserted in a
  /// registry and the epoch will not be increased.
  ///
  /// @warning This function will increase the reference count of this
  ///          @ref location_md object.
  //////////////////////////////////////////////////////////////////////
  std::pair<object_virtual_address, epoch_type>
  register_object(void* const p, const bool avoid_registry)
  {
    // increase ref count to protect this location_md object
    if (++m_num_objects==1)
      this->add_ref();

    if (avoid_registry) {
      // registry can be avoided, do not advance epoch
      return std::make_pair(object_virtual_address{p}, get_epoch());
    }

    // object registered normally, epoch advanced
    const auto e = get_epoch() + 1;
    object_virtual_address vp{m_registry.register_object(p, e)};
    advance_epoch();
    return std::make_pair(vp, e);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Unregisters the pointer associated with handle @p h.
  ///
  /// @warning This function will decrease the reference count of the
  ///          @ref location_md object. If the reference count is 0, it will be
  ///          destroyed.
  //////////////////////////////////////////////////////////////////////
  void unregister_object(object_virtual_address const& h)
  {
    switch (h.get_type()) {
      case object_virtual_address::DIRECT:
        break;

      case object_virtual_address::INDIRECT: {
        auto const& t = h.indirect();
        const bool immediate_unregistration = is_single();
        m_registry.unregister_object(t, immediate_unregistration);
      } break;

      default:
        STAPL_RUNTIME_ERROR("Incorrect handle type.");
        break;
    }

    // decrease ref count, this location_md object may be destroyed
    if (--m_num_objects==0)
      this->release();
  }

  void* get_object(object_virtual_address const& h,
                   const epoch_type e = logical_clock::no_time) const
  {
    switch (h.get_type()) {
      case object_virtual_address::DIRECT:
        return h.direct();

      case object_virtual_address::INDIRECT:
        return m_registry.get_object(h.indirect(), e);

      default:
        STAPL_RUNTIME_ERROR("Incorrect handle type.");
        return nullptr;
    }
  }


// ----------------------------------------------------------------------------
// Fence support
// ----------------------------------------------------------------------------

  //////////////////////////////////////////////////////////////////////
  /// @brief Pre @ref rmi_fence() operations.
  ///
  /// This notifies the @ref runqueue to change the scheduling policy.
  //////////////////////////////////////////////////////////////////////
  void pre_fence(void)
  { m_runqueue.fence_enter(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of unregistrations that will be committed
  /// on the location during the next invocation of @ref rmi_fence.
  //////////////////////////////////////////////////////////////////////
  unsigned int num_pending_unregistrations(void) const
  { return m_registry.num_pending_unregistrations(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Post @ref rmi_fence() operations.
  ///
  /// This makes all unregistered handles from @ref spmd_registry and the SPMD
  /// keys available again.
  //////////////////////////////////////////////////////////////////////
  void post_fence(void)
  {
    m_registry.commit_unregistrations();
    const auto e = m_clock.tick();
    m_runqueue.fence_exit(e);
  }

  fence_md const& get_fence_md(void) const noexcept
  { return m_fence_md; }

  fence_md& get_fence_md(void) noexcept
  { return m_fence_md; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns if there are pending RMIs.
  ///
  /// @warning Only works for gangs with one location.
  //////////////////////////////////////////////////////////////////////
  bool has_pending_rmis(void) const noexcept
  {
    STAPL_RUNTIME_ASSERT(m_gang_ptr->size()==1);
    return ((get_fence_md().pending()!=0) ||
            !m_gang_ptr->get_fence_md().none_pending());
  }

// ----------------------------------------------------------------------------
// Location specific storage
// ----------------------------------------------------------------------------

  //////////////////////////////////////////////////////////////////////
  /// @brief Replaces the object associated with the given address.
  ///
  /// If there is a stored object, it will destroy it.
  ///
  /// @see location_specific_storage
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename U>
  T& reset_lss(void* addr, U&& u)
  {
    using stored_type = lss_element<T>;

    auto it = m_lss.find(lss_element_base{addr});
    if (it!=m_lss.end())
      return static_cast<stored_type&>(*it).reset(std::forward<U>(u));
    auto* const p = new stored_type{addr, std::forward<U>(u)};

    m_lss.insert(*p);
    return p->get();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an object associated with the given address.
  ///
  /// If there is no object, it will create one.
  ///
  /// @see location_specific_storage
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename CreatorFunction>
  T& get_lss(void* addr, CreatorFunction&& f)
  {
    using stored_type = lss_element<T>;

    auto it = m_lss.find(lss_element_base{addr});
    if (it!=m_lss.end())
      return static_cast<stored_type&>(*it).get();

    auto* const p =
      (!f ? new stored_type{addr}
          : new stored_type{addr, std::forward<CreatorFunction>(f), int{}});

    m_lss.insert(*p);
    return p->get();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an object associated with the given address.
  ///
  /// If there is no object, it will not create one.
  ///
  /// @see location_specific_storage
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  T* try_get_lss(void* addr) noexcept
  {
    auto it = m_lss.find(lss_element_base{addr});
    if (it==m_lss.end())
      return nullptr;
    return &(static_cast<lss_element<T>&>(*it).get());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroys the object associated with the given address.
  ///
  /// @see location_specific_storage
  //////////////////////////////////////////////////////////////////////
  void destroy_lss(void* addr) noexcept
  {
    auto it = m_lss.find(lss_element_base{addr});
    if (it!=m_lss.end())
      m_lss.erase_and_dispose(it, std::default_delete<lss_element_base>{});
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns if @p x is equal to @p y.
///
/// Since there is only one @ref location_md object per each location of any
/// gang on one process, the equality comparison is implemented by checking if
/// the pointers to the objects are the same.
///
/// @related location_md
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
constexpr bool operator==(location_md const& x, location_md const& y) noexcept
{
  return (&x==&y);
}

constexpr bool operator!=(location_md const& x, location_md const& y) noexcept
{
  return !(x==y);
}


//////////////////////////////////////////////////////////////////////
/// @brief Enters a section of the code that is related to fence.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class fence_section
{
private:
  location_md& m_location;

public:
  explicit fence_section(location_md& l)
  : m_location(l)
  { m_location.pre_fence(); }

  ~fence_section(void)
  { m_location.post_fence(); }
};

} // namespace runtime

} // namespace stapl

#endif
