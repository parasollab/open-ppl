/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_GANG_MD_HPP
#define STAPL_RUNTIME_GANG_MD_HPP

#include "config.hpp"
#include "exception.hpp"
#include "gang_description.hpp"
#include "gang_md_registry.hpp"
#include "rmi_handle_fwd.hpp"
#include "runqueue.hpp"
#include "tags.hpp"
#include "communicator/topology.hpp"
#include "concurrency/shared_object_factory.hpp"
#include "utility/ref_counted.hpp"
#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <tuple>
#include <utility>
#include <unordered_map>
#include <stapl/utility/hash_fwd.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Metadata that can be shared between multiple gangs.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class common_gang_md
: public ref_counted<common_gang_md>
{
public:
  using size_type = gang_description::size_type;

private:
  const topology         m_topology;
  const gang_description m_desc;

public:
  explicit common_gang_md(const size_type nlocs = 1)
  : m_desc(nlocs, m_topology.get_id())
  { }

  explicit common_gang_md(gang_description gd)
  : m_topology(gd),
    m_desc(std::move(gd))
  { }

  topology const& get_topology(void) const noexcept
  { return m_topology; }

  gang_description const& get_description(void) const noexcept
  { return m_desc; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Gang metadata that is shared by the locations that are on the same
///        process.
///
/// @ref gang_md contains fence metadata for intergang requests (see also
/// @ref context). It acts as a proxy to the contained @ref gang_description to
/// return information on which process each location exists.
///
/// It also contains a @ref topology object that describes the topology of the
/// processes that the gang consists of.
///
/// Finally, it is used by the locations on the same process to create objects
/// that can be shared between them.
///
/// A @ref gang_md object is reference counted. It is destroyed when all the
/// locally managed locations, their associated @ref location_md objects as well
/// as the @ref gang_md objects in the children processes have been destroyed.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class gang_md
: public ref_counted<gang_md>
{
public:
  using id            = gang_id;
  using size_type     = common_gang_md::size_type;
  using runqueue_type = shared_runqueue;

  //////////////////////////////////////////////////////////////////////
  /// @brief Fence metadata for intergang requests.
  //////////////////////////////////////////////////////////////////////
  class fence_md
  {
  public:
    using key_type       = std::pair<id, nesting_level>;
    using container_type =
      std::unordered_map<key_type, int, boost::hash<key_type>>;
  private:
    container_type     m_pending;
    mutable std::mutex m_mtx;

    void update_impl(key_type const& key, const int count)
    {
      auto p = m_pending.emplace(key, count);
      if (p.second)
        return; // no previous value
      int& pending = p.first->second;
      pending     += count;
      if (pending==0)
        m_pending.erase(p.first);
    }

  public:
    void update_sent(const id gid, const nesting_level n,
                     const unsigned int sent)
    {
      std::lock_guard<std::mutex> lock{m_mtx};
      update_impl(key_type{gid, (n+1)}, sent);
    }

    void update_processed(const id gid, const nesting_level n,
                          const unsigned int processed)
    {
      std::lock_guard<std::mutex> lock{m_mtx};
      update_impl(key_type{gid, n}, -processed);
    }

    void update(const id gid, const nesting_level n,
                const unsigned int sent, const unsigned int processed)
    {
      std::lock_guard<std::mutex> lock{m_mtx};
      update_impl(key_type{gid, (n+1)}, sent);
      update_impl(key_type{gid, n}, -processed);
    }

    template<typename T>
    void update(T&& t)
    {
      std::lock_guard<std::mutex> lock{m_mtx};
      for (auto&& p : t)
        update_impl(p.first, p.second);
    }

    container_type retrieve(void)
    {
      std::lock_guard<std::mutex> lock{m_mtx};
      return std::move(m_pending);
    }

    bool none_pending(void) const
    {
      std::lock_guard<std::mutex> lock{m_mtx};
      return m_pending.empty();
    }
  };

private:
  using shared_object_factory_type =
    shared_object_factory<object_virtual_address>;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Updates counts for sent and processed intergang requests.
  //////////////////////////////////////////////////////////////////////
  static void update_fence_md(const gang_md::id gid,
                              fence_md::container_type const& c)
  {
    gang_md* const g = gang_md_registry::try_get(gid);
    if (!g)
      STAPL_RUNTIME_ERROR("Outstanding coherence traffic for destroyed gang "
                          "detected.");
    g->get_fence_md().update(c);
  }

private:
  id                                   m_id;
  const id                             m_parent_id;
  boost::intrusive_ptr<common_gang_md> m_common_md;
  runqueue_type                        m_runqueue;
  fence_md                             m_fence_md;
  /// Shared objects for locally managed locations.
  shared_object_factory_type           m_sh_obj;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates the global gang metadata with id @c 0 for processes with
  ///        @p nlocs locally managed location.
  ///
  /// The object will be automatically registered to the @ref gang_md_registry.
  ///
  /// @param desc  @ref gang_description object associated with this gang.
  /// @param nlocs Number of locally managed locations.
  //////////////////////////////////////////////////////////////////////
  gang_md(gang_description desc, const size_type nlocs)
  : ref_counted<gang_md>(nlocs),
    m_id(0),
    m_parent_id(invalid_gang_id),
    m_common_md(new common_gang_md{std::move(desc)}),
    m_runqueue((m_common_md->get_topology().get_id() * nlocs), nlocs)
  {
    STAPL_RUNTIME_ASSERT(
      get_description().get_num_locations(get_topology().get_id())==nlocs);
    // avoid queuing deferred requests until this gang_md is registered
    auto lock = m_runqueue.acquire(*this);
    gang_md_registry::register_gang_md(*this, m_id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates gang metadata for a single location gang.
  ///
  /// The object will be automatically registered to the @ref gang_md_registry.
  ///
  /// @param parent_id Id of the gang that created this one.
  /// @param c_md      Shared gang metadata.
  //////////////////////////////////////////////////////////////////////
  gang_md(const id parent_id, common_gang_md& c_md)
  : ref_counted<gang_md>(1),
    m_id(gang_md_registry::reserve_id()),
    m_parent_id(parent_id),
    m_common_md(&c_md)
  { gang_md_registry::register_gang_md(*this, m_id); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates gang metadata for a shared memory only gang.
  ///
  /// The object will be automatically registered to the @ref gang_md_registry.
  ///
  /// @param parent_id Id of the gang that created this one.
  /// @param nlocs     Number of locally managed locations.
  //////////////////////////////////////////////////////////////////////
  gang_md(const id parent_id, const size_type nlocs)
  : ref_counted<gang_md>(nlocs),
    m_id(gang_md_registry::reserve_id()),
    m_parent_id(parent_id),
    m_common_md(new common_gang_md{nlocs}),
    m_runqueue(0, nlocs)
  {
    STAPL_RUNTIME_ASSERT(nlocs>1);
    gang_md_registry::register_gang_md(*this, m_id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates gang metadata with a known id.
  ///
  /// The object will be automatically registered to the @ref gang_md_registry.
  ///
  /// @param gid       Gang id.
  /// @param parent_id Parent gang id.
  /// @param desc      @ref gang_description object associated with this gang.
  //////////////////////////////////////////////////////////////////////
  gang_md(const id gid, const id parent_id, gang_description desc)
  : m_id(gid),
    m_parent_id(parent_id),
    m_common_md(new common_gang_md{std::move(desc)}),
    m_runqueue(m_common_md->get_description(),
               m_common_md->get_topology().get_id())
  {
    STAPL_RUNTIME_ASSERT(
      !get_description().is_on_shmem() &&
      local_size()==get_description().get_num_locations(get_topology().get_id())
    );
    this->add_ref(local_size() + get_topology().children().size());
    // avoid queuing deferred requests until this gang_md is registered
    auto lock = m_runqueue.acquire(*this);
    gang_md_registry::register_gang_md(*this, m_id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates gang metadata with an automatically generated id.
  ///
  /// The object will be automatically registered to the @ref gang_md_registry.
  ///
  /// @param parent_id Parent gang id.
  /// @param desc      @ref gang_description object associated with the gang.
  //////////////////////////////////////////////////////////////////////
  gang_md(const id parent_id, gang_description desc)
  : m_id(gang_md_registry::reserve_id()),
    m_parent_id(parent_id),
    m_common_md(new common_gang_md{std::move(desc)}),
    m_runqueue(m_common_md->get_description(),
               m_common_md->get_topology().get_id())
  {
    STAPL_RUNTIME_ASSERT(
      local_size()==get_description().get_num_locations(get_topology().get_id())
    );
    this->add_ref(local_size() + get_topology().children().size());
    gang_md_registry::register_gang_md(*this, m_id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates gang metadata with an unknown id.
  ///
  /// The object will be registered by @ref set_id().
  ///
  /// @param parent_id Parent gang id.
  /// @param desc      @ref gang_description object associated with the gang.
  //////////////////////////////////////////////////////////////////////
  gang_md(const deferred_t, const id parent_id, gang_description desc)
  : m_id(invalid_gang_id),
    m_parent_id(parent_id),
    m_common_md(new common_gang_md{std::move(desc)}),
    m_runqueue(m_common_md->get_description(),
               m_common_md->get_topology().get_id())
  {
    STAPL_RUNTIME_ASSERT(
      !get_description().is_on_shmem() &&
      local_size()==get_description().get_num_locations(get_topology().get_id())
    );
    this->add_ref(local_size() + get_topology().children().size());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates gang metadata with a known id.
  ///
  /// The object will be automatically registered to the @ref gang_md_registry.
  ///
  /// @param gid       Gang id.
  /// @param parent_id Parent gang id.
  /// @param desc      @ref gang_description object associated with this gang.
  /// @param r         Locally managed location id range.
  //////////////////////////////////////////////////////////////////////
  template<typename Range>
  gang_md(const id gid, const id parent_id, gang_description desc, Range&& r)
  : m_id(gid),
    m_parent_id(parent_id),
    m_common_md(new common_gang_md{std::move(desc)}),
    m_runqueue(std::forward<Range>(r))
  {
    STAPL_RUNTIME_ASSERT(
      !get_description().is_on_shmem() &&
      local_size()==get_description().get_num_locations(get_topology().get_id())
    );
    this->add_ref(local_size() + get_topology().children().size());
    // avoid queuing deferred requests until this gang_md is registered
    auto lock = m_runqueue.acquire(*this);
    gang_md_registry::register_gang_md(*this, m_id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates gang metadata with an automatically generated id.
  ///
  /// The object will be automatically registered to the @ref gang_md_registry.
  ///
  /// @param parent_id Parent gang id.
  /// @param desc      @ref gang_description object associated with the gang.
  /// @param r         Locally managed location id range.
  //////////////////////////////////////////////////////////////////////
  template<typename Range>
  gang_md(const id parent_id, gang_description desc, Range&& r)
  : m_id(gang_md_registry::reserve_id()),
    m_parent_id(parent_id),
    m_common_md(new common_gang_md{std::move(desc)}),
    m_runqueue(std::forward<Range>(r))
  {
    STAPL_RUNTIME_ASSERT(
      local_size()==get_description().get_num_locations(get_topology().get_id())
    );
    this->add_ref(local_size() + get_topology().children().size());
    gang_md_registry::register_gang_md(*this, m_id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates gang metadata that is identical to @p other with a known
  ///        id.
  ///
  /// The object will be automatically registered to the @ref gang_md_registry.
  ///
  /// @param gid       Gang id.
  /// @param parent_id Parent gang id.
  /// @param other     Gang to clone.
  //////////////////////////////////////////////////////////////////////
  gang_md(const id gid, const id parent_id, gang_md const& other)
  : m_id(gid),
    m_parent_id(parent_id),
    m_common_md(other.m_common_md),
    m_runqueue(other.m_runqueue)
  {
    this->add_ref(local_size() + get_topology().children().size());
    // avoid queuing deferred requests until this gang_md is registered
    auto lock = m_runqueue.acquire(*this);
    gang_md_registry::register_gang_md(*this, m_id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates gang metadata that is identical to @p other with an
  ///        automatically generated id.
  ///
  /// The object will be automatically registered to the @ref gang_md_registry.
  ///
  /// @param parent_id Parent gang id.
  /// @param other     Gang to clone.
  //////////////////////////////////////////////////////////////////////
  gang_md(const id parent_id, gang_md const& other)
  : m_id(gang_md_registry::reserve_id()),
    m_parent_id(parent_id),
    m_common_md(other.m_common_md),
    m_runqueue(other.m_runqueue)
  {
    this->add_ref(local_size() + get_topology().children().size());
    gang_md_registry::register_gang_md(*this, m_id);
  }

  ~gang_md(void)
  {
    STAPL_RUNTIME_ASSERT(m_id!=invalid_gang_id);
    gang_md_registry::unregister_gang_md(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the gang id and automatically registers @c *this to the
  ///        @ref gang_md_registry.
  ///
  /// @warning This function is only valid if the @ref gang_md was constructed
  ///          without an id.
  ///
  /// @param gid Id to register the gang metadata with.
  //////////////////////////////////////////////////////////////////////
  void set_id(const id gid)
  {
    STAPL_RUNTIME_ASSERT(m_id==invalid_gang_id);
    m_id = gid;
    // avoid enqueuing deferred requests until this gang_md is registered
    auto lock = m_runqueue.acquire(*this);
    gang_md_registry::register_gang_md(*this, m_id);
  }

  id get_id(void) const noexcept
  { return m_id; }

  id get_parent_id(void) const noexcept
  { return m_parent_id; }

  gang_description const& get_description(void) const noexcept
  { return m_common_md->get_description(); }

  topology const& get_topology(void) const noexcept
  { return m_common_md->get_topology(); }

  runqueue_type const& get_runqueue(void) const noexcept
  { return m_runqueue; }

  runqueue_type& get_runqueue(void) noexcept
  { return m_runqueue; }

  fence_md const& get_fence_md(void) const noexcept
  { return m_fence_md; }

  fence_md& get_fence_md(void) noexcept
  { return m_fence_md; }

  size_type size(void) const noexcept
  { return get_description().get_num_locations(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of locally managed locations.
  //////////////////////////////////////////////////////////////////////
  size_type local_size(void) const noexcept
  { return m_runqueue.size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns if the location id exists in this gang.
  //////////////////////////////////////////////////////////////////////
  bool exists(const location_id lid) const noexcept
  { return (lid<size()); }

  process_id get_process_id(const location_id lid) const noexcept
  { return get_description().get_process_id(lid); }

  bool conforms_with(gang_md const& g) const noexcept
  { return (m_common_md==g.m_common_md); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an object that is shared among all the locally managed
  ///        locations.
  ///
  /// The returned object is valid for @ref local_size() requesting locations
  /// and is indexed with @p h.
  ///
  /// @warning Only one shared object is allowed per handle @p h.
  ///
  /// @param h    Handle to index the shared object.
  /// @param args Arguments to pass to the constructor of the shared object.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Handle, typename... Args>
  std::shared_ptr<T> get_shared_object(Handle const& h, Args&&... args)
  {
    return m_sh_obj.create<T>(h.internal_handle(),
                              local_size(),
                              std::forward<Args>(args)...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns if @p x is equal to @p y.
///
/// Since there is only one @ref gang_md object per process, the equality
/// comparison is implemented by checking if the pointers to the objects are the
/// same.
///
/// @related gang_md
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
constexpr bool operator==(gang_md const& x, gang_md const& y) noexcept
{
  return (&x==&y);
}

constexpr bool operator!=(gang_md const& x, gang_md const& y) noexcept
{
  return !(x==y);
}


class fence_md;

//////////////////////////////////////////////////////////////////////
/// @brief Destroys @ref fence_md objects while sending the metadata to the
///        correct process.
///
/// @related fence_md
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
struct fence_md_delete
{
  void operator()(fence_md* p) const
  { gang_md_registry::erase_fence_md(p); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Fence metadata for intergang communication when a @ref gang_md has no
///        representative.
///
/// @related gang_md
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class fence_md
: public ref_counted<fence_md, fence_md_delete>,
  public gang_md::fence_md
{
private:
  gang_md::id m_gid;

public:
  explicit fence_md(const gang_md::id gid)
  : m_gid(gid)
  { }

  gang_md::id get_gang_id(void) const noexcept
  { return m_gid; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Retrieves the @ref gang_md& associated with the gang id.
///
/// @related gang_md
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class gang_retriever
{
public:
  using member_types = std::tuple<gang_md::id>;

private:
  gang_md::id m_gid;

public:
  constexpr explicit
  gang_retriever(const gang_md::id gid = invalid_gang_id) noexcept
  : m_gid(gid)
  { }

  gang_md& operator()(void) const
  { return gang_md_registry::get(m_gid); }

  operator gang_md&(void) const
  { return operator()(); }
};

} // namespace runtime

} // namespace stapl

#endif
