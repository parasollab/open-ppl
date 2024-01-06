/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_NON_RMI_CONSTRUCT_HPP
#define STAPL_RUNTIME_NON_RMI_CONSTRUCT_HPP

#include "rpc.hpp"
#include "../context.hpp"
#include "../future.hpp"
#include "../gang.hpp"
#include "../location_range.hpp"
#include "../promise.hpp"
#include "../rmi_handle.hpp"
#include "../runqueue.hpp"
#include "../tags.hpp"
#include "../request/arg_storage.hpp"
#include "../request/location_rpc_request.hpp"
#include "../utility/ref_counted.hpp"
#include <algorithm>
#include <functional>
#include <iterator>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <boost/range/adaptor/transformed.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Container that associates parent gang to child gang location ids only
///        for locally managed locations.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class managed_locations_info
: public ref_counted<managed_locations_info>
{
public:
  using id             = location_md::id;
  using size_type      = location_md::size_type;
private:
  using container_type = std::unordered_map<id, std::pair<id, size_type>>;
public:
  using value_type     = typename container_type::value_type;
  using const_iterator = typename container_type::const_iterator;
  using iterator       = const_iterator;
private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns parent gang location ids.
  //////////////////////////////////////////////////////////////////////
  struct extract_location_id
  {
    id operator()(value_type const& v) const noexcept
    { return v.first; }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns child gang location ids.
  //////////////////////////////////////////////////////////////////////
  struct extract_mapped_location_id
  {
    id operator()(value_type const& v) const noexcept
    { return v.second.first; }
  };

  container_type m_map;

public:
  template<typename Range>
  managed_locations_info(gang_description const& gd, Range const& r)
  {
    if (gd.is_on_shmem()) {
      id i = 0;
      for (auto it = std::begin(r); it != std::end(r); ++it, ++i) {
        const auto lid = *it;
        m_map.emplace(lid, std::make_pair(i, i));
      }
    }
    else if (gd.is_on_distmem()) {
      const auto my_pid = runqueue::get_process_id();
      id i = 0;
      for (auto it = std::begin(r); it != std::end(r); ++it, ++i) {
        const auto lid = *it;
        if (gd.get_process_id(lid)==my_pid) {
          m_map.emplace(lid, std::make_pair(i, 0));
          break;
        }
      }
    }
    else {
      const auto my_pid = runqueue::get_process_id();
      id i = 0;
      for (auto it = std::begin(r); it != std::end(r); ++it, ++i) {
        const auto lid = *it;
        if (gd.get_process_id(lid)==my_pid)
          m_map.emplace(lid, std::make_pair(i, m_map.size()));
      }
    }
  }

  size_type size(void) const noexcept
  { return m_map.size(); }

  const_iterator begin(void) const
  { return m_map.begin(); }

  const_iterator end(void) const
  { return m_map.end(); }

  std::pair<id, size_type> const& find(const id lid) const noexcept
  {
    auto it = m_map.find(lid);
    STAPL_RUNTIME_ASSERT(it!=m_map.end());
    return it->second;
  }

  auto get_location_ids(void)
    -> decltype(boost::adaptors::transform(
                  std::declval<ref_counted_range<managed_locations_info>>(),
                  extract_location_id{}))
  {
    return boost::adaptors::transform(
             ref_counted_range<managed_locations_info>{*this},
             extract_location_id{});
  }

  auto get_mapped_location_ids(void)
    -> decltype(boost::adaptors::transform(
                  std::declval<ref_counted_range<managed_locations_info>>(),
                  extract_mapped_location_id{}))
  {
    return boost::adaptors::transform(
             ref_counted_range<managed_locations_info>{*this},
             extract_mapped_location_id{});
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Request to construct an object in a new single location gang.
///
/// @tparam Callback Callback to pass a pointer to the constructed object.
/// @tparam Function Function that creates a new object.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Callback, typename Function>
class single_location_construct_request final
: public location_rpc_request,
  private arg_storage_t<Callback, Callback const&>,
  private arg_storage_t<Function, Function const&>
{
private:
  using callback_storage_type = arg_storage_t<Callback, Callback const&>;
  using function_storage_type = arg_storage_t<Function, Function const&>;

public:
  template<typename C, typename F>
  static std::size_t expected_size(C&& c, F&& f) noexcept
  {
    return (sizeof(single_location_construct_request)              +
            callback_storage_type::packed_size(std::forward<C>(c)) +
            function_storage_type::packed_size(std::forward<F>(f)));
  }

  template<typename C, typename F>
  single_location_construct_request(C&& c, F&& f) noexcept
  : location_rpc_request(sizeof(*this)),
    callback_storage_type(std::forward<C>(c), this, this->size()),
    function_storage_type(std::forward<F>(f), this, this->size())
  { }

  bool operator()(location_md&, message_shared_ptr&) final
  {
    {
      gang g;
      auto* const t = function_storage_type::get(this)();
      callback_storage_type::get(this)(t);
    }

    this->~single_location_construct_request();
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Request to construct an object on all locally managed locations.
///
/// @tparam Callback           Callback to pass a pointer to the constructed
///                            object.
/// @tparam Function           Function that creates a new object.
/// @tparam LocallyManagedOnly If @c true, then this request executes only on
///                            the locally managed locations.
///
/// This request always runs on the same process as the gang it references,
/// therefore it is correct to have a reference to the @ref gang_md object.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Callback, typename Function, bool LocallyManagedOnly>
class all_locs_construct_request final
: public location_rpc_request,
  private arg_storage_t<Callback, Callback const&>,
  private arg_storage_t<Function, Function const&>
{
private:
  using callback_storage_type = arg_storage_t<Callback, Callback const&>;
  using function_storage_type = arg_storage_t<Function, Function const&>;

  gang_md& m_g;

public:
  template<typename C, typename F>
  static std::size_t expected_size(C&& c, F&& f) noexcept
  {
    return (sizeof(all_locs_construct_request)                     +
            callback_storage_type::packed_size(std::forward<C>(c)) +
            function_storage_type::packed_size(std::forward<F>(f)));
  }

  template<typename C, typename F>
  all_locs_construct_request(C&& c, gang_md& g, F&& f) noexcept
  : location_rpc_request(sizeof(*this)),
    callback_storage_type(std::forward<C>(c), this, this->size()),
    function_storage_type(std::forward<F>(f), this, this->size()),
    m_g(g)
  { }

  bool operator()(location_md& l, message_shared_ptr&) final
  {
    // create metadata
    auto* const nl = (LocallyManagedOnly
                       ? new location_md(l.local_index(), m_g)
                       : new location_md{l.get_id(), l.local_index(), m_g});

    // create object
    {
      context ctx{*nl};
      auto* const t = function_storage_type::get(this)();
      callback_storage_type::get(this)(t);
    }

    this->~all_locs_construct_request();
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Request to construct an object in a new gang over specific locations
///        of an existing gang.
///
/// @tparam Callback        Callback to pass a pointer to the constructed
///                         object.
/// @tparam Function        Function that creates a new object.
/// @tparam LocationMapping Associative container with the mapping from current
///                         to new location id.
///
/// This request has a reference counted pointer to a shared data structure that
/// maps current gang location ids to new gang location ids.
///
/// The reference count is increased prior to assigning the pointer in the
/// request, as requests are memcpy'd instead of fully constructed. The
/// destructor of the request will assure that the last thread will delete the
/// shared data structure.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Callback, typename Function, typename LocationMapping>
class construct_request final
: public location_rpc_request,
  private arg_storage_t<Callback, Callback const&>,
  private arg_storage_t<Function, Function const&>
{
private:
  using callback_storage_type = arg_storage_t<Callback, Callback const&>;
  using function_storage_type = arg_storage_t<Function, Function const&>;

  gang_md&                              m_g;
  boost::intrusive_ptr<LocationMapping> m_locs_info;

public:
  template<typename C, typename F>
  static std::size_t expected_size(C&& c, F&& f) noexcept
  {
    return (sizeof(construct_request)                              +
            callback_storage_type::packed_size(std::forward<C>(c)) +
            function_storage_type::packed_size(std::forward<F>(f)));
  }

  template<typename C, typename Map, typename F>
  construct_request(C&& c, gang_md& g, Map&& m, F&& f) noexcept
  : location_rpc_request(sizeof(*this)),
    callback_storage_type(std::forward<C>(c), this, this->size()),
    function_storage_type(std::forward<F>(f), this, this->size()),
    m_g(g),
    m_locs_info(std::forward<Map>(m))
  { }

  bool operator()(location_md& l, message_shared_ptr&) final
  {
    // create metadata by mapping current location id to new gang location id
    const auto lid     = l.get_id();
    const auto mapping = m_locs_info->find(lid);
    auto* const nl     = new location_md{mapping.first, mapping.second, m_g};

    // create object
    {
      context ctx{*nl};
      auto* const t = function_storage_type::get(this)();
      callback_storage_type::get(this)(t);
    }

    this->~construct_request();
    return true;
  }
};


template<typename ProcessRange, typename Callback, typename Function>
void call_construct_all_impl(ProcessRange&&,
                             Callback&&,
                             const gang_md::id,
                             const context::epoch_type,
                             const gang_md::id,
                             Function&&);

template<typename ProcessRange,
         typename Callback,
         typename T,
         typename Function>
void call_construct_range_impl(ProcessRange&&,
                               Callback&&,
                               const gang_md::id,
                               location_range_wrapper<T> const&,
                               const context::epoch_type,
                               const gang_md::id,
                               Function&&);


//////////////////////////////////////////////////////////////////////
/// @brief Creates a distributed object through @p f on all locations of gang
///        @p g.
///
/// @param c       Callback to pass a pointer to the constructed object.
/// @param g       Destination gang metadata.
/// @param e       Caller epoch.
/// @param new_gid New gang id; if @ref invalid_gang_id, then a new id will be
///                generated.
/// @param f       Object creation function.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Callback, typename Function>
void construct_all_impl(Callback&& c,
                        gang_md& g,
                        const context::epoch_type e,
                        const gang_md::id new_gid,
                        Function&& f)
{
  const auto gid    = g.get_id();
  auto* const new_g = (new_gid==invalid_gang_id ? new gang_md{gid, g}
                                                : new gang_md{new_gid, gid, g});

  // forward information to children processes
  auto const& children = new_g->get_topology().children();
  if (!children.empty())
    call_construct_all_impl(children, c, gid, e, new_g->get_id(), f);

  // create object on all locally managed locations
  using request_type = all_locs_construct_request<
                         typename std::decay<Callback>::type,
                         typename std::decay<Function>::type,
                         false>;

  managed_locations_rpc_aggregator a{g, e};
  const auto size = request_type::expected_size(std::forward<Callback>(c),
                                                std::forward<Function>(f));
  new(a.allocate(size)) request_type{std::forward<Callback>(c),
                                     *new_g,
                                     std::forward<Function>(f)};
}


//////////////////////////////////////////////////////////////////////
/// @brief Calls @ref construct_all_impl() on processes @p pids.
///
/// @param pids    Destination processes.
/// @param c       Callback to pass a pointer to the constructed object.
/// @param gid     Destination gang id.
/// @param e       Caller epoch.
/// @param new_gid New gang id.
/// @param f       Object creation function.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename ProcessRange, typename Callback, typename Function>
void call_construct_all_impl(ProcessRange&& pids,
                             Callback&& c,
                             const gang_md::id gid,
                             const context::epoch_type e,
                             const gang_md::id new_gid,
                             Function&& f)
{
  rpc(&construct_all_impl<decltype(c), decltype(f)>,
      std::forward<ProcessRange>(pids),
      std::forward<Callback>(c),
      gang_retriever{gid}, e, new_gid, std::forward<Function>(f));
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates a distributed object through @p f on the subset @p locs of
///        the locations of the gang associated with @p g.
///
/// @param c       Callback to pass a pointer to the constructed object.
/// @param g       Destination gang metadata.
/// @param r       Destination locations of @p g.
/// @param e       Caller epoch.
/// @param new_gid New gang id; if @ref invalid_gang_id, then a new id will be
///                generated.
/// @param f       Object creation function.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Callback, typename T, typename Function>
void construct_range_impl(Callback&& c,
                          gang_md& g,
                          location_range_wrapper<T> const& r,
                          const context::epoch_type e,
                          const gang_md::id new_gid,
                          Function&& f)
{
  // find participating locations on this process and create index in new gang
  auto managed_locs =
    make_ref_counted<managed_locations_info>(g.get_description(), r);
  const auto num_managed = managed_locs->size();
  managed_locs->add_ref(num_managed - 1);

  gang_md* new_g = nullptr;
  if (num_managed==r.size()) {
    // new gang only on shared memory
    STAPL_RUNTIME_ASSERT(new_gid==invalid_gang_id);
    new_g = new gang_md{g.get_id(), num_managed};
  }
  else {
    // create gang metadata
    const auto gid = g.get_id();
    gang_description new_gd{g.get_description(),
                            arbitrary<location_md::id, location_md::id>{r},
                            r.size()};
    auto new_lids = managed_locs->get_mapped_location_ids();
    new_g =
      ((new_gid==invalid_gang_id)
        ? new gang_md{gid, std::move(new_gd), std::move(new_lids)}
        : new gang_md{new_gid, gid, std::move(new_gd), std::move(new_lids)});

    // forward information to children processes
    auto const& children = new_g->get_topology().children();
    if (!children.empty())
      call_construct_range_impl(children, c, gid, r, e, new_g->get_id(), f);
  }

  // send to locally managed locations
  using request_type = construct_request<
                         typename std::decay<Callback>::type,
                         typename std::decay<Function>::type,
                         managed_locations_info>;
  managed_locations_rpc_aggregator a{g, managed_locs->get_location_ids(), e};
  const auto size = request_type::expected_size(std::forward<Callback>(c),
                                                std::forward<Function>(f));
  new(a.allocate(size)) request_type{std::forward<Callback>(c),
                                     *new_g,
                                     std::move(managed_locs),
                                     std::forward<Function>(f)};
}


//////////////////////////////////////////////////////////////////////
/// @brief Calls @ref construct_range_impl() on processes @p pids.
///
/// @param pids    Destination processes.
/// @param c       Callback to pass a pointer to the constructed object.
/// @param gid     Destination gang id.
/// @param r       Destination locations of @p g.
/// @param e       Caller epoch.
/// @param new_gid New gang id.
/// @param f       Object creation function.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename ProcessRange,
         typename Callback,
         typename T,
         typename Function>
void call_construct_range_impl(ProcessRange&& pids,
                               Callback&& c,
                               const gang_md::id gid,
                               location_range_wrapper<T> const& r,
                               const context::epoch_type e,
                               const gang_md::id new_gid,
                               Function&& f)
{
  rpc(&construct_range_impl<decltype(c), T, decltype(f)>,
      std::forward<ProcessRange>(pids),
      std::forward<Callback>(c),
      gang_retriever{gid}, r, e, new_gid, std::forward<Function>(f));
}


//////////////////////////////////////////////////////////////////////
/// @brief Target for forwarding the request to @ref construct_range_impl().
///
/// @param c Callback to pass a pointer to the constructed object.
/// @param g Destination gang metadata.
/// @param r Destination locations of @p g.
/// @param e Caller epoch.
/// @param f Object creation function.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Callback, typename T, typename Function>
void fwd_construct_range_impl(Callback&& c,
                              gang_md& g,
                              location_range_wrapper<T> const& r,
                              const context::epoch_type e,
                              Function&& f)
{
  const auto pid = g.get_process_id(*std::begin(r));
  if (runqueue::get_process_id()==pid) {
    construct_range_impl(std::forward<Callback>(c),
                         g, r, e,
                         invalid_gang_id, std::forward<Function>(f));
  }
  else {
    call_construct_range_impl(pid,
                              std::forward<Callback>(c),
                              g.get_id(), r, e,
                              invalid_gang_id, std::forward<Function>(f));
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Calls @c promise<rmi_handle::reference>::set_value() from location 0.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class call_promise_set_value
{
public:
  using promise_type = promise<rmi_handle::reference>;

private:
  promise_type m_promise;

public:
  explicit call_promise_set_value(promise_type p)
  : m_promise(std::move(p))
  { }

  template<typename T>
  void operator()(T* const t)
  {
    if (t->get_location_id()==0)
      m_promise.set_value(t->get_rmi_handle());
  }

  void define_type(typer& t)
  { t.member(m_promise); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Calls the constructor of @p T with @p Args.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
struct create_object
{
  template<typename... Args>
  T* operator()(Args&&... args) const
  { return new T(std::forward<Args>(args)...); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Creates a function object used to construct an instance @p T with
///        @p args.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T, typename... Args>
auto call_constructor(Args&&... args)
  -> decltype(std::bind(create_object<T>{}, std::forward<Args>(args)...))
{
  return std::bind(create_object<T>{}, std::forward<Args>(args)...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates a distributed object through @p f in a new single location
///        gang over location @p lid of gang @p g.
///
/// @param c   Callback to pass a pointer to the constructed object.
/// @param gid Destination gang id.
/// @param g   Destination gang metadata.
/// @param lid Destination location id.
/// @param e   Caller epoch.
/// @param f   Object creation function.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename Callback, typename Function>
void construct_single(Callback&& c,
                      gang_md::id gid,
                      gang_md* g,
                      const location_md::id lid,
                      const context::epoch_type e,
                      Function&& f)
{
  using request_type = single_location_construct_request<
                         typename std::decay<Callback>::type,
                         typename std::decay<Function>::type>;
  const auto size = request_type::expected_size(std::forward<Callback>(c),
                                                std::forward<Function>(f));
  if (g) {
    location_rpc_aggregator a{*g, lid, e};
    new(a.allocate(size)) request_type{std::forward<Callback>(c),
                                       std::forward<Function>(f)};
  }
  else {
    location_rpc_aggregator a{gid, lid, e};
    new(a.allocate(size)) request_type{std::forward<Callback>(c),
                                       std::forward<Function>(f)};
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates a distributed object through @p f in a new gang over all the
///        locations of the gang associated with @p ctx that are on shared
///        memory.
///
/// @param c Callback to pass a pointer to the constructed object.
/// @param g Destination gang metadata.
/// @param e Caller epoch.
/// @param f Object creation function.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename Callback, typename Function>
void construct_neighbors(Callback&& c,
                         gang_md& g,
                         const context::epoch_type e,
                         Function&& f)
{
  const auto nmanaged = g.local_size();
  managed_locations_rpc_aggregator a{g, e};
  if (nmanaged==1) {
    using request_type = single_location_construct_request<
                           typename std::decay<Callback>::type,
                           typename std::decay<Function>::type>;

    const std::size_t size =
      request_type::expected_size(std::forward<Callback>(c),
                                  std::forward<Function>(f));
    new(a.allocate(size)) request_type{std::forward<Callback>(c),
                                       std::forward<Function>(f)};
  }
  else {
    using request_type = all_locs_construct_request<
                           typename std::decay<Callback>::type,
                           typename std::decay<Function>::type,
                           true>;

    auto* const ng         = new gang_md{g.get_id(), nmanaged};
    const std::size_t size =
      request_type::expected_size(std::forward<Callback>(c),
                                  std::forward<Function>(f));
    new(a.allocate(size)) request_type{std::forward<Callback>(c),
                                       *ng,
                                       std::forward<Function>(f)};
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates a distributed object through @p f in a new gang over all
///        locations of the gang of @p h.
///
/// @param c   Callback to pass a pointer to the constructed object.
/// @param gid Destination gang id.
/// @param g   Destination gang metadata.
/// @param e   Caller epoch.
/// @param f   Object creation function.
///
/// @ingroup requestBuildingBlock
///
/// @todo Everything is forwarded to the gang id owner process. Needs to be
///       avoided for scalability.
/// @todo Optimize for gangs of size 1.
////////////////////////////////////////////////////////////////////
template<typename Callback, typename Function>
void construct_all(Callback&& c,
                   const gang_md::id gid,
                   gang_md* g,
                   const context::epoch_type e,
                   Function&& f)
{
  const auto pid = gang_md_registry::id_owner(gid);
  if (runqueue::get_process_id()==pid) {
    // owner
    if (!g)
      STAPL_RUNTIME_ERROR("Gang metadata should be available at the owner.");
    construct_all_impl(std::forward<Callback>(c),
                       *g, e,
                       invalid_gang_id, std::forward<Function>(f));
  }
  else {
    // forward request to the owner process
    call_construct_all_impl(pid,
                            std::forward<Callback>(c),
                            gid, e,
                            invalid_gang_id, std::forward<Function>(f));
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates a distributed object through @p f in a new gang over the
///        range @p r of the locations of the gang associated with @p g.
///
/// @param c   Callback to pass a pointer to the constructed object.
/// @param gid Destination gang id.
/// @param g   Destination gang metadata.
/// @param r   Destination locations of @p g.
/// @param e   Caller epoch.
/// @param f   Object creation function.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename Callback, typename T, typename Function>
void construct_range(Callback&& c,
                     gang_md::id gid,
                     gang_md* g,
                     location_range_wrapper<T> const& r,
                     const context::epoch_type e,
                     Function&& f)
{
  if (r.size()==1) {
    construct_single(std::forward<Callback>(c),
                     gid,
                     g,
                     *std::begin(r),
                     e,
                     std::forward<Function>(f));
  }
  else {
    if (!g) {
      // gang metadata unavailable; forward to owner
      const auto pid = gang_md_registry::id_owner(gid);
      if (pid==runqueue::get_process_id())
        STAPL_RUNTIME_ERROR("Gang metadata should be available at the owner.");
      rpc(&fwd_construct_range_impl<decltype(c), T, decltype(f)>,
          pid,
          std::forward<Callback>(c),
          gang_retriever{gid}, r, e, std::forward<Function>(f));
    }
    else {
      fwd_construct_range_impl(std::forward<Callback>(c),
                               *g, r, e, std::forward<Function>(f));
    }
  }
}

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Creates an object of type @p T in a new gang over all the locations
///        of the current gang that are on shared memory.
///
/// @param args Arguments to pass to the constructor of @p T.
///
/// @return A @ref future object with an @ref rmi_handle::reference to the
///         constructed object.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T, typename... Args>
future<rmi_handle::reference> construct(neighbor_locations_t,
                                        Args&&... args)
{
  using namespace runtime;

  auto& ctx = this_context::get();

  promise<rmi_handle::reference> p;
  auto f = p.get_future();
  {
    STAPL_RUNTIME_PROFILE("construct(neighbor_locations)",
                          (primitive_traits::non_blocking |
                           primitive_traits::p2m          |
                           primitive_traits::comm));
    construct_neighbors(call_promise_set_value{std::move(p)},
                        ctx.get_gang_md(),
                        ctx.get_epoch(),
                        call_constructor<T>(std::forward<Args>(args)...));
  }

  scheduling_point(ctx);
  return f;
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates an object of type @p T in a new gang over the range @p r of
///        the locations of the current gang.
///
/// @warning @p r may only contain locations of the current gang.
///
/// @param r    Location id range of the current that the new gang will be
///             created in.
/// @param args Arguments to pass to the constructor of @p T.
///
/// @return A @ref future object with an @ref rmi_handle::reference to the
///         constructed object.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T, typename U, typename... Args>
future<rmi_handle::reference> construct(location_range_wrapper<U> const& r,
                                        Args&&... args)
{
  using namespace runtime;

  auto& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(!r.has_invalid(ctx.get_gang_md().size()),
                           "Range contains invalid location id.");

  promise<rmi_handle::reference> p;
  auto f = p.get_future();
  {
    STAPL_RUNTIME_PROFILE("construct(range)", (primitive_traits::non_blocking |
                                               primitive_traits::p2m          |
                                               primitive_traits::comm));
    construct_range(call_promise_set_value{std::move(p)},
                    ctx.get_gang_id(),
                    &(ctx.get_gang_md()),
                    r,
                    ctx.get_epoch(),
                    call_constructor<T>(std::forward<Args>(args)...));
  }

  scheduling_point(ctx);
  return f;
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates an object of type @p T in a new gang over the range @p r of
///        the locations of the current gang and calls @p f with the object
///        pointer.
///
/// @warning @p r may only contain locations of the current gang.
///
/// @param f    Function to pass the pointer of the constructed object.
/// @param r    Location id range of the current that the new gang will be
///             created in.
/// @param args Arguments to pass to the constructor of @p T.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T, typename Function, typename U, typename... Args>
void async_construct(Function&& f,
                     location_range_wrapper<U> const& r,
                     Args&&... args)
{
  using namespace runtime;

  auto& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(!r.has_invalid(ctx.get_gang_md().size()),
                           "Range contains invalid location id.");

  {
    STAPL_RUNTIME_PROFILE("async_construct(range)",
                          (primitive_traits::non_blocking |
                           primitive_traits::p2m          |
                           primitive_traits::comm));
    construct_range(std::forward<Function>(f),
                    ctx.get_gang_id(),
                    &(ctx.get_gang_md()),
                    r,
                    ctx.get_epoch(),
                    call_constructor<T>(std::forward<Args>(args)...));
  }

  scheduling_point(ctx);
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates an object of type @p T in a new gang over all the locations
///        of the gang of @p h.
///
/// @param h    Target distributed object handle.
/// @param args Arguments to pass to the constructor of @p T.
///
/// @return A @ref future object with an @ref rmi_handle::reference to the
///         constructed object.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T, typename... Args>
future<rmi_handle::reference> construct(rmi_handle::const_reference const& h,
                                        all_locations_t,
                                        Args&&... args)
{
  using namespace runtime;

  auto& ctx = this_context::get();

  promise<rmi_handle::reference> p;
  auto f = p.get_future();
  {
    STAPL_RUNTIME_PROFILE("construct(handle,all_locations)",
                          (primitive_traits::non_blocking |
                           primitive_traits::p2m          |
                           primitive_traits::comm));
    construct_all(call_promise_set_value{std::move(p)},
                  h.get_gang_id(),
                  find_target_gang_md(ctx, h),
                  h.get_epoch(),
                  call_constructor<T>(std::forward<Args>(args)...));
  }

  scheduling_point(ctx);
  return f;
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates an object of type @p T in a new gang over all the locations
///        of the gang of @p h and calls @p f with the object pointer.
///
/// @param f    Function to pass the pointer of the constructed object.
/// @param h    Target distributed object handle.
/// @param args Arguments to pass to the constructor of @p T.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T, typename Function, typename... Args>
void async_construct(Function&& f,
                     rmi_handle::const_reference const& h,
                     all_locations_t,
                     Args&&... args)
{
  using namespace runtime;

  auto& ctx = this_context::get();

  {
    STAPL_RUNTIME_PROFILE("async_construct(handle,all_locations)",
                          (primitive_traits::non_blocking |
                           primitive_traits::p2m          |
                           primitive_traits::comm));
    construct_all(std::forward<Function>(f),
                  h.get_gang_id(),
                  find_target_gang_md(ctx, h),
                  h.get_epoch(),
                  call_constructor<T>(std::forward<Args>(args)...));
  }

  scheduling_point(ctx);
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates an object of type @p T in a new gang over the range @p r of
///        the locations of the gang of @p h.
///
/// @warning @p r may only contain locations of the gang of the object
///          associated with the @ref rmi_handle::const_reference.
///
/// @param h    Target distributed object handle.
/// @param r    Location id range of the gang of @p h that the new gang will be
///             created in.
/// @param args Arguments to pass to the constructor of @p T.
///
/// @return A @ref future object with an @ref rmi_handle::reference to the
///         constructed object.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T, typename U, typename... Args>
future<rmi_handle::reference> construct(rmi_handle::const_reference const& h,
                                        location_range_wrapper<U> const& r,
                                        Args&&... args)
{
  using namespace runtime;

  auto& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(!r.has_invalid(h.get_num_locations()),
                           "Range contains invalid location id.");

  promise<rmi_handle::reference> p;
  auto f = p.get_future();
  {
    STAPL_RUNTIME_PROFILE("construct(handle,range)",
                          (primitive_traits::non_blocking |
                           primitive_traits::p2m          |
                           primitive_traits::comm));
    construct_range(call_promise_set_value{std::move(p)},
                    h.get_gang_id(),
                    find_target_gang_md(ctx, h),
                    r,
                    h.get_epoch(),
                    call_constructor<T>(std::forward<Args>(args)...));
  }

  scheduling_point(ctx);
  return f;
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates an object of type @p T in a new gang over the range @p r of
///        the locations of the gang of @p h and calls @p f with the object
///        pointer.
///
/// @warning @p r may only contain locations of the gang of the object
///          associated with the @ref rmi_handle::const_reference.
///
/// @param f    Function to pass the pointer of the constructed object.
/// @param h    Target distributed object handle.
/// @param r    Location id range of the gang of @p h that the new gang will be
///             created in.
/// @param args Arguments to pass to the constructor of @p T.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T, typename Function, typename U, typename... Args>
void async_construct(Function&& f,
                     rmi_handle::const_reference const& h,
                     location_range_wrapper<U> const& r,
                     Args&&... args)
{
  using namespace runtime;

  auto& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(!r.has_invalid(h.get_num_locations()),
                           "Range contains invalid location id.");

  {
    STAPL_RUNTIME_PROFILE("async_construct(handle,range)",
                          (primitive_traits::non_blocking |
                           primitive_traits::p2m          |
                           primitive_traits::comm));
    construct_range(std::forward<Function>(f),
                    h.get_gang_id(),
                    find_target_gang_md(ctx, h),
                    r,
                    h.get_epoch(),
                    call_constructor<T>(std::forward<Args>(args)...));
  }

  scheduling_point(ctx);
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates an object of type @p T in a new gang over the level @p l of
///        the hierarchy.
///
/// @param l    Target hierarchy level.
/// @param args Arguments to pass to the constructor of @p T.
///
/// @return A @ref future object with an @ref rmi_handle::reference to the
///         constructed object.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T, typename... Args>
future<rmi_handle::reference> construct(level const& l,
                                        Args&&... args)
{
  using namespace runtime;

  auto& ctx = this_context::get();

  if (l.get_level()==level::invalid)
    STAPL_RUNTIME_ERROR("construct(level): Invalid level requested.");

  promise<rmi_handle::reference> p;
  auto f = p.get_future();
  {
    STAPL_RUNTIME_PROFILE("construct(level)", (primitive_traits::non_blocking |
                                               primitive_traits::p2m          |
                                               primitive_traits::comm));

    call_promise_set_value callback{std::move(p)};
    auto cons_f = call_constructor<T>(std::forward<Args>(args)...);

    if (l.get_level()==level::current) {
      if (l.is_restricted()) {
        // all locations of the current level that are on shared memory
        construct_neighbors(std::move(callback),
                            ctx.get_gang_md(),
                            ctx.get_epoch(),
                            std::move(cons_f));
      }
      else {
        // all locations of the current level
        STAPL_RUNTIME_ERROR("Unrestricted current_level not supported yet.");
      }
    }
    else if (l.get_level()==level::max) {
      if (l.is_restricted()) {
        // maximum level, 1 location
        construct_single(std::move(callback),
                         ctx.get_gang_id(),
                         &(ctx.get_gang_md()),
                         ctx.get_location_id(),
                         ctx.get_epoch(),
                         std::move(cons_f));
      }
      else {
        // maximum level, all locations
        STAPL_RUNTIME_ERROR("Unrestricted lowest level not supported yet.");
      }
    }
    else {
      // lower level
      if (l.is_restricted()) {
        construct_neighbors(std::move(callback),
                            ctx.get_gang_md(),
                            ctx.get_epoch(),
                            std::move(cons_f));
      }
      else {
        STAPL_RUNTIME_ERROR("Unrestricted lower level not supported yet.");
      }
    }
  }

  scheduling_point(ctx);
  return f;
}

} // namespace stapl

#endif
