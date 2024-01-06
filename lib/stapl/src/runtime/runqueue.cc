/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/runqueue.hpp>
#include <stapl/runtime/exception.hpp>
#include <stapl/runtime/instrumentation.hpp>
#include <stapl/runtime/location_range.hpp>
#include <stapl/runtime/tags.hpp>
#include <stapl/runtime/communicator/collective.hpp>
#include <stapl/runtime/communicator/communicator.hpp>
#include <stapl/runtime/concurrency/concurrency.hpp>
#include <stapl/runtime/concurrency/mutex.hpp>
#include <stapl/runtime/concurrency/thread_local_storage.hpp>
#include <stapl/runtime/request/rpc_request.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <tuple>
#include <type_traits>
#include <utility>
#include <boost/range/adaptor/filtered.hpp>

#if 1
// this is the only implementation for the time being
# include "runqueue/impl.cc"
#endif


/// Backoff initial wait in milliseconds.
#ifndef STAPL_RUNTIME_BACKOFF_INIT_WAIT
# define STAPL_RUNTIME_BACKOFF_INIT_WAIT 10
#endif


namespace stapl {

namespace runtime {

static void bcast_unordered(gang_md&, message_ptr,
                            const process_id = invalid_process_id);


// --------------------------------------------------------------------
// Shared Runqueue
// --------------------------------------------------------------------

/// Deferred requests per gang id.
static std::unordered_map<gang_md::id, message_slist> deferred_requests;

/// Deferred requests mutex.
static std::mutex deferred_requests_mtx;


//////////////////////////////////////////////////////////////////////
/// @brief Adds a message that was deferred because the gang metadata was not
///        available.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
static void deferred_process(gang_md& g, message_ptr m)
{
  switch (m->type()) {
    case header::LOCATION_RPC: {
      auto const& h  = m->get_extended_header<header::location_rpc>();
      const auto lid = h.get_destination().get_location_id();
      if (lid!=invalid_location_id)
        g.get_runqueue().get_impl().enqueue(lid, std::move(m));
      else
        g.get_runqueue().get_impl().enqueue_any(std::move(m));
    } break;

    case header::BCAST_LOCATION_RPC: {
      auto const& h = m->get_extended_header<header::bcast_location_rpc>();
      bcast_unordered(g, std::move(m), h.get_process_id());
    } break;

    case header::RMI: {
      auto const& h  = m->get_extended_header<header::request>();
      const auto lid = h.get_destination().get_location_id();
      g.get_runqueue().get_impl().enqueue(lid, std::move(m));
    } break;

    case header::BCAST_RMI:
      g.get_runqueue().get_impl().enqueue_all(true, std::move(m));
      break;

    case header::FWD_UNORDERED_BCAST_RMI:
      bcast_unordered(g, std::move(m));
      break;

    case header::UNORDERED_BCAST_RMI: {
      auto const& h = m->get_extended_header<header::bcast_request>();
      bcast_unordered(g, std::move(m), h.get_process_id());
    } break;

    default:
      STAPL_RUNTIME_ERROR("Unrecognized header");
      break;
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Defers the execution of the requests in @p m.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
static void deferred_enqueue(const gang_md::id gid, message_ptr m)
{
  std::lock_guard<std::mutex> lock{deferred_requests_mtx};
  auto* const g = gang_md_registry::try_get(gid);
  if (g)
    deferred_process(*g, std::move(m));
  else
    deferred_requests[gid].push_back(std::move(m));
}


// Constructs a new shared_runqueue for a single location
shared_runqueue::shared_runqueue(void)
: m_impl(new impl)
{ }


// Creates a new shared_runqueue for location ids [first, first + n)
shared_runqueue::shared_runqueue(const location_id first, const size_type n)
: m_impl(new impl{first, n})
{ STAPL_RUNTIME_ASSERT(n>0); }


// Constructs a new shared_runqueue based on the location id range
shared_runqueue::shared_runqueue(shared_runqueue::location_id_range const& r)
: m_impl(new impl{location_range(r)})
{ }


// Constructs a new shared_runqueue based on the metadata the given one
shared_runqueue::shared_runqueue(shared_runqueue const& other)
: m_impl(new impl{*other.m_impl})
{ }


// Creates a new shared_runqueue from the gang_description object
shared_runqueue::shared_runqueue(gang_description const& gd,
                                 const process_id pid)
{
  if (gd.is_on_shmem()) {
    m_impl.reset(new impl{0, gd.get_num_locations()});
  }
  else if (gd.is_on_distmem()) {
    m_impl.reset(new impl);
  }
  else {
    m_impl.reset(
      new impl{location_range(gd.get_location_ids(pid))});
  }
}


shared_runqueue::~shared_runqueue(void) = default;


// Acquires an already created shared_runqueue::impl or creates a new one
std::unique_lock<std::mutex> shared_runqueue::acquire(gang_md& g)
{
  std::unique_lock<std::mutex> lock{deferred_requests_mtx};

  auto it = deferred_requests.find(g.get_id());
  if (it==deferred_requests.end())
    return lock;

  auto& ml = it->second;
  do {
    deferred_process(g, ml.pop_front());
  } while (!ml.empty());

  deferred_requests.erase(it);

  return lock;
}


// Returns the size of the shared_runqueue
shared_runqueue::size_type shared_runqueue::size(void) const noexcept
{ return m_impl->size(); }


// --------------------------------------------------------------------
// Runqueue
// --------------------------------------------------------------------

//////////////////////////////////////////////////////////////////////
/// @brief Communicator polling policy.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
enum polling_policy_t
{
  /// No thread polls.
  NO_THREAD = 0x0,
  /// Single threaded execution.
  SINGLE_THREAD,
  /// First thread to arrive polls.
  FIRST_THREAD,
  /// Master thread polls.
  MASTER_THREAD
};


/// Positive number if the process is overloaded.
static std::atomic<int> overloaded{0};

/// Polling policy.
static polling_policy_t polling_policy = FIRST_THREAD;

/// Maximum backoff wait in milliseconds.
static std::chrono::milliseconds backoff_max_wait;

/// Initial backoff wait in milliseconds.
static std::chrono::milliseconds backoff_init_wait;


//////////////////////////////////////////////////////////////////////
/// @brief Backoff mechanism metadata.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
template<typename Clock = std::chrono::high_resolution_clock>
class backoff_metadata
{
private:
  using duration   = std::chrono::milliseconds;
  using time_point = typename Clock::time_point;

  duration           m_wait;
  time_point         m_timestamp;
  mutable time_point m_cached_now;

public:
  void operator()(void) noexcept
  {
    using rand_dist = std::uniform_int_distribution<std::uint_least64_t>;

    if (backoff_max_wait == duration::zero())
      return;

    if (m_wait == duration::zero()) {
      // backoff was not initialized
      m_wait      = backoff_init_wait;
      m_timestamp = Clock::now();
    }
    else {
      m_wait      = std::min(backoff_max_wait, (2 * m_wait));
      m_timestamp = m_cached_now;
    }
  }

  bool expired(void) const noexcept
  {
    if (m_wait == duration::zero())
      return true;
    m_cached_now = Clock::now();
    return ((m_cached_now - m_timestamp) >= m_wait);
  }

  void reset(void) noexcept
  { m_wait = duration::zero(); }
};


/// Thread-local backoff metadata.
static STAPL_RUNTIME_THREAD_LOCAL(backoff_metadata<>, backoff)


//////////////////////////////////////////////////////////////////////
/// @brief Broadcasts @p m to all children processes, except process @p id.
///
/// If @p m is already broadcast, it adds it locally.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
static void bcast_unordered(gang_md& g, message_ptr m, const process_id id)
{
  auto const& topo = g.get_topology();
  auto const& pids = topo.children();

  if (m->is_forwarded() ||
      pids.empty()      ||
      (pids.size()==1 && pids[0]==id)) {
    // no processes to forward to; enqueue it locally
    if (m->type()==header::FWD_UNORDERED_BCAST_RMI)
      m->retarget(header::UNORDERED_BCAST_RMI);
    g.get_runqueue().get_impl().enqueue_all(false, std::move(m));
  }
  else {
    // forward m; when forwarding is done, the communicator will return m to the
    // current process, which will schedule it for execution
    m->mark_forwarded();

    if (id!=invalid_process_id && id!=topo.root_id()) {
      // do not send to id
      communicator::forward_and_store(pids, id, std::move(m));
    }
    else {
      communicator::forward_and_store(pids, std::move(m));
    }
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Initializes a broadcast of @p m to all the locations of @p g.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
static void bcast_unordered_init(gang_md& g, message_ptr m)
{
  if (g.get_description().is_on_shmem() || m->is_forwarded()) {
    if (m->type()==header::FWD_UNORDERED_BCAST_RMI)
      m->retarget(header::UNORDERED_BCAST_RMI);
    g.get_runqueue().get_impl().enqueue_all(false, std::move(m));
  }
  else {
    // give it to the communicator and when done it will be added locally
    m->mark_forwarded();
    communicator::forward_and_store(g.get_topology(), std::move(m));
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Adds @p m to its destination runqueue.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
static void runqueue_add(message_ptr m)
{
  switch (m->type()) {

    case header::RPC: {
      // execute in place
      rpc_executor()(std::move(m));
    } break;

    case header::FWD_LOCATION_RPC: {
      // gang metadata is here, forward it to the correct destination if needed
      auto const& h  = m->fix_payload_offset<header::location_rpc>();
      m->retarget(header::LOCATION_RPC);
      const auto gid = h.get_destination().get_gang_id();
      auto& g        = gang_md_registry::get(gid);
      const auto lid = h.get_destination().get_location_id();
      if (lid!=invalid_location_id) {
        const auto dst_pid = g.get_process_id(lid);
        if (dst_pid==runqueue::get_process_id())
          g.get_runqueue().get_impl().enqueue(lid, std::move(m));
        else
          communicator::send(dst_pid, std::move(m));
      }
      else {
        g.get_runqueue().get_impl().enqueue_any(std::move(m));
      }
    } break;

    case header::LOCATION_RPC: {
      // attempt to queue, if the gang metadata is not present, defer
      auto const& h  = m->fix_payload_offset<header::location_rpc>();
      const auto gid = h.get_destination().get_gang_id();
      auto* const g  = gang_md_registry::try_get(gid);
      if (g) {
        const auto lid = h.get_destination().get_location_id();
        if (lid!=invalid_location_id)
          g->get_runqueue().get_impl().enqueue(lid, std::move(m));
        else
          g->get_runqueue().get_impl().enqueue_any(std::move(m));
      }
      else {
        deferred_enqueue(gid, std::move(m));
      }
    } break;

    case header::BCAST_LOCATION_RPC: {
      // attempt to queue, if the gang metadata is not present, defer
      auto const& h =
        m->fix_payload_offset<header::bcast_location_rpc>();
      const auto gid = h.get_destination_gang_id();
      auto* const g  = gang_md_registry::try_get(gid);
      if (g)
        bcast_unordered(*g, std::move(m), h.get_process_id());
      else
        deferred_enqueue(gid, std::move(m));
    } break;

    case header::FWD_RMI: {
      // gang metadata is here, forward it to the correct destination if needed
      auto const& h      = m->fix_payload_offset<header::request>();
      m->retarget(header::RMI);
      const auto gid     = h.get_destination().get_gang_id();
      auto& g            = gang_md_registry::get(gid);
      const auto lid     = h.get_destination().get_location_id();
      const auto dst_pid = g.get_process_id(lid);
      if (dst_pid==runqueue::get_process_id())
        g.get_runqueue().get_impl().enqueue(lid, std::move(m));
      else
        communicator::send(dst_pid, std::move(m));
    } break;

    case header::RMI: {
      // attempt to queue, if the gang metadata is not present, defer
      auto const& h  = m->fix_payload_offset<header::request>();
      const auto gid = h.get_destination().get_gang_id();
      auto* const g  = gang_md_registry::try_get(gid);
      if (g) {
        const auto lid = h.get_destination().get_location_id();
        g->get_runqueue().get_impl().enqueue(lid, std::move(m));
      }
      else {
        deferred_enqueue(gid, std::move(m));
      }
    } break;

    case header::FWD_BCAST_RMI: {
      // gang metadata is here, broadcast
      auto const& h  = m->fix_payload_offset<header::bcast_request>();
      m->retarget(header::BCAST_RMI);
      const auto gid = h.get_destination_gang_id();
      STAPL_RUNTIME_ASSERT(
        gang_md_registry::id_owner(gid)==runqueue::get_process_id());
      auto& g = gang_md_registry::get(gid);
      runqueue::add_all(g, true, std::move(m));
    } break;

    case header::BCAST_RMI: {
      // attempt to broadcast, if the gang metadata is not present, defer
      auto const& h  = m->fix_payload_offset<header::bcast_request>();
      const auto gid = h.get_destination_gang_id();
      auto* const g  = gang_md_registry::try_get(gid);
      if (g)
        g->get_runqueue().get_impl().enqueue_all(true, std::move(m));
      else
        deferred_enqueue(gid, std::move(m));
    } break;

    case header::FWD_UNORDERED_BCAST_RMI: {
      // gang metadata is here, broadcast
      auto const& h  = m->fix_payload_offset<header::bcast_request>();
      const auto gid = h.get_destination_gang_id();
      if (gang_md_registry::id_owner(gid)==runqueue::get_process_id()) {
        auto& g = gang_md_registry::get(gid);
        bcast_unordered_init(g, std::move(m));
      }
      else {
        auto* const g = gang_md_registry::try_get(gid);
        if (g)
          bcast_unordered(*g, std::move(m));
        else
          deferred_enqueue(gid, std::move(m));
      }
    } break;

    case header::UNORDERED_BCAST_RMI: {
      // attempt to broadcast, if the gang metadata is not present, defer
      auto const& h  = m->fix_payload_offset<header::bcast_request>();
      const auto gid = h.get_destination_gang_id();
      auto* const g  = gang_md_registry::try_get(gid);
      if (g)
        bcast_unordered(*g, std::move(m), h.get_process_id());
      else
        deferred_enqueue(gid, std::move(m));
    } break;

    default:
      STAPL_RUNTIME_ERROR("Unrecognized header");
      break;
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Process incoming messages.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
static void process_incoming(const bool can_block)
{
  auto msg_list = communicator::poll(can_block);
  while (!msg_list.empty()) {
    // add received messages to runqueue
    runqueue_add(msg_list.pop_front());
  }
}


// Initializes the process
void runqueue::initialize(option const& opts)
{
  // backoff mechanism set-up
  const auto backoff_init_wait_ms =
    opts.get<unsigned int>("STAPL_RUNTIME_BACKOFF_INIT_WAIT",
                           STAPL_RUNTIME_BACKOFF_INIT_WAIT);
  if (backoff_init_wait_ms < 1)
    STAPL_RUNTIME_ERROR("STAPL_RUNTIME_BACKOFF_INIT_WAIT has to be a positive "
                        "number.");
  backoff_init_wait = std::chrono::milliseconds{backoff_init_wait_ms};

  // determine polling policy
  const auto policy = opts.get<std::string>("STAPL_RUNTIME_COMM_THREAD",
                                            "first");
  if (policy=="first") {
    // first thread arriving in yield will poll the communicator
    polling_policy = FIRST_THREAD;
  }
  else if (policy=="master") {
    // only the master thread will poll the communicator, backoff disabled
    polling_policy = MASTER_THREAD;
    backoff_max_wait = backoff_init_wait = std::chrono::milliseconds{0};
  }
  else {
    STAPL_RUNTIME_ERROR("STAPL_RUNTIME_COMM_THREAD can only be \"first\" or "
                        "\"master\".");
  }

  // initialize components
  communicator::initialize(opts);

  concurrency::initialize(opts,
                          communicator::get_id(), communicator::size(),
                          communicator::get_num_procs_per_node());
  memory_allocator::initialize(opts);
  message::initialize(opts, communicator::get_num_procs_per_node());
  communicator::start();
#ifdef STAPL_RUNTIME_ENABLE_INSTRUMENTATION
  instrument::initialize(opts);
#endif
  runqueue::impl::initialize(opts);
  gang_md_registry::initialize(communicator::get_id(), communicator::size());

  if (communicator::size()==1) {
    // only one process, disable access to the communication layer
    polling_policy = NO_THREAD;
  }
  else if (concurrency::hardware_concurrency()==1) {
    // only one process, only one master thread
    polling_policy = SINGLE_THREAD;
  }
  else {
    backoff_max_wait = concurrency::hardware_concurrency() * backoff_init_wait;
  }
}


// Tears down the process
void runqueue::finalize(void)
{
  gang_md_registry::finalize();
  {
    std::lock_guard<std::mutex> lock{deferred_requests_mtx};
    if (!deferred_requests.empty())
      STAPL_RUNTIME_ERROR("Deferred requests present during finalization.");
  }
  runqueue::impl::finalize();
#ifdef STAPL_RUNTIME_ENABLE_INSTRUMENTATION
  instrument::finalize();
#endif
  communicator::stop();
  message::finalize();
  memory_allocator::finalize();
  concurrency::finalize();
  communicator::finalize();
}


// Locks the runqueue
runqueue::lock_type::lock_type(void)
{
  communicator::lock();
}


// Unlocks the runqueue
runqueue::lock_type::~lock_type(void)
{
  communicator::unlock();
}


// Returns the current process id
process_id runqueue::get_process_id(void) noexcept
{
  return communicator::get_id();
}


// Returns the current process id
runqueue::size_type runqueue::get_num_processes(void) noexcept
{
  return communicator::size();
}


// Causes normal termination.
void runqueue::exit(int exit_code)
{
  communicator::exit(exit_code);
}


// Notifies if the process is overloaded or not
void runqueue::set_overloaded(const bool over) noexcept
{
  if (over)
    overloaded.fetch_add(1, std::memory_order_relaxed);
  else
    overloaded.fetch_sub(1, std::memory_order_relaxed);
}


// Schedules requests to execute on the given process
void runqueue::add(const process_id dst_pid, message_ptr m)
{
  communicator::send(dst_pid, std::move(m));
}


// Schedules requests to execute on the given processes
void runqueue::add_all(process_id_range r, message_ptr m)
{
  communicator::send_all(std::move(r), std::move(m));
}


// Schedules requests to execute on the given location
void runqueue::add(gang_md& dst_g,
                   const location_id dst_lid,
                   const bool on_shmem,
                   message_ptr m)
{
  STAPL_RUNTIME_ASSERT((m->type()==header::LOCATION_RPC) ||
                       (m->type()==header::RMI));
  if (on_shmem) {
    // location is here; add it locally
    STAPL_RUNTIME_ASSERT(dst_g.get_process_id(dst_lid)==get_process_id());
    dst_g.get_runqueue().get_impl().enqueue(dst_lid, std::move(m));
  }
  else {
    // location not here; forward it
    const process_id dst_pid = dst_g.get_process_id(dst_lid);
    STAPL_RUNTIME_ASSERT(dst_pid!=get_process_id());
    communicator::send(dst_pid, std::move(m));
  }
}


// Schedules requests to execute on a range of locally managed locations
void runqueue::add_managed(gang_md& g,
                           location_id_range r,
                           const bool ordered,
                           message_ptr m)
{
  STAPL_RUNTIME_ASSERT(m->type()==header::BCAST_LOCATION_RPC);
  if (r.empty())
    g.get_runqueue().get_impl().enqueue_all(ordered, std::move(m));
  else
    g.get_runqueue().get_impl().enqueue_range(r, ordered, std::move(m));
}


// Schedules requests to execute on all locations of the gang
void runqueue::add_all(gang_md& g, const bool ordered, message_ptr m)
{
  if (ordered) {
    STAPL_RUNTIME_ASSERT(m->type()==header::BCAST_RMI);
    auto const& gd = g.get_description();
    if (!gd.is_on_shmem()) {
      // create a clone of the message for the communicator and send it to all
      // processes but the current one
      boost::intrusive_ptr<gang_md> p = &g;
      communicator::send_all(communicator::process_id_range{
                               boost::adaptors::filter(
                                 gd.get_processes(),
                                 [p](const process_id pid)
                                 { return (pid!=p->get_topology().get_id()); }),
                               (gd.get_num_processes() - 1)},
                             m->clone());
    }
    // enqueue in shared memory
    g.get_runqueue().get_impl().enqueue_all(true, std::move(m));
  }
  else {
    STAPL_RUNTIME_ASSERT((m->type()==header::BCAST_LOCATION_RPC) ||
                         (m->type()==header::UNORDERED_BCAST_RMI));
    bcast_unordered_init(g, std::move(m));
  }
}


// Forwards m to the owner of dst_gid
void runqueue::forward(const gang_id dst_gid, message_ptr m)
{
  STAPL_RUNTIME_ASSERT((m->type()==header::FWD_LOCATION_RPC) ||
                       (m->type()==header::FWD_RMI)          ||
                       (m->type()==header::FWD_BCAST_RMI)    ||
                       (m->type()==header::FWD_UNORDERED_BCAST_RMI));
  const auto owner = gang_md_registry::id_owner(dst_gid);
  STAPL_RUNTIME_ASSERT(owner!=get_process_id());
  communicator::send(owner, std::move(m));
}


// Yields to the runqueue
void runqueue::yield(const unsigned int intensity)
{
  STAPL_RUNTIME_ASSERT(intensity!=0);

  auto& stack      = get_thread_stack();
  bool light_yield = (intensity==1);

  switch (polling_policy) {
    case NO_THREAD:
      break;

    case SINGLE_THREAD:
      if (overloaded.load(std::memory_order_relaxed)!=0) {
        // process is overloaded, service as many requests as possible
        light_yield = false;
        stack.yield();
      }
      process_incoming(false);
      break;

    case FIRST_THREAD:
      if (overloaded.load(std::memory_order_relaxed)!=0) {
        // process is overloaded, service as many requests as possible
        light_yield = false;
        stack.yield();
        auto& mtx = concurrency::get_mutex();
        using mutex_type = typename std::decay<decltype(mtx)>::type;
        std::unique_lock<mutex_type> lock{mtx, std::try_to_lock};
        if (lock.owns_lock())
          process_incoming(false);
      }
      else {
        auto& b = backoff.get();
        if (b.expired()) {
          auto& mtx = concurrency::get_mutex();
          using mutex_type = typename std::decay<decltype(mtx)>::type;
          std::unique_lock<mutex_type> lock{mtx, std::try_to_lock};
          if (lock.owns_lock()) {
            process_incoming(false);
            lock.unlock();
            b.reset();
          }
          else {
            b();
          }
        }
      }
      break;

    case MASTER_THREAD:
      if (std::this_thread::get_id()==concurrency::get_master_thread_id()) {
        if (overloaded.load(std::memory_order_relaxed)!=0) {
          // process is overloaded, service as many requests as possible
          light_yield = false;
          stack.yield();
        }
        process_incoming(false);
      }
      break;

    default:
      STAPL_RUNTIME_ERROR("Unknown communicator policy.");
  }

  stack.yield(light_yield);
}


// Informs that the caller is blocked
void runqueue::wait(std::function<bool(void)> const& pred)
{
  auto& stack = get_thread_stack();

  switch (polling_policy) {
    case NO_THREAD:
      do {
        stack.yield();
      } while (!bool(pred()));
      break;

    case SINGLE_THREAD:
      do {
        bool can_block = true;
        switch (stack.yield()) {
          case YIELDED:
          case IDLE_CANNOT_BLOCK:
            can_block = false;
            break;
          case IDLE_CAN_BLOCK:
            break;
          default:
            STAPL_RUNTIME_ERROR("Incorrect runqueue status.");
            break;
        }

        // check if a request satisfied the condition
        if (bool(pred()))
          return;

        process_incoming(can_block);

      } while (!bool(pred()));
      break;

    case FIRST_THREAD: {
      auto& mtx = concurrency::get_mutex();
      auto& b   = backoff.get();
      do {
        stack.yield();

        // check if a request satisfied the condition
        if (bool(pred()))
          return;

        // do not access communication layer if backoff did not expire
        if (!b.expired())
          continue;

        {
          using mutex_type = typename std::decay<decltype(mtx)>::type;
          std::unique_lock<mutex_type> lock{mtx, std::try_to_lock};
          if (lock.owns_lock()) {
            // requires correct detection of all blocked threads; see @todo
            process_incoming(false);
            lock.unlock();
            b.reset();
          }
          else {
            b();
          }
        }
      } while (!bool(pred()));
    } break;

    case MASTER_THREAD:
      if (std::this_thread::get_id()==concurrency::get_master_thread_id()) {
        do {
          stack.yield();

          // check if a request satisfied the condition
          if (bool(pred()))
            return;

          // requires correct detection of all blocked threads; see @todo
          process_incoming(false);

        } while (!bool(pred()));
      }
      else {
        do {
          stack.yield();
        } while (!bool(pred()));
      }
      break;

    default:
      STAPL_RUNTIME_ERROR("Unknown communicator policy.");
  }
}


// Returns the required size for the internal implementation
std::size_t runqueue::required_size(void) noexcept
{
  return sizeof(runqueue::impl);
}


// Returns the required alignment for the internal implementation
std::size_t runqueue::required_alignment(void) noexcept
{
  return std::alignment_of<runqueue::impl>::value;
}


// Constructs a new runqueue
runqueue::runqueue(void* p, location_md& l)
: m_impl(runqueue::impl::create(p, l))
{ }


// Destroys the runqueue
runqueue::~runqueue(void)
{
  runqueue::impl::destroy(m_impl);
}


// Returns the yield intensity
unsigned int runqueue::yield_intensity(void) const noexcept
{
  return m_impl->yield_intensity();
}


// Notifies that we entered a fence
void runqueue::fence_enter(void)
{
  m_impl->fence_enter();
}


// Notifies that we exited a fence
void runqueue::fence_exit(const epoch_type e)
{
  m_impl->fence_exit(e);
}


// Make runnable anything that was blocked because of epoch
void runqueue::advance_epoch(const epoch_type e)
{
  m_impl->advance_epoch(e);
}

void runqueue::undefer_requests(void)
{
  m_impl->undefer_requests();
}

void runqueue::defer_requests(void)
{
  m_impl->defer_requests();
}

bool runqueue::try_defer_requests(void)
{
  return m_impl->try_defer_requests();
}

} // namespace runtime

} // namespace stapl
