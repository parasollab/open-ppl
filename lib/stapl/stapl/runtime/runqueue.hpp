/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_RUNQUEUE_HPP
#define STAPL_RUNTIME_RUNQUEUE_HPP

#include "config.hpp"
#include "full_location.hpp"
#include "gang_description.hpp"
#include "message.hpp"
#include "utility/any_range.hpp"
#include "utility/logical_clock.hpp"
#include "utility/option.hpp"
#include <functional>
#include <memory>
#include <mutex>

namespace stapl {

namespace runtime {

class gang_md;
class location_md;


//////////////////////////////////////////////////////////////////////
/// @brief Queue for scheduling requests.
///
/// The static functions are the entry point for all remote calls. They are
/// well-known function pointers that they can be used to schedule requests in
/// any process.
///
/// @ingroup runtimeMetadata
///
/// @todo All the static functions should be in a different class named with a
///       distinct name that signifies that it is an RPC-like entry point.
/// @todo The @ref communicator should be folded in the @ref runqueue. Then we
///       can interface with other systems that offer RPCs to specific threads.
/// @todo Abort operations should be folded in the @ref runqueue.
//////////////////////////////////////////////////////////////////////
class runqueue
{
public:
  using size_type         = process_id;
  using epoch_type        = logical_clock::time_type;
  using location_id_range = any_range<location_id>;
  using process_id_range  = any_range<process_id>;

  ////////////////////////////////////////////////////////////////////
  /// @brief Locks the runqueue at construction, unlocks at destruction.
  ////////////////////////////////////////////////////////////////////
  struct lock_type
  {
    lock_type(void);
    ~lock_type(void);
  };

  /// Status of the yield call.
  enum yield_status
  {
    /// Caller has yielded.
    YIELDED,
    /// Caller has not yielded and cannot block.
    IDLE_CANNOT_BLOCK,
    /// Caller has not yielded and can block.
    IDLE_CAN_BLOCK
  };

  class impl;

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets up the process.
  ///
  /// @param opts @ref option object to initialize the environment with.
  ////////////////////////////////////////////////////////////////////
  static void initialize(option const& opts);

  ////////////////////////////////////////////////////////////////////
  /// @brief Tears-down the process.
  ////////////////////////////////////////////////////////////////////
  static void finalize(void);

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the current process id.
  ////////////////////////////////////////////////////////////////////
  static process_id get_process_id(void) noexcept;

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of processes.
  ////////////////////////////////////////////////////////////////////
  static size_type get_num_processes(void) noexcept;

  ////////////////////////////////////////////////////////////////////
  /// @brief Exits with exit code @p exit_code.
  ///
  /// @todo All processes have to be notified of the exit in order to run the
  ///       exit-registered functions.
  ////////////////////////////////////////////////////////////////////
  static void exit(int exit_code);

  ////////////////////////////////////////////////////////////////////
  /// @brief Receives notification that the process is overloaded.
  ////////////////////////////////////////////////////////////////////
  static void set_overloaded(const bool) noexcept;

  //////////////////////////////////////////////////////////////////////
  /// @brief Schedules the requests in @p m to execute in process @p pid.
  ///
  /// @warning The current process id cannot be @p pid.
  //////////////////////////////////////////////////////////////////////
  static void add(const process_id pid, message_ptr m);

  //////////////////////////////////////////////////////////////////////
  /// @brief Schedules the requests in @p m to execute in the processes in range
  ///        @p r.
  ///
  /// @warning The current process id cannot be in the range @p v.
  //////////////////////////////////////////////////////////////////////
  static void add_all(process_id_range r, message_ptr m);

  //////////////////////////////////////////////////////////////////////
  /// @brief Schedules the requests in @p m to execute in the location @p lid of
  ///        gang @p g.
  ///
  /// @p on_shmem defines if the requests' source and destination are on shared
  /// memory.
  //////////////////////////////////////////////////////////////////////
  static void add(gang_md& g,
                  const location_id lid,
                  const bool on_shmem,
                  message_ptr m);

  //////////////////////////////////////////////////////////////////////
  /// @brief Schedules the requests in @p m to execute in the range @p r of
  ///        locally managed locations of gang @p g.
  ///
  /// If @p r is empty, then @p m is scheduled on all locally managed locations.
  ///
  /// @warning All the locations in @p r have to be in the current process.
  //////////////////////////////////////////////////////////////////////
  static void add_managed(gang_md& g,
                          location_id_range r,
                          const bool ordered,
                          message_ptr m);

  //////////////////////////////////////////////////////////////////////
  /// @brief Schedules the requests in @p m to execute in all locations of the
  ///        gang @p g.
  ///
  /// @todo Ordered broadcast in the communication layer is required to make
  ///       this call more efficient.
  //////////////////////////////////////////////////////////////////////
  static void add_all(gang_md& g, const bool ordered, message_ptr m);

  //////////////////////////////////////////////////////////////////////
  /// @brief Forwards the requests to the owner of the gang with id @p gid.
  ///
  /// @warning This function always forwards to the gang id owner, even if the
  ///          metadata is locally available. This may degrade performance.
  //////////////////////////////////////////////////////////////////////
  static void forward(const gang_id gid, message_ptr m);

  //////////////////////////////////////////////////////////////////////
  /// @brief Yields to the @ref runqueue and executes all requests that are
  ///        pending.
  ///
  /// @param intensity A positive number indicating how intense yielding should
  ///                  be.
  //////////////////////////////////////////////////////////////////////
  static void yield(const unsigned int intensity = 2);

  //////////////////////////////////////////////////////////////////////
  /// @brief Yields to the @ref runqueue, informing it that the caller is
  ///        blocked until @p pred returns @c true.
  ///
  /// A backoff algorithm guarantees that contention will be minimized when
  /// multiple threads call this function. When contention is detected, then the
  /// caller will avoid @ref concurrency::hardware_concurrency() times to
  /// declare itself as leader, therefore it will avoid any calls to the
  /// communication layer.
  ///
  /// @param pred Predicate to satisfy before exiting the function.
  ///
  /// @todo In multi-threaded mode, correct detection of all threads that are
  ///       blocked is required for the call to be blocking.
  //////////////////////////////////////////////////////////////////////
  static void wait(std::function<bool(void)> const& pred);

  static std::size_t required_size(void) noexcept;

  static std::size_t required_alignment(void) noexcept;

private:
  impl* m_impl;

public:
  runqueue(void*, location_md&);

  runqueue(runqueue const&) = delete;
  runqueue& operator=(runqueue const&) = delete;

  ~runqueue(void);

  impl& get_impl(void) noexcept
  { return *m_impl; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a positive integer with how intense yielding will be, or
  ///        @c 0 if yielding should be avoided.
  //////////////////////////////////////////////////////////////////////
  unsigned int yield_intensity(void) const noexcept;

  //////////////////////////////////////////////////////////////////////
  /// @brief Notifies that the location entered a fence.
  //////////////////////////////////////////////////////////////////////
  void fence_enter(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Notifies that the location exited a fence.
  //////////////////////////////////////////////////////////////////////
  void fence_exit(const epoch_type);

  //////////////////////////////////////////////////////////////////////
  /// @brief Make runnable anything that was blocked because of future epoch.
  //////////////////////////////////////////////////////////////////////
  void advance_epoch(const epoch_type);

  void undefer_requests(void);
  void defer_requests(void);
  bool try_defer_requests(void);
};


//////////////////////////////////////////////////////////////////////
/// @brief Runqueue shared among locations of a gang that are on the same
///        process.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class shared_runqueue
{
public:
  using size_type         = std::size_t;
  using location_id_range = any_range<location_id>;

  class impl;

private:
  std::unique_ptr<impl> m_impl;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a runqueue for a single location.
  //////////////////////////////////////////////////////////////////////
  shared_runqueue(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a runqueue for @p n location ids that start from
  ///        @p first.
  //////////////////////////////////////////////////////////////////////
  explicit shared_runqueue(const location_id first, const size_type n = 1);

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a runqueue based on the metadata of @p other.
  //////////////////////////////////////////////////////////////////////
  shared_runqueue(shared_runqueue const& other);

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a runqueue based on the range @p r.
  //////////////////////////////////////////////////////////////////////
  explicit shared_runqueue(location_id_range const& r);

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a runqueue from @p gd for process @p pid.
  //////////////////////////////////////////////////////////////////////
  shared_runqueue(gang_description const& gd, const process_id pid);

  ~shared_runqueue(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Acquires the runqueue associated with @p g.
  ///
  /// @warning This is a potential contention point as the lock in the deferred
  ///          requests queue can only be held by one thread at a time.
  ///
  /// @return An acquired lock that protects the queue of deferred requests and
  ///         should only be released when gang metadata @p g is registered.
  //////////////////////////////////////////////////////////////////////
  std::unique_lock<std::mutex> acquire(gang_md& g);

  impl& get_impl(void) noexcept
  { return *m_impl; }

  size_type size(void) const noexcept;
};

} // namespace runtime

} // namespace stapl

#endif
