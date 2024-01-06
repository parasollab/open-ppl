/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RUNTIME_HPP
#define STAPL_RUNTIME_RUNTIME_HPP

#include "config.hpp"
#include "new.hpp"
#include "serialization.hpp"
#include "affinity.hpp"
#include "exit_code.hpp"
#include "gang.hpp"
#include "main.hpp"
#include "runtime_fwd.hpp"
#include "rmi_handle.hpp"
#include "tags.hpp"
#include "this_context.hpp"
#include "yield.hpp"
#include "utility/option.hpp"
#include <functional>
#include <utility>
#include <vector>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Initializes the STAPL Runtime System.
///
/// @warning This is an SPMD function.
///
/// @param opts Options to pass for initialization.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
void initialize(option opts = option{});


//////////////////////////////////////////////////////////////////////
/// @brief Initializes the STAPL Runtime System.
///
/// @warning This is an SPMD function.
///
/// @param argc Number of arguments from @c main().
/// @param argv Argument vector from @c main().
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
inline void initialize(int& argc, char**& argv)
{ initialize(option(argc, argv)); }


//////////////////////////////////////////////////////////////////////
/// @brief Finalizes the STAPL Runtime System.
///
/// @warning This is an SPMD function.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
void finalize(void);


//////////////////////////////////////////////////////////////////////
/// @brief Returns @c true if the STAPL Runtime System is initialized.
///
/// @warning This is an SPMD function.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
bool is_initialized(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Returns the widths of all hierarchy levels.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
std::vector<unsigned int> const& get_hierarchy_widths(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Returns the available parallelism levels.
///
/// This is based on the environment variable @c STAPL_PROC_HIERARCHY that
/// accepts a comma separated value list for the shared memory hierarchy.
///
/// Each time @ref execute() is called, one or more levels are consumed.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
unsigned int get_available_levels(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Executes the given function on a new created environment.
///
/// This function will consume @p n levels of the machine hierarchy.
///
/// @param f Function to be executed.
/// @param n Parallelism levels that will be consumed.
///
/// @see get_available_levels()
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
void execute(std::function<void(void)> f, unsigned int n = 1);


//////////////////////////////////////////////////////////////////////
/// @brief Returns the current process id.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
process_id get_process_id(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Returns the number of processes.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
process_id get_num_processes(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Returns the current location id.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
unsigned int get_location_id(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Returns the number of locations in the current gang.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
unsigned int get_num_locations(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Returns the current location information consisting of the location
///        id and the number of locations in the gang.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
std::pair<unsigned int, unsigned int> get_location_info(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Returns the affinity of the current processing element.
///
/// @related affinity_tag
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
affinity_tag get_affinity(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Returns if the two handles are defined over identical, similar or
///        completely different gangs.
///
/// @return @c 0 if the gangs of the two objects are identical,
///         @c 1 if the gangs are similar (they exist on the same processing
///         elements) or
///         @c -1 if the gangs are completely different or information is not
///         enough.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
int compare_gangs(rmi_handle::const_reference const& x,
                  rmi_handle::const_reference const& y) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief A synchronization point for all locations in a gang.
///
/// This function does not require communication, but it will increase the
/// epoch.
///
/// @warning This is an SPMD function.
///
/// @ingroup ARMISyncs
//////////////////////////////////////////////////////////////////////
void rmi_synchronize(void);


//////////////////////////////////////////////////////////////////////
/// @brief Barrier for all locations in a gang.
///
/// @warning This is an SPMD function.
///
/// @ingroup ARMISyncs
//////////////////////////////////////////////////////////////////////
void rmi_barrier(void);


//////////////////////////////////////////////////////////////////////
/// @brief Ensures that all outstanding RMI requests have been completed.
///
/// This function also is an implicit barrier.
///
/// @warning This is an SPMD function.
///
/// @ingroup ARMISyncs
//////////////////////////////////////////////////////////////////////
void rmi_fence(void);


//////////////////////////////////////////////////////////////////////
/// @brief Causes the calling location to check for and process all available
///        requests. If none are available it returns immediately.
///
/// The main purpose of @c rmi_poll() is to improve timeliness of request
/// processing for a location that does not perform much communication, in
/// support of a location that does.
///
/// @warning User code should never call this function.
///
/// @ingroup ARMI
///
/// @todo Remove from the public interface.
//////////////////////////////////////////////////////////////////////
void rmi_poll(void);


//////////////////////////////////////////////////////////////////////
/// @brief Flush all remaining aggregated RMI requests on the calling location.
///
/// This is useful upon completion of a bulk communication phase of asynchronous
/// requests to ensure the final few requests, which will be fewer than the
/// current aggregation setting, to each location are in transit.
///
/// @ingroup ARMIAggregation
//////////////////////////////////////////////////////////////////////
void rmi_flush(void);


//////////////////////////////////////////////////////////////////////
/// @brief Override the current aggregation settings.
///
/// All asynchronous requests can be internally buffered and issued in groups to
/// reduce network congestion caused by many small requests. Requests are
/// aggregated per destination location, meaning a setting of 5 tries to issue
/// requests in groups of 5 to a specific destination location.
///
/// By default, aggregation is the maximum possible (as determined by internal
/// buffers). If possible, user-defined aggregation will be satisfied, as
/// constrained by the size of internal buffers. For example, it may not be
/// possible to aggregate several large requests given small internal buffers.
/// Consult your specific implementation for ways of increasing internal
/// buffers.
///
/// @param agg Requested number of aggregated requests per internal buffer.
///
/// @return Aggregation setting obtained, which may be less than the requested
///         if the internal buffers are not large enough.
///
/// @ingroup ARMIAggregation
//////////////////////////////////////////////////////////////////////
unsigned int set_aggregation(const unsigned int agg) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Obtain the current aggregation setting.
///
/// @ingroup ARMIAggregation
//////////////////////////////////////////////////////////////////////
unsigned int get_aggregation(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Sets the default RMI buffer size. It will reset aggregation to
///        maximum.
///
/// @warning This is a collective function. The user has to make sure that there
///          are no pending requests. Calling this function in a scope different
///          than that of the application entry point is undefined behavior.
///
/// @ingroup ARMIAggregation
//////////////////////////////////////////////////////////////////////
void set_rmi_buffer_size(const std::size_t);


//////////////////////////////////////////////////////////////////////
/// @brief STAPL Runtime System implementation classes and functions.
//////////////////////////////////////////////////////////////////////
namespace runtime
{ }

} // namespace stapl

#include "context.hpp"
#include "exception.hpp"
#include "instrumentation.hpp"
#include "primitive_traits.hpp"
#include "executor/executor_base.hpp"
#include "executor/gang_executor.hpp"
#include <memory>
#include <type_traits>

namespace stapl {

/////////////////////////////////////////////////////////////////////
/// @brief Sets the scheduler of the @ref gang_executor of the current execution
///        context.
///
/// @warning If the executor has entries or is bound to a parent executor, then
///          replacing the scheduler will result in an error.
///
/// @ingroup executors
//////////////////////////////////////////////////////////////////////
template<typename Scheduler>
void set_executor_scheduler(Scheduler&& scheduler)
{
  using namespace runtime;

  typedef typename std::decay<Scheduler>::type scheduler_type;
  typedef gang_executor<scheduler_type>        executor_type;

  location_md& l = this_context::get().get_location_md();
  l.set_executor(std::unique_ptr<executor_base>{
                   new executor_type{l, std::forward<Scheduler>(scheduler)}});
}


/////////////////////////////////////////////////////////////////////
/// @brief Returns the @ref gang_executor of the current execution context.
///
/// @ingroup executors
//////////////////////////////////////////////////////////////////////
executor_base& get_executor(void);


/////////////////////////////////////////////////////////////////////
/// @brief Returns the default sliding window size for executors.
///
/// @ingroup executors
//////////////////////////////////////////////////////////////////////
std::size_t get_default_executor_window_size(void) noexcept;


/////////////////////////////////////////////////////////////////////
/// @brief Returns the default retire chunk size for executors.
///
/// @ingroup executors
//////////////////////////////////////////////////////////////////////
std::size_t get_default_executor_retire_chunk(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Causes the calling location to block until the given predicate
///        returns @c true.
///
/// While the predicate returns @c false, requests may be executed.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
void block_until(Predicate&& pred)
{
  using namespace runtime;
  STAPL_RUNTIME_PROFILE("block_until()", (primitive_traits::blocking |
                                          primitive_traits::yield));
  yield_until(std::forward<Predicate>(pred));
}

} // namespace stapl

// one-sided RMIs
#include "rmi/async_rmi.hpp"
#include "rmi/executor_rmi.hpp"
#include "rmi/opaque_rmi.hpp"
#include "rmi/reduce_rmi.hpp"
#include "rmi/sync_rmi.hpp"
#include "rmi/try_rmi.hpp"
// collective RMIs
#include "rmi/allgather_rmi.hpp"
#include "rmi/allreduce_rmi.hpp"
#include "rmi/alltoall_rmi.hpp"
#include "rmi/broadcast_rmi.hpp"
// other primitives
#include "non_rmi/abort.hpp"
#include "non_rmi/construct.hpp"
#include "non_rmi/external.hpp"
#include "non_rmi/p_object_delete.hpp"
#include "non_rmi/restore.hpp"
// utilities
#include "bind_rmi.hpp"
#include "immutable_range.hpp"
#include "immutable_ref.hpp"
#include "immutable_shared.hpp"
#include "lazy_ref.hpp"
#include "pointer.hpp"
#include "promise.hpp"
#include "range.hpp"

#endif
