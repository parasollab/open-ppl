/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/runtime.hpp>
#include <stapl/runtime/context.hpp>
#include <stapl/runtime/exception.hpp>
#include <stapl/runtime/instrumentation.hpp>
#include <stapl/runtime/message.hpp>
#include <stapl/runtime/runqueue.hpp>
#include <stapl/runtime/synchronization.hpp>
#include <stapl/runtime/collective/barrier_object.hpp>
#include <stapl/runtime/communicator/communicator.hpp>
#include <stapl/runtime/concurrency/concurrency.hpp>
#include <stapl/runtime/utility/functional.hpp>
#include <algorithm>
#include <functional>
#include <iterator>
#include <limits>
#include <utility>
#ifdef STAPL_USE_PAPI
# include <papi.h>
#endif

#ifdef STAPL_RUNTIME_DISABLE_COMBINING
# warning "Request combining is disabled. Performance may degrade."
#endif

#ifndef STAPL_EXECUTOR_DEFAULT_WINDOW_SIZE
# define STAPL_EXECUTOR_DEFAULT_WINDOW_SIZE std::numeric_limits<std::size_t>::max()
#endif

#ifndef STAPL_EXECUTOR_DEFAULT_RETIRE_CHUNK
# define STAPL_EXECUTOR_DEFAULT_RETIRE_CHUNK 1
#endif

namespace stapl {

namespace runtime {

/// Maximum number of RMI requests to aggregate before sending a message.
static unsigned int aggregation_factor = 0;

/// @c true if the runtime is initialized, otherwise @c false.
static bool initialized = false;

/// Debug level.
static int debug_level = 0;

/// Executor window size
static std::size_t executor_window_size = 0;

/// Executor retire chunk.
static std::size_t executor_retire_chunk = 0;

/// All the available hierarchy level widths.
static std::vector<unsigned int> hierarchy_widths;

/// @c true if the first level of parallelism is consumed
static bool consumed_1st_level = false;

} // namespace runtime


using namespace runtime;


// --------------------------------------------------------------------------
// Process information
// --------------------------------------------------------------------------

// Returns the current process id
process_id get_process_id(void) noexcept
{
  return runqueue::get_process_id();
}


// Returns the number of processes
process_id get_num_processes(void) noexcept
{
  return runqueue::get_num_processes();
}


// --------------------------------------------------------------------------
// Parallelization Primitives
// --------------------------------------------------------------------------

// Returns the location id in the the current gang
unsigned int get_location_id(void) noexcept
{
  const context* const ctx = this_context::try_get();
  return (!ctx ? 0 : ctx->get_location_id());
}


// Returns the number of locations of the current gang
unsigned int get_num_locations(void) noexcept
{
  const context* const ctx = this_context::try_get();
  return (!ctx ? 1 : ctx->get_gang_md().size());
}


// Returns the location id and number of locations in the the current gang
std::pair<unsigned int, unsigned int> get_location_info(void) noexcept
{
  using result_type = std::pair<unsigned int, unsigned int>;
  const context* const ctx = this_context::try_get();
  return (!ctx
            ? result_type{0, 1}
            : result_type{ctx->get_location_id(), ctx->get_gang_md().size()});
}


// Returns the affinity tag of the current processing element
affinity_tag get_affinity(void) noexcept
{
  return concurrency::get_affinity();
}


// Compares the gangs of the two handles
int compare_gangs(rmi_handle::const_reference const& x,
                  rmi_handle::const_reference const& y) noexcept
{
#if 0
  // invalid handles are not tolerated
  if (!x.valid())
    STAPL_RUNTIME_ERROR("First handle is invalid.");
  if (!y.valid())
    STAPL_RUNTIME_ERROR("Second handle is invalid.");
#else
  // For now since not everybody is using gangs correctly
  // 1 Both handles invalid means they are the same gang
  // 2 Only one of them invalid, different gangs
  if (!x.valid() && !y.valid())
    return 0;
  if (!x.valid() || !y.valid())
    return -1;
#endif

  // if gang ids match, then they are the same gang
  if (x.get_gang_id()==y.get_gang_id())
    return 0;

  // if both gangs have one location and the same owner, they are similar
  if ((x.get_num_locations()==1) &&
      (y.get_num_locations()==1) &&
      (gang_md_registry::id_owner(x.get_gang_id()) ==
       gang_md_registry::id_owner(y.get_gang_id())))
    return 1;

  // check if gang metadata is conformant
  auto const* gx = gang_md_registry::try_get(x.get_gang_id());
  if (!gx)
    return -1;
  auto const* gy = gang_md_registry::try_get(y.get_gang_id());
  if (!gy)
    return -1;
  if (gx->conforms_with(*gy))
    return 1;

  return -1;
}


// --------------------------------------------------------------------------
// Aggregation related primitives
// --------------------------------------------------------------------------

// Sets the aggregation
unsigned int set_aggregation(const unsigned int aggregation) noexcept
{
  const unsigned int max_aggregation =
    (message::default_body_capacity() / rmi_request::minimum_size());
  aggregation_factor = std::min(aggregation, max_aggregation);
  if (aggregation_factor==0)
    STAPL_RUNTIME_ERROR("Insufficient aggregation. Increase "
                        "STAPL_RUNTIME_MSG_SIZE.");
  return aggregation_factor;
}


// Returns the aggregation
unsigned int get_aggregation(void) noexcept
{
  return aggregation_factor;
}


// Sets the default RMI buffer size
void set_rmi_buffer_size(const std::size_t size)
{
  context* const ctx = this_context::try_get();
  if (!ctx || !ctx->is_base() || ctx->get_gang_id()!=0)
    STAPL_RUNTIME_ERROR("Can only be called by the application entry point");

  location_md& l = ctx->get_location_md();
  if (l.is_leader()) {
    communicator::set_default_message_size(size);
    set_aggregation(std::numeric_limits<unsigned int>::max());
    STAPL_RUNTIME_STATISTICS("maximum aggregation", get_aggregation());
  }
  barrier_object b{*ctx};
  b();
  b.wait();
}


// Flushes the aggregation buffers
void rmi_flush(void)
{
  STAPL_RUNTIME_PROFILE("rmi_flush()", (primitive_traits::non_blocking |
                                        primitive_traits::p2m          |
                                        primitive_traits::comm));
  context* const ctx = this_context::try_get();
  if (ctx)
    ctx->flush_requests();
}


// --------------------------------------------------------------------------
// Polling primitives
// --------------------------------------------------------------------------

// Executes pending requests
void rmi_poll(void)
{
  STAPL_RUNTIME_PROFILE("rmi_poll()", primitive_traits::yield);
  context* const ctx = this_context::try_get();
  if (ctx)
    ctx->flush_requests();
  runqueue::yield();
}


namespace runtime {

// Executes some pending requests
void scheduling_point(context& ctx)
{
  location_md& l       = ctx.get_location_md();
  const auto intensity = l.get_runqueue().yield_intensity();
  if (intensity==0)
    return;
  STAPL_RUNTIME_PROFILE("scheduling_point()", primitive_traits::yield);
  runqueue::yield(intensity);
}

} // namespace runtime


// --------------------------------------------------------------------------
// Synchronization primitives
// --------------------------------------------------------------------------

// Synchronization with no communication
void rmi_synchronize(void)
{
  STAPL_RUNTIME_PROFILE("rmi_synchronize()", (primitive_traits::non_blocking |
                                              primitive_traits::coll         |
                                              primitive_traits::sync));
  context* const ctx = this_context::try_get();
  if (!ctx || ctx->get_gang_md().size()==1)
    return;
  STAPL_RUNTIME_ASSERT_MSG(ctx->is_base(), "Only allowed in SPMD");
  ctx->flush_requests();
  ctx->get_location_md().advance_epoch();
}


// Barrier, blocks until all locations have reached it
void rmi_barrier(void)
{
  STAPL_RUNTIME_PROFILE("rmi_barrier()", (primitive_traits::blocking |
                                          primitive_traits::comm     |
                                          primitive_traits::coll     |
                                          primitive_traits::sync));
  context* const ctx = this_context::try_get();
  if (!ctx || ctx->get_gang_md().size()==1)
    return;
  STAPL_RUNTIME_ASSERT_MSG(ctx->is_base(), "Only allowed in SPMD");
  ctx->flush_requests();
  location_md& l = ctx->get_location_md();
  barrier_object b{*ctx};
  b();
  b.wait();
  l.advance_epoch();
}


// Fence, blocks until all RMIs have been executed
void rmi_fence(void)
{
  STAPL_RUNTIME_PROFILE("rmi_fence()", (primitive_traits::blocking |
                                        primitive_traits::comm     |
                                        primitive_traits::coll     |
                                        primitive_traits::sync));
  context* const ctx = this_context::try_get();
  if (!ctx)
    return;
  STAPL_RUNTIME_ASSERT_MSG(ctx->is_base(), "Only allowed in SPMD");
  rmi_fence(*ctx);
}


// --------------------------------------------------------------------------
// Environment primitives
// --------------------------------------------------------------------------

// Initializes the runtime
void initialize(option opts)
{
  if (is_initialized())
    STAPL_RUNTIME_ERROR("STAPL Runtime is already initialized");

#ifdef STAPL_USE_PAPI
  if (PAPI_is_initialized()!=PAPI_LOW_LEVEL_INITED)
    STAPL_RUNTIME_CHECK(PAPI_library_init(PAPI_VER_CURRENT)==PAPI_VER_CURRENT,
                        "PAPI initialization failed");
#endif

#ifdef STAPL_RUNTIME_DEBUG
  debug_level = opts.get<int>("STAPL_RUNTIME_DEBUG_LEVEL", 1);
#else
  debug_level = opts.get<int>("STAPL_RUNTIME_DEBUG_LEVEL", 0);
#endif

  executor_window_size =
    opts.get<std::size_t>("STAPL_EXECUTOR_WINDOW_SIZE",
                          STAPL_EXECUTOR_DEFAULT_WINDOW_SIZE);
  executor_retire_chunk =
    opts.get<std::size_t>("STAPL_EXECUTOR_RETIRE_CHUNK",
                          STAPL_EXECUTOR_DEFAULT_RETIRE_CHUNK);

  runqueue::initialize(opts);
  set_aggregation(std::numeric_limits<unsigned int>::max());
  STAPL_RUNTIME_STATISTICS("maximum aggregation", get_aggregation());

  auto widths = concurrency::get_level_widths();
  hierarchy_widths.resize(widths.size() + 1);
  hierarchy_widths[0] = runqueue::get_num_processes();
  std::copy(std::begin(widths), std::end(widths), &hierarchy_widths[1]);

  initialized = true;
}


// Finalizes the runtime
void finalize(void)
{
  if (!is_initialized())
    STAPL_RUNTIME_ERROR("STAPL Runtime is not initialized");
  initialized = false;

  runqueue::finalize();
}


// Returns if the runtime is initialized
bool is_initialized(void) noexcept
{
  return initialized;
}


// Returns the hierarchy widths
std::vector<unsigned int> const& get_hierarchy_widths(void) noexcept
{
  return hierarchy_widths;
}


namespace runtime {

// Returns the debug level
int get_debug_level(void) noexcept
{
  return debug_level;
}

} // namespace runtime


// --------------------------------------------------------------------------
// Execution primitives
// --------------------------------------------------------------------------

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Function object to create a new threads-only environment.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class nested_wf
{
private:
  using function_type = std::function<void(void)>;

  function_type     m_f;
  const gang_md::id m_parent_id;
  gang_md*          m_g;

public:
  template<typename Function>
  nested_wf(Function&& f, const gang_md::id parent_id)
  : m_f(std::forward<Function>(f)),
    m_parent_id(parent_id),
    m_g(nullptr)
  { }

  void initialize(unsigned int nth)
  {
    if (nth>1)
      m_g = new gang_md{m_parent_id, nth};
  }

  void operator()(unsigned int tid)
  {
    if (m_g) {
      location_md* const l = new location_md{tid, *m_g};
      context ctx{*l};
      function_type{m_f}();
    }
    else {
      gang g;
      m_f();
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object to create a new distributed or mixed-mode
///        environment.
///
/// The function operator is effectively the program entry point.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class global_wf
{
private:
  using function_type = std::function<void(void)>;

  function_type m_f;
  gang_md*      m_g;

public:
  template<typename Function>
  explicit global_wf(Function&& f)
  : m_f(std::forward<Function>(f)),
    m_g(nullptr)
  { }

  void initialize(unsigned int nth)
  {
    using size_type = gang_md::size_type;

    using stapl::runtime::identity;
    using stapl::runtime::blocked;

    const size_type npids = runqueue::get_num_processes();

    if (nth==1) {
      // 1 location per process
      gang_description gd{
        identity<location_md::id, process_id>{}, npids,
        identity<size_type, process_id>{}, npids
      };
      STAPL_RUNTIME_ASSERT((gd.get_num_locations()==npids) &&
                           (gd.get_num_processes()==npids));
      m_g = new gang_md{std::move(gd), 1};
    }
    else {
      // nth locations per process (mixed-mode)
      const size_type nlocs = (nth * npids);
      gang_description gd{
        blocked<location_md::id, process_id>{process_id(nth)}, nlocs,
        identity<size_type, process_id>{}, npids
      };
      STAPL_RUNTIME_ASSERT((gd.get_num_locations()==nlocs) &&
                           (gd.get_num_processes()==npids));
      m_g = new gang_md{std::move(gd), nth};
    }
  }

  void operator()(unsigned int tid)
  {
    const location_md::id mylid =
      (runqueue::get_process_id() * m_g->local_size() + tid);
    location_md* const l = new location_md{mylid, tid, *m_g};
    context ctx{*l};
    function_type{m_f}();
    if (!l->unique()) {
      STAPL_RUNTIME_ERROR("Cannot delete metadata for program entry point. "
                          "There might be p_objects still registered.");
    }
  }
};

} // namespace runtime


// Returns the available parallelism levels
unsigned int get_available_levels(void) noexcept
{
  if (!consumed_1st_level)
    return (concurrency::available_levels() + 1);
  return concurrency::available_levels();
}


// Starts a new environment
void execute(std::function<void(void)> f, unsigned int n)
{
  STAPL_RUNTIME_PROFILE("execute()", (primitive_traits::blocking |
                                      primitive_traits::environment));

  if (!consumed_1st_level) {
    global_wf wf{std::move(f)};
    consumed_1st_level = true;
    if (n==1) {
      wf.initialize(1);
      wf(0);
    }
    else {
      concurrency::fork([&wf](unsigned int nth)
                        { wf.initialize(nth); },
                        [&wf](unsigned int tid)
                        { wf(tid); },
                        n);
    }
    consumed_1st_level = false;
  }
  else {
    nested_wf wf{std::move(f), this_context::get().get_gang_id()};
    concurrency::fork([&wf](unsigned int nth)
                      { wf.initialize(nth); },
                      [&wf](unsigned int tid)
                      { wf(tid); },
                      n);
  }
}


// Returns the executor of the current gang
executor_base& get_executor(void)
{
  return this_context::get().get_location_md().get_executor();
}


// Returns the default executor window size
std::size_t get_default_executor_window_size(void) noexcept
{
  return executor_window_size;
}


// Returns the default executor retire chunk
std::size_t get_default_executor_retire_chunk(void) noexcept
{
  return executor_retire_chunk;
}

} // namespace stapl
